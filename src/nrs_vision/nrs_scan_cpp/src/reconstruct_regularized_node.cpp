#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/memory.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>

namespace fs = std::filesystem;

class ReconstructRegularizedNode final : public rclcpp::Node
{
public:
  ReconstructRegularizedNode()
  : Node("reconstruct_regularized_node")
  {
    scan_dir_ = declare_parameter<std::string>("scan_dir", "");
    view_filename_ =
      declare_parameter<std::string>("view_filename", "cloud_world_roi.pcd");

    voxel_leaf_size_m_ =
      declare_parameter<double>("voxel_leaf_size_m", 0.003);

    enable_sor_ =
      declare_parameter<bool>("enable_sor", true);
    sor_mean_k_ =
      declare_parameter<int>("sor_mean_k", 20);
    sor_stddev_mul_ =
      declare_parameter<double>("sor_stddev_mul", 3.0);

    normal_radius_m_ =
      declare_parameter<double>("normal_radius_m", 0.012);

    gp3_search_radius_m_ =
      declare_parameter<double>("gp3_search_radius_m", 0.016);
    gp3_mu_ =
      declare_parameter<double>("gp3_mu", 3.5);
    gp3_max_neighbors_ =
      declare_parameter<int>("gp3_max_neighbors", 150);
    gp3_max_surface_angle_deg_ =
      declare_parameter<double>("gp3_max_surface_angle_deg", 75.0);
    gp3_min_triangle_angle_deg_ =
      declare_parameter<double>("gp3_min_triangle_angle_deg", 5.0);
    gp3_max_triangle_angle_deg_ =
      declare_parameter<double>("gp3_max_triangle_angle_deg", 150.0);
    gp3_normal_consistency_ =
      declare_parameter<bool>("gp3_normal_consistency", false);

    enable_mesh_regularization_ =
      declare_parameter<bool>("enable_mesh_regularization", true);
    mesh_smooth_iterations_ =
      declare_parameter<int>("mesh_smooth_iterations", 20);
    mesh_smooth_passband_ =
      declare_parameter<double>("mesh_smooth_passband", 0.10);
    mesh_normalize_coordinates_ =
      declare_parameter<bool>("mesh_normalize_coordinates", true);
    mesh_boundary_smoothing_ =
      declare_parameter<bool>("mesh_boundary_smoothing", false);
    mesh_feature_edge_smoothing_ =
      declare_parameter<bool>("mesh_feature_edge_smoothing", false);
    mesh_feature_angle_deg_ =
      declare_parameter<double>("mesh_feature_angle_deg", 60.0);
    mesh_edge_angle_deg_ =
      declare_parameter<double>("mesh_edge_angle_deg", 15.0);
    mesh_target_faces_ =
      declare_parameter<int>("mesh_target_faces", 3000);

    merged_raw_name_ =
      declare_parameter<std::string>("merged_raw_name", "merged_raw_regularized.pcd");
    merged_filtered_name_ =
      declare_parameter<std::string>(
      "merged_filtered_name", "merged_filtered_regularized.pcd");
    gp3_raw_name_ =
      declare_parameter<std::string>("gp3_raw_name", "reconstructed_gp3_raw.stl");
    gp3_smoothed_name_ =
      declare_parameter<std::string>(
      "gp3_smoothed_name", "reconstructed_gp3_smoothed.stl");
    gp3_regularized_name_ =
      declare_parameter<std::string>(
      "gp3_regularized_name", "reconstructed_gp3_regularized.stl");
  }

  void run()
  {
    validateParameters();

    const fs::path scan_dir(scan_dir_);
    const fs::path views_dir = scan_dir / "views";

    if (!fs::exists(views_dir) || !fs::is_directory(views_dir)) {
      throw std::runtime_error(
              "views directory not found: " + views_dir.string());
    }

    const std::vector<fs::path> view_files =
      collectViewFiles(views_dir, view_filename_);

    if (view_files.empty()) {
      throw std::runtime_error(
              "No view PCD files named '" + view_filename_ +
              "' were found under: " + views_dir.string());
    }

    RCLCPP_INFO(
      get_logger(), "Found %zu view clouds.", view_files.size());

    auto merged_raw = loadAndMerge(view_files);
    saveCloud(scan_dir / merged_raw_name_, merged_raw);

    auto filtered = voxelDownsample(merged_raw, voxel_leaf_size_m_);

    if (enable_sor_) {
      filtered = applySor(filtered, sor_mean_k_, sor_stddev_mul_);
    }

    if (filtered->size() < 100U) {
      throw std::runtime_error(
              "Filtered cloud contains fewer than 100 points.");
    }

    saveCloud(scan_dir / merged_filtered_name_, filtered);

    RCLCPP_INFO(
      get_logger(),
      "Point counts: merged_raw=%zu, filtered=%zu",
      merged_raw->size(), filtered->size());

    auto cloud_with_normals = estimatePointNormals(
      filtered, normal_radius_m_);

    if (cloud_with_normals->size() < 100U) {
      throw std::runtime_error(
              "Fewer than 100 valid PointNormal samples remain.");
    }

    pcl::PolygonMesh gp3_mesh = reconstructGp3(cloud_with_normals);

    if (gp3_mesh.polygons.empty()) {
      throw std::runtime_error("GP3 produced an empty mesh.");
    }

    saveMesh(scan_dir / gp3_raw_name_, gp3_mesh);

    pcl::PolygonMesh final_mesh = gp3_mesh;

    if (enable_mesh_regularization_) {
      pcl::PolygonMesh smoothed_mesh = smoothMesh(gp3_mesh);
      saveMesh(scan_dir / gp3_smoothed_name_, smoothed_mesh);
      final_mesh = decimateMesh(smoothed_mesh, mesh_target_faces_);
    }

    saveMesh(scan_dir / gp3_regularized_name_, final_mesh);

    RCLCPP_INFO(
      get_logger(),
      "Faces: GP3 raw=%zu, final=%zu",
      gp3_mesh.polygons.size(), final_mesh.polygons.size());

    RCLCPP_INFO(
      get_logger(),
      "Done. Regularized STL: %s",
      (scan_dir / gp3_regularized_name_).c_str());
  }

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;
  using PointNormalT = pcl::PointNormal;
  using CloudNormalT = pcl::PointCloud<PointNormalT>;

  static constexpr double kPi = 3.14159265358979323846;

  static double degToRad(const double degrees)
  {
    return degrees * kPi / 180.0;
  }

  void validateParameters() const
  {
    if (scan_dir_.empty()) {
      throw std::runtime_error(
              "Parameter 'scan_dir' is empty. Pass -p scan_dir:=<path>.");
    }

    if (voxel_leaf_size_m_ <= 0.0) {
      throw std::runtime_error(
              "voxel_leaf_size_m must be greater than zero.");
    }

    if (normal_radius_m_ <= voxel_leaf_size_m_) {
      throw std::runtime_error(
              "normal_radius_m should be larger than voxel_leaf_size_m.");
    }

    if (gp3_search_radius_m_ <= voxel_leaf_size_m_) {
      throw std::runtime_error(
              "gp3_search_radius_m should be larger than voxel_leaf_size_m.");
    }

    if (gp3_mu_ <= 0.0) {
      throw std::runtime_error("gp3_mu must be greater than zero.");
    }

    if (mesh_smooth_iterations_ < 0) {
      throw std::runtime_error(
              "mesh_smooth_iterations must be non-negative.");
    }

    if (mesh_smooth_passband_ <= 0.0 ||
      mesh_smooth_passband_ > 2.0)
    {
      throw std::runtime_error(
              "mesh_smooth_passband must be in the range (0, 2].");
    }
  }

  std::vector<fs::path> collectViewFiles(
    const fs::path & root,
    const std::string & filename) const
  {
    std::vector<fs::path> files;

    for (const auto & entry : fs::recursive_directory_iterator(root)) {
      if (entry.is_regular_file() &&
        entry.path().filename() == filename)
      {
        files.push_back(entry.path());
      }
    }

    std::sort(files.begin(), files.end());
    return files;
  }

  CloudT::Ptr loadAndMerge(
    const std::vector<fs::path> & view_files) const
  {
    auto merged = pcl::make_shared<CloudT>();

    for (const auto & file : view_files) {
      pcl::PCLPointCloud2 blob;

      if (pcl::io::loadPCDFile(file.string(), blob) < 0) {
        throw std::runtime_error(
                "Failed to load PCD: " + file.string());
      }

      CloudT cloud;
      pcl::fromPCLPointCloud2(blob, cloud);

      CloudT clean;
      std::vector<int> valid_indices;
      pcl::removeNaNFromPointCloud(cloud, clean, valid_indices);

      *merged += clean;

      RCLCPP_INFO(
        get_logger(),
        "Loaded %s points=%zu",
        file.parent_path().filename().c_str(),
        clean.size());
    }

    if (merged->empty()) {
      throw std::runtime_error("Merged point cloud is empty.");
    }

    merged->width = static_cast<std::uint32_t>(merged->size());
    merged->height = 1U;
    merged->is_dense = true;
    return merged;
  }

  CloudT::Ptr voxelDownsample(
    const CloudT::ConstPtr & input,
    const double leaf_size) const
  {
    auto output = pcl::make_shared<CloudT>();

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input);

    const float leaf = static_cast<float>(leaf_size);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*output);
    return output;
  }

  CloudT::Ptr applySor(
    const CloudT::ConstPtr & input,
    const int mean_k,
    const double stddev_mul) const
  {
    if (input->size() <= static_cast<std::size_t>(mean_k)) {
      throw std::runtime_error(
              "SOR mean_k is not smaller than the cloud size.");
    }

    auto output = pcl::make_shared<CloudT>();

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(input);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul);
    sor.filter(*output);
    return output;
  }

  CloudNormalT::Ptr estimatePointNormals(
    const CloudT::ConstPtr & input,
    const double radius) const
  {
    auto normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();

    pcl::NormalEstimation<PointT, pcl::Normal> estimator;
    estimator.setInputCloud(input);
    estimator.setSearchMethod(
      pcl::make_shared<pcl::search::KdTree<PointT>>());
    estimator.setRadiusSearch(radius);
    estimator.compute(*normals);

    if (normals->size() != input->size()) {
      throw std::runtime_error(
              "Normal count does not match point count.");
    }

    auto output = pcl::make_shared<CloudNormalT>();
    output->reserve(input->size());

    for (std::size_t i = 0; i < input->size(); ++i) {
      const PointT & point = input->points[i];
      const pcl::Normal & normal = normals->points[i];

      if (!pcl::isFinite(point) || !pcl::isFinite(normal)) {
        continue;
      }

      PointNormalT point_normal;
      point_normal.x = point.x;
      point_normal.y = point.y;
      point_normal.z = point.z;
      point_normal.normal_x = normal.normal_x;
      point_normal.normal_y = normal.normal_y;
      point_normal.normal_z = normal.normal_z;
      point_normal.curvature = normal.curvature;
      output->push_back(point_normal);
    }

    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1U;
    output->is_dense = true;
    return output;
  }

  pcl::PolygonMesh reconstructGp3(
    const CloudNormalT::ConstPtr & input) const
  {
    pcl::GreedyProjectionTriangulation<PointNormalT> gp3;

    gp3.setInputCloud(input);
    gp3.setSearchMethod(
      pcl::make_shared<pcl::search::KdTree<PointNormalT>>());

    gp3.setSearchRadius(gp3_search_radius_m_);
    gp3.setMu(gp3_mu_);
    gp3.setMaximumNearestNeighbors(gp3_max_neighbors_);
    gp3.setMaximumSurfaceAngle(
      degToRad(gp3_max_surface_angle_deg_));
    gp3.setMinimumAngle(
      degToRad(gp3_min_triangle_angle_deg_));
    gp3.setMaximumAngle(
      degToRad(gp3_max_triangle_angle_deg_));
    gp3.setNormalConsistency(gp3_normal_consistency_);

    pcl::PolygonMesh mesh;
    gp3.reconstruct(mesh);
    return mesh;
  }

  pcl::PolygonMesh smoothMesh(
    const pcl::PolygonMesh & input) const
  {
    if (mesh_smooth_iterations_ == 0) {
      return input;
    }

    pcl::MeshSmoothingWindowedSincVTK smoother;
    smoother.setInputMesh(
      pcl::make_shared<pcl::PolygonMesh>(input));
    smoother.setNumIter(mesh_smooth_iterations_);
    smoother.setPassBand(
      static_cast<float>(mesh_smooth_passband_));
    smoother.setNormalizeCoordinates(
      mesh_normalize_coordinates_);
    smoother.setBoundarySmoothing(
      mesh_boundary_smoothing_);
    smoother.setFeatureEdgeSmoothing(
      mesh_feature_edge_smoothing_);
    smoother.setFeatureAngle(
      static_cast<float>(mesh_feature_angle_deg_));
    smoother.setEdgeAngle(
      static_cast<float>(mesh_edge_angle_deg_));

    pcl::PolygonMesh output;
    smoother.process(output);

    if (output.polygons.empty()) {
      throw std::runtime_error(
              "Windowed-Sinc smoothing produced an empty mesh.");
    }
    return output;
  }

  pcl::PolygonMesh decimateMesh(
    const pcl::PolygonMesh & input,
    const int target_faces) const
  {
    const std::size_t current_faces = input.polygons.size();

    if (target_faces <= 0 ||
      current_faces <= static_cast<std::size_t>(target_faces))
    {
      return input;
    }

    float reduction =
      1.0F -
      static_cast<float>(target_faces) /
      static_cast<float>(current_faces);

    reduction = std::clamp(reduction, 0.0F, 0.99F);

    pcl::MeshQuadricDecimationVTK decimator;
    decimator.setInputMesh(
      pcl::make_shared<pcl::PolygonMesh>(input));
    decimator.setTargetReductionFactor(reduction);

    pcl::PolygonMesh output;
    decimator.process(output);

    if (output.polygons.empty()) {
      throw std::runtime_error(
              "Quadric decimation produced an empty mesh.");
    }
    return output;
  }

  void saveCloud(
    const fs::path & output_path,
    const CloudT::ConstPtr & cloud) const
  {
    if (pcl::io::savePCDFileBinary(
        output_path.string(), *cloud) < 0)
    {
      throw std::runtime_error(
              "Failed to save PCD: " + output_path.string());
    }

    RCLCPP_INFO(
      get_logger(),
      "Saved PCD: %s (%zu points)",
      output_path.c_str(), cloud->size());
  }

  void saveMesh(
    const fs::path & output_path,
    const pcl::PolygonMesh & mesh) const
  {
    if (pcl::io::savePolygonFileSTL(
        output_path.string(), mesh) <= 0)
    {
      throw std::runtime_error(
              "Failed to save STL: " + output_path.string());
    }

    RCLCPP_INFO(
      get_logger(),
      "Saved STL: %s (%zu faces)",
      output_path.c_str(), mesh.polygons.size());
  }

  std::string scan_dir_;
  std::string view_filename_;
  double voxel_leaf_size_m_;
  bool enable_sor_;
  int sor_mean_k_;
  double sor_stddev_mul_;
  double normal_radius_m_;
  double gp3_search_radius_m_;
  double gp3_mu_;
  int gp3_max_neighbors_;
  double gp3_max_surface_angle_deg_;
  double gp3_min_triangle_angle_deg_;
  double gp3_max_triangle_angle_deg_;
  bool gp3_normal_consistency_;
  bool enable_mesh_regularization_;
  int mesh_smooth_iterations_;
  double mesh_smooth_passband_;
  bool mesh_normalize_coordinates_;
  bool mesh_boundary_smoothing_;
  bool mesh_feature_edge_smoothing_;
  double mesh_feature_angle_deg_;
  double mesh_edge_angle_deg_;
  int mesh_target_faces_;
  std::string merged_raw_name_;
  std::string merged_filtered_name_;
  std::string gp3_raw_name_;
  std::string gp3_smoothed_name_;
  std::string gp3_regularized_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ReconstructRegularizedNode>();
    node->run();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("reconstruct_regularized_node"),
      "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
