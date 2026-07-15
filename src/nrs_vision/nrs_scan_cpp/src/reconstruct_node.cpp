#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>


#include <rclcpp/rclcpp.hpp>

namespace fs = std::filesystem;

namespace nrs_scan_cpp
{

class ReconstructNode : public rclcpp::Node
{
public:
  ReconstructNode()
  : Node("nrs_reconstruct_node")
  {
    declare_parameter<std::string>("scan_dir", "/tmp/nrs_scan_cpp");
    declare_parameter<double>("voxel_size_m", 0.003);
    declare_parameter<int>("sor_mean_k", 30);
    declare_parameter<double>("sor_stddev_mul", 1.5);
    declare_parameter<double>("ror_radius_m", 0.010);
    declare_parameter<int>("ror_min_neighbors", 4);
    declare_parameter<double>("normal_radius_m", 0.015);
    declare_parameter<int>("poisson_depth", 9);
    declare_parameter<double>("gp3_search_radius_m", 0.012);
    declare_parameter<double>("gp3_mu", 2.5);
    declare_parameter<int>("gp3_max_neighbors", 100);
    declare_parameter<int>("preview_size_px", 1024);

    scan_dir_ = get_parameter("scan_dir").as_string();
    voxel_size_m_ = get_parameter("voxel_size_m").as_double();
    sor_mean_k_ = static_cast<int>(get_parameter("sor_mean_k").as_int());
    sor_stddev_mul_ = get_parameter("sor_stddev_mul").as_double();
    ror_radius_m_ = get_parameter("ror_radius_m").as_double();
    ror_min_neighbors_ = static_cast<int>(get_parameter("ror_min_neighbors").as_int());
    normal_radius_m_ = get_parameter("normal_radius_m").as_double();
    poisson_depth_ = static_cast<int>(get_parameter("poisson_depth").as_int());
    gp3_search_radius_m_ = get_parameter("gp3_search_radius_m").as_double();
    gp3_mu_ = get_parameter("gp3_mu").as_double();
    gp3_max_neighbors_ = static_cast<int>(get_parameter("gp3_max_neighbors").as_int());
    preview_size_px_ = static_cast<int>(get_parameter("preview_size_px").as_int());
  }

  int run()
  {
    const fs::path root(scan_dir_);
    if (!fs::exists(root)) {
      throw std::runtime_error("scan_dir does not exist: " + root.string());
    }

    std::vector<fs::path> input_files;
    for (const auto & entry : fs::recursive_directory_iterator(root / "views")) {
      if (entry.is_regular_file() && entry.path().filename() == "cloud_world_roi.pcd") {
        input_files.push_back(entry.path());
      }
    }
    std::sort(input_files.begin(), input_files.end());
    if (input_files.empty()) {
      throw std::runtime_error("No views/*/cloud_world_roi.pcd files found");
    }

    auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<std::pair<std::string, std::size_t>> per_view_counts;
    for (const auto & file : input_files) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      if (pcl::io::loadPCDFile(file.string(), cloud) != 0) {
        throw std::runtime_error("Failed to load PCD: " + file.string());
      }
      per_view_counts.emplace_back(file.parent_path().filename().string(), cloud.size());
      *merged += cloud;
      RCLCPP_INFO(
        get_logger(), "LOAD %s points=%zu", file.c_str(), cloud.size());
    }

    merged->width = static_cast<std::uint32_t>(merged->size());
    merged->height = 1;
    merged->is_dense = false;
    const fs::path merged_raw_path = root / "merged_raw.pcd";
    pcl::io::savePCDFileBinary(merged_raw_path.string(), *merged);

    auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(merged);
    voxel.setLeafSize(
      static_cast<float>(voxel_size_m_), static_cast<float>(voxel_size_m_),
      static_cast<float>(voxel_size_m_));
    voxel.filter(*downsampled);
    RCLCPP_INFO(
      get_logger(), "VOXEL %zu -> %zu points", merged->size(), downsampled->size());

    auto statistical = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(downsampled);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_stddev_mul_);
    sor.filter(*statistical);

    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(statistical);
    ror.setRadiusSearch(ror_radius_m_);
    ror.setMinNeighborsInRadius(ror_min_neighbors_);
    ror.filter(*filtered);
    RCLCPP_INFO(
      get_logger(), "FILTER %zu -> %zu points", downsampled->size(), filtered->size());

    if (filtered->size() < 100) {
      throw std::runtime_error("Too few points remain after filtering");
    }

    const fs::path filtered_path = root / "merged_filtered.pcd";
    pcl::io::savePCDFileBinary(filtered_path.string(), *filtered);
    saveTopDownPreview(*filtered, root / "merged_topdown_preview.png");

    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
    auto normal_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    normal_estimator.setInputCloud(filtered);
    normal_estimator.setSearchMethod(normal_tree);
    normal_estimator.setRadiusSearch(normal_radius_m_);
    normal_estimator.compute(*normals);

    auto point_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::concatenateFields(*filtered, *normals, *point_normals);

    pcl::PolygonMesh poisson_mesh;
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(poisson_depth_);
    poisson.setInputCloud(point_normals);
    poisson.reconstruct(poisson_mesh);
    const fs::path poisson_stl = root / "reconstructed_poisson.stl";
    const int poisson_result = pcl::io::savePolygonFileSTL(poisson_stl.string(), poisson_mesh);
    if (poisson_result <= 0) {
      RCLCPP_WARN(get_logger(), "Poisson mesh was empty or failed to save");
    } else {
      RCLCPP_INFO(
        get_logger(), "Poisson STL saved: %s polygons=%zu",
        poisson_stl.c_str(), poisson_mesh.polygons.size());
    }

    pcl::PolygonMesh gp3_mesh;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    auto gp3_tree = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
    gp3.setSearchRadius(gp3_search_radius_m_);
    gp3.setMu(gp3_mu_);
    gp3.setMaximumNearestNeighbors(gp3_max_neighbors_);
    gp3.setMaximumSurfaceAngle(M_PI / 4.0);
    gp3.setMinimumAngle(M_PI / 18.0);
    gp3.setMaximumAngle(2.0 * M_PI / 3.0);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(point_normals);
    gp3.setSearchMethod(gp3_tree);
    gp3.reconstruct(gp3_mesh);
    const fs::path gp3_stl = root / "reconstructed_gp3.stl";
    const int gp3_result = pcl::io::savePolygonFileSTL(gp3_stl.string(), gp3_mesh);
    if (gp3_result <= 0) {
      RCLCPP_WARN(get_logger(), "GP3 mesh was empty or failed to save");
    } else {
      RCLCPP_INFO(
        get_logger(), "GP3 STL saved: %s polygons=%zu",
        gp3_stl.c_str(), gp3_mesh.polygons.size());
    }

    writeReport(
      root / "reconstruction_report.txt", per_view_counts, merged->size(),
      filtered->size(), poisson_mesh.polygons.size(), gp3_mesh.polygons.size());

    RCLCPP_INFO(get_logger(), "Reconstruction complete: %s", root.c_str());
    return 0;
  }

private:
  void saveTopDownPreview(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const fs::path & output_path) const
  {
    pcl::PointXYZ minimum;
    pcl::PointXYZ maximum;
    pcl::getMinMax3D(cloud, minimum, maximum);

    const int size = std::max(256, preview_size_px_);
    cv::Mat height_image(size, size, CV_32FC1, cv::Scalar(-1.0f));
    const double width_x = std::max(1.0e-9, static_cast<double>(maximum.x - minimum.x));
    const double width_y = std::max(1.0e-9, static_cast<double>(maximum.y - minimum.y));
    const double width_z = std::max(1.0e-9, static_cast<double>(maximum.z - minimum.z));

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const int col = static_cast<int>(
        std::round((point.x - minimum.x) / width_x * static_cast<double>(size - 1)));
      const int row = static_cast<int>(
        std::round((maximum.y - point.y) / width_y * static_cast<double>(size - 1)));
      if (row < 0 || row >= size || col < 0 || col >= size) {
        continue;
      }
      const float normalized_height = static_cast<float>(
        std::clamp((point.z - minimum.z) / width_z, 0.0, 1.0));
      height_image.at<float>(row, col) =
        std::max(height_image.at<float>(row, col), normalized_height);
    }

    cv::Mat mask = height_image >= 0.0f;
    cv::Mat gray(size, size, CV_8UC1, cv::Scalar(0));
    height_image.convertTo(gray, CV_8UC1, 255.0);
    gray.setTo(0, mask == 0);
    cv::Mat colour;
    cv::applyColorMap(gray, colour, cv::COLORMAP_TURBO);
    colour.setTo(cv::Scalar(0, 0, 0), mask == 0);
    cv::imwrite(output_path.string(), colour);
  }

  void writeReport(
    const fs::path & path,
    const std::vector<std::pair<std::string, std::size_t>> & view_counts,
    const std::size_t merged_count,
    const std::size_t filtered_count,
    const std::size_t poisson_polygons,
    const std::size_t gp3_polygons) const
  {
    std::ofstream report(path);
    report << "scan_dir " << scan_dir_ << '\n';
    report << "view_count " << view_counts.size() << '\n';
    for (const auto & [name, count] : view_counts) {
      report << "view " << name << " points " << count << '\n';
    }
    report << "merged_point_count " << merged_count << '\n';
    report << "filtered_point_count " << filtered_count << '\n';
    report << "voxel_size_m " << voxel_size_m_ << '\n';
    report << "normal_radius_m " << normal_radius_m_ << '\n';
    report << "poisson_polygon_count " << poisson_polygons << '\n';
    report << "gp3_polygon_count " << gp3_polygons << '\n';
  }

  std::string scan_dir_;
  double voxel_size_m_{0.003};
  int sor_mean_k_{30};
  double sor_stddev_mul_{1.5};
  double ror_radius_m_{0.010};
  int ror_min_neighbors_{4};
  double normal_radius_m_{0.015};
  int poisson_depth_{9};
  double gp3_search_radius_m_{0.012};
  double gp3_mu_{2.5};
  int gp3_max_neighbors_{100};
  int preview_size_px_{1024};
};

}  // namespace nrs_scan_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<nrs_scan_cpp::ReconstructNode>();
    const int result = node->run();
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    std::cerr << "[FATAL] " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
