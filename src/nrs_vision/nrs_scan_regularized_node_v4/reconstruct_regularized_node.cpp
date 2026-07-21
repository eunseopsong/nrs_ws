#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
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

    // 기존 reconstruct_node와 동일한 per-view Z crop
    crop_z_enabled_ =
      declare_parameter<bool>("crop_z_enabled", true);
    crop_min_z_m_ =
      declare_parameter<double>("crop_min_z_m", 0.003);
    crop_max_z_m_ =
      declare_parameter<double>("crop_max_z_m", 0.180);

    enable_sor_ =
      declare_parameter<bool>("enable_sor", true);
    sor_mean_k_ =
      declare_parameter<int>("sor_mean_k", 20);
    sor_stddev_mul_ =
      declare_parameter<double>("sor_stddev_mul", 3.0);

    // 기존 reconstruct_node와 동일한 SOR 이후 ROR
    enable_ror_ =
      declare_parameter<bool>("enable_ror", true);
    ror_radius_m_ =
      declare_parameter<double>("ror_radius_m", 0.010);
    ror_min_neighbors_ =
      declare_parameter<int>("ror_min_neighbors", 4);

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

    // Blender의 Recalculate Outside에 해당하는 최종 face winding 정리.
    enable_mesh_orientation_fix_ =
      declare_parameter<bool>("enable_mesh_orientation_fix", true);
    mesh_orientation_hint_x_ =
      declare_parameter<double>("mesh_orientation_hint_x", 0.0);
    mesh_orientation_hint_y_ =
      declare_parameter<double>("mesh_orientation_hint_y", 0.0);
    mesh_orientation_hint_z_ =
      declare_parameter<double>("mesh_orientation_hint_z", 1.0);

    merged_raw_name_ =
      declare_parameter<std::string>("merged_raw_name", "merged_raw_regularized.pcd");
    z_cropped_merged_name_ =
      declare_parameter<std::string>(
      "z_cropped_merged_name", "merged_z_cropped_regularized.pcd");
    merged_filtered_name_ =
      declare_parameter<std::string>(
      "merged_filtered_name", "merged_filtered_regularized.pcd");
    gp3_raw_name_ =
      declare_parameter<std::string>("gp3_raw_name", "reconstructed_gp3_raw.stl");
    gp3_smoothed_name_ =
      declare_parameter<std::string>(
      "gp3_smoothed_name", "reconstructed_gp3_smoothed.stl");
    gp3_unoriented_name_ =
      declare_parameter<std::string>(
      "gp3_unoriented_name", "reconstructed_gp3_regularized_unoriented.stl");
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

    // loadAndMerge 내부에서 각 view에 기존 노드와 동일한 Z crop 적용
    auto merged_raw = loadAndMerge(view_files);
    saveCloud(scan_dir / merged_raw_name_, merged_raw);
    saveCloud(scan_dir / z_cropped_merged_name_, merged_raw);

    auto filtered = voxelDownsample(
      merged_raw, voxel_leaf_size_m_);

    if (enable_sor_) {
      filtered = applySor(filtered, sor_mean_k_, sor_stddev_mul_);
    }

    if (enable_ror_) {
      filtered = applyRor(
        filtered, ror_radius_m_, ror_min_neighbors_);
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

    // 비교용으로 orientation 수정 전 결과도 남긴다.
    saveMesh(scan_dir / gp3_unoriented_name_, final_mesh);

    if (enable_mesh_orientation_fix_) {
      final_mesh = orientMeshOutside(final_mesh);
    }

    // 경로 생성에는 이 파일을 사용한다.
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

    if (crop_z_enabled_ && crop_min_z_m_ >= crop_max_z_m_) {
      throw std::runtime_error(
              "crop_min_z_m must be smaller than crop_max_z_m.");
    }

    if (enable_ror_) {
      if (ror_radius_m_ <= 0.0) {
        throw std::runtime_error(
                "ror_radius_m must be greater than zero.");
      }
      if (ror_min_neighbors_ < 1) {
        throw std::runtime_error(
                "ror_min_neighbors must be at least 1.");
      }
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

    const double hint_norm = std::sqrt(
      mesh_orientation_hint_x_ * mesh_orientation_hint_x_ +
      mesh_orientation_hint_y_ * mesh_orientation_hint_y_ +
      mesh_orientation_hint_z_ * mesh_orientation_hint_z_);

    if (enable_mesh_orientation_fix_ && hint_norm <= 1.0e-12) {
      throw std::runtime_error(
              "mesh orientation hint vector must not be zero.");
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

      const std::size_t loaded_count = clean.size();
      auto view_cloud = pcl::make_shared<CloudT>(clean);

      if (crop_z_enabled_) {
        auto cropped = pcl::make_shared<CloudT>();

        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(view_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(
          static_cast<float>(crop_min_z_m_),
          static_cast<float>(crop_max_z_m_));
        pass.filter(*cropped);

        view_cloud = cropped;
      }

      *merged += *view_cloud;

      RCLCPP_INFO(
        get_logger(),
        "Loaded %s points=%zu -> %zu after per-view Z crop",
        file.parent_path().filename().c_str(),
        loaded_count,
        view_cloud->size());
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

  CloudT::Ptr applyRor(
    const CloudT::ConstPtr & input,
    const double radius,
    const int min_neighbors) const
  {
    auto output = pcl::make_shared<CloudT>();

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(input);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(min_neighbors);
    ror.filter(*output);

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


  struct Vec3d
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct EdgeKey
  {
    std::uint32_t a{0U};
    std::uint32_t b{0U};

    bool operator==(const EdgeKey & other) const
    {
      return a == other.a && b == other.b;
    }
  };

  struct EdgeKeyHash
  {
    std::size_t operator()(const EdgeKey & edge) const noexcept
    {
      const std::size_t h1 = std::hash<std::uint32_t>{}(edge.a);
      const std::size_t h2 = std::hash<std::uint32_t>{}(edge.b);
      return h1 ^ (h2 + 0x9e3779b9U + (h1 << 6U) + (h1 >> 2U));
    }
  };

  struct EdgeUse
  {
    std::size_t face_index{0U};
    int direction{1};  // +1: min->max, -1: max->min
  };

  struct FaceNeighbor
  {
    std::size_t face_index{0U};
    bool xor_flip{false};
  };

  static Vec3d subtract(const PointT & a, const PointT & b)
  {
    return Vec3d{
      static_cast<double>(a.x - b.x),
      static_cast<double>(a.y - b.y),
      static_cast<double>(a.z - b.z)};
  }

  static Vec3d subtract(const Vec3d & a, const Vec3d & b)
  {
    return Vec3d{a.x - b.x, a.y - b.y, a.z - b.z};
  }

  static Vec3d cross(const Vec3d & a, const Vec3d & b)
  {
    return Vec3d{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x};
  }

  static double dot(const Vec3d & a, const Vec3d & b)
  {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  static double norm(const Vec3d & value)
  {
    return std::sqrt(dot(value, value));
  }

  pcl::PolygonMesh orientMeshOutside(
    const pcl::PolygonMesh & input) const
  {
    if (input.polygons.empty()) {
      throw std::runtime_error(
              "orientMeshOutside received an empty mesh.");
    }

    pcl::PolygonMesh output = input;

    pcl::PointCloud<PointT> vertices;
    pcl::fromPCLPointCloud2(output.cloud, vertices);

    if (vertices.empty()) {
      throw std::runtime_error(
              "Mesh contains no vertices.");
    }

    using EdgeMap =
      std::unordered_map<EdgeKey, std::vector<EdgeUse>, EdgeKeyHash>;

    EdgeMap edge_uses;
    std::vector<bool> valid_face(output.polygons.size(), true);
    std::size_t invalid_face_count = 0U;

    // 1. 모든 polygon edge의 방향을 기록한다.
    for (std::size_t face_index = 0U;
      face_index < output.polygons.size();
      ++face_index)
    {
      const auto & polygon = output.polygons[face_index];
      const auto & ids = polygon.vertices;

      if (ids.size() < 3U) {
        valid_face[face_index] = false;
        ++invalid_face_count;
        continue;
      }

      bool indices_valid = true;
      for (const std::uint32_t id : ids) {
        if (id >= vertices.size()) {
          indices_valid = false;
          break;
        }
      }

      if (!indices_valid) {
        valid_face[face_index] = false;
        ++invalid_face_count;
        continue;
      }

      for (std::size_t i = 0U; i < ids.size(); ++i) {
        const std::uint32_t u = ids[i];
        const std::uint32_t v = ids[(i + 1U) % ids.size()];

        if (u == v) {
          continue;
        }

        const EdgeKey key{
          std::min(u, v),
          std::max(u, v)};

        const int direction =
          (u == key.a && v == key.b) ? 1 : -1;

        edge_uses[key].push_back(
          EdgeUse{face_index, direction});
      }
    }

    // 2. 공유 edge를 따라 이웃 face winding이 서로 반대가 되도록
    //    XOR 제약 그래프를 만든다.
    std::vector<std::vector<FaceNeighbor>> adjacency(
      output.polygons.size());

    std::size_t boundary_edge_count = 0U;
    std::size_t nonmanifold_edge_count = 0U;

    for (const auto & [edge, uses] : edge_uses) {
      (void)edge;

      if (uses.size() == 1U) {
        ++boundary_edge_count;
        continue;
      }

      if (uses.size() > 2U) {
        ++nonmanifold_edge_count;
      }

      // 비매니폴드 edge도 첫 face를 기준으로 가능한 만큼 정렬한다.
      const EdgeUse & reference = uses.front();

      for (std::size_t i = 1U; i < uses.size(); ++i) {
        const EdgeUse & current = uses[i];

        // 원래 같은 방향으로 edge를 순회하면 둘 중 하나를 뒤집어야 한다.
        const bool xor_flip =
          reference.direction == current.direction;

        adjacency[reference.face_index].push_back(
          FaceNeighbor{current.face_index, xor_flip});
        adjacency[current.face_index].push_back(
          FaceNeighbor{reference.face_index, xor_flip});
      }
    }

    // 3. 연결 성분별 BFS로 local winding을 일관화한다.
    std::vector<int> face_flip(output.polygons.size(), -1);
    std::vector<int> component_id(output.polygons.size(), -1);
    std::vector<std::vector<std::size_t>> components;

    std::size_t winding_conflict_count = 0U;

    for (std::size_t start = 0U;
      start < output.polygons.size();
      ++start)
    {
      if (!valid_face[start] || face_flip[start] != -1) {
        continue;
      }

      const int current_component =
        static_cast<int>(components.size());

      components.emplace_back();
      std::queue<std::size_t> pending;

      face_flip[start] = 0;
      component_id[start] = current_component;
      pending.push(start);

      while (!pending.empty()) {
        const std::size_t face_index = pending.front();
        pending.pop();

        components.back().push_back(face_index);

        for (const FaceNeighbor & neighbor :
          adjacency[face_index])
        {
          if (!valid_face[neighbor.face_index]) {
            continue;
          }

          const int expected_flip =
            face_flip[face_index] ^
            static_cast<int>(neighbor.xor_flip);

          if (face_flip[neighbor.face_index] == -1) {
            face_flip[neighbor.face_index] = expected_flip;
            component_id[neighbor.face_index] = current_component;
            pending.push(neighbor.face_index);
          } else if (
            face_flip[neighbor.face_index] != expected_flip)
          {
            ++winding_conflict_count;
          }
        }
      }
    }

    std::size_t locally_flipped_faces = 0U;

    for (std::size_t i = 0U; i < output.polygons.size(); ++i) {
      if (face_flip[i] == 1) {
        std::reverse(
          output.polygons[i].vertices.begin(),
          output.polygons[i].vertices.end());
        ++locally_flipped_faces;
      }
    }

    // 방향 힌트. 열린 상부 가공면에서 안/밖 판정이 애매할 때 사용.
    Vec3d orientation_hint{
      mesh_orientation_hint_x_,
      mesh_orientation_hint_y_,
      mesh_orientation_hint_z_};

    const double hint_length = norm(orientation_hint);
    orientation_hint.x /= hint_length;
    orientation_hint.y /= hint_length;
    orientation_hint.z /= hint_length;

    // 4. 연결 성분 전체가 안쪽을 향하면 component 전체를 뒤집는다.
    std::size_t globally_flipped_components = 0U;

    for (std::size_t component = 0U;
      component < components.size();
      ++component)
    {
      const auto & faces = components[component];

      if (faces.empty()) {
        continue;
      }

      std::unordered_set<std::uint32_t> component_vertices;

      for (const std::size_t face_index : faces) {
        for (const std::uint32_t vertex_id :
          output.polygons[face_index].vertices)
        {
          component_vertices.insert(vertex_id);
        }
      }

      Vec3d centroid;

      for (const std::uint32_t id : component_vertices) {
        centroid.x += vertices[id].x;
        centroid.y += vertices[id].y;
        centroid.z += vertices[id].z;
      }

      const double vertex_count =
        static_cast<double>(component_vertices.size());

      centroid.x /= vertex_count;
      centroid.y /= vertex_count;
      centroid.z /= vertex_count;

      bool component_closed = true;

      for (const auto & [edge, uses] : edge_uses) {
        (void)edge;
        std::size_t uses_in_component = 0U;

        for (const EdgeUse & use : uses) {
          if (
            use.face_index < component_id.size() &&
            component_id[use.face_index] ==
            static_cast<int>(component))
          {
            ++uses_in_component;
          }
        }

        if (uses_in_component != 0U &&
          uses_in_component != 2U)
        {
          component_closed = false;
          break;
        }
      }

      double signed_volume = 0.0;
      double radial_score = 0.0;
      double hint_score = 0.0;
      double doubled_area_sum = 0.0;

      for (const std::size_t face_index : faces) {
        const auto & ids =
          output.polygons[face_index].vertices;

        if (ids.size() < 3U) {
          continue;
        }

        const PointT & p0 = vertices[ids[0]];

        for (std::size_t k = 1U; k + 1U < ids.size(); ++k) {
          const PointT & p1 = vertices[ids[k]];
          const PointT & p2 = vertices[ids[k + 1U]];

          const Vec3d e1 = subtract(p1, p0);
          const Vec3d e2 = subtract(p2, p0);
          const Vec3d area_normal = cross(e1, e2);

          const Vec3d p0_vec{
            static_cast<double>(p0.x),
            static_cast<double>(p0.y),
            static_cast<double>(p0.z)};

          const Vec3d triangle_center{
            (
              static_cast<double>(p0.x) +
              static_cast<double>(p1.x) +
              static_cast<double>(p2.x)) / 3.0,
            (
              static_cast<double>(p0.y) +
              static_cast<double>(p1.y) +
              static_cast<double>(p2.y)) / 3.0,
            (
              static_cast<double>(p0.z) +
              static_cast<double>(p1.z) +
              static_cast<double>(p2.z)) / 3.0};

          signed_volume +=
            dot(p0_vec, area_normal) / 6.0;

          radial_score +=
            dot(
            area_normal,
            subtract(triangle_center, centroid));

          hint_score +=
            dot(area_normal, orientation_hint);

          doubled_area_sum += norm(area_normal);
        }
      }

      const double tolerance =
        std::max(1.0e-15, doubled_area_sum * 1.0e-12);

      bool flip_component = false;

      if (component_closed &&
        std::abs(signed_volume) > tolerance)
      {
        // 닫힌 메쉬는 signed volume이 양수가 되도록 정렬.
        flip_component = signed_volume < 0.0;
      } else if (std::abs(radial_score) > tolerance) {
        // 열린 메쉬는 centroid에서 바깥쪽으로 향하는지 판정.
        flip_component = radial_score < 0.0;
      } else if (std::abs(hint_score) > tolerance) {
        // 마지막 fallback: 기본 +Z 방향을 우세하게 만든다.
        flip_component = hint_score < 0.0;
      }

      if (flip_component) {
        for (const std::size_t face_index : faces) {
          std::reverse(
            output.polygons[face_index].vertices.begin(),
            output.polygons[face_index].vertices.end());
        }

        ++globally_flipped_components;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Mesh orientation fix: components=%zu, "
      "local_face_flips=%zu, global_component_flips=%zu, "
      "boundary_edges=%zu, nonmanifold_edges=%zu, "
      "winding_conflicts=%zu, invalid_faces=%zu",
      components.size(),
      locally_flipped_faces,
      globally_flipped_components,
      boundary_edge_count,
      nonmanifold_edge_count,
      winding_conflict_count,
      invalid_face_count);

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
  bool crop_z_enabled_;
  double crop_min_z_m_;
  double crop_max_z_m_;
  bool enable_sor_;
  int sor_mean_k_;
  double sor_stddev_mul_;
  bool enable_ror_;
  double ror_radius_m_;
  int ror_min_neighbors_;
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
  bool enable_mesh_orientation_fix_;
  double mesh_orientation_hint_x_;
  double mesh_orientation_hint_y_;
  double mesh_orientation_hint_z_;
  std::string merged_raw_name_;
  std::string z_cropped_merged_name_;
  std::string merged_filtered_name_;
  std::string gp3_raw_name_;
  std::string gp3_smoothed_name_;
  std::string gp3_unoriented_name_;
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
