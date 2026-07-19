#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif
#if __has_include(<tf2_ros/transform_listener.hpp>)
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/transform_listener.h>
#endif
#include <y2_rob_motion_interfaces/srv/single_arm_command.hpp>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace nrs_scan_cpp
{

struct ViewPlan
{
  int index{};
  std::string name;
  double azimuth_deg{};
  double elevation_deg{};
  Eigen::Vector3d camera_position_world_m{Eigen::Vector3d::Zero()};
  Eigen::Matrix4d t_world_camera_ros_m{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d t_world_tcp_m{Eigen::Matrix4d::Identity()};
  std::vector<double> service_pose_mm_spatial;
};

struct PoseError
{
  double position_m{std::numeric_limits<double>::infinity()};
  double orientation_deg{std::numeric_limits<double>::infinity()};
};

struct ActualPoseResult
{
  Eigen::Matrix4d t_world_pointcloud_m{Eigen::Matrix4d::Identity()};
  geometry_msgs::msg::TransformStamped transform_message;
  std::string world_frame;
  std::string tf_camera_frame;
  std::string pointcloud_frame;
  bool used_latest_fallback{false};
};

static Eigen::Matrix4d matrixFromFlat16(const std::vector<double> & values)
{
  if (values.size() != 16) {
    throw std::runtime_error("Expected a 16-element transform matrix parameter");
  }
  Eigen::Matrix4d matrix;
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
        values[row * 4 + col];
    }
  }
  return matrix;
}

static Eigen::Matrix4d makeTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Matrix3d & rotation)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;
  return transform;
}

static Eigen::Matrix4d makeLookAtCameraPoseRos(
  const Eigen::Vector3d & camera_position,
  const Eigen::Vector3d & target_position,
  const bool top_view)
{
  // ROS optical convention used by the PointCloud2:
  // +Z forward, +X right, +Y down.
  Eigen::Vector3d z_axis = target_position - camera_position;
  const double norm = z_axis.norm();
  if (norm < 1.0e-9) {
    throw std::runtime_error("Camera position and target position are identical");
  }
  z_axis /= norm;

  Eigen::Vector3d up_reference =
    top_view ? Eigen::Vector3d(0.0, 1.0, 0.0) : Eigen::Vector3d(0.0, 0.0, 1.0);

  Eigen::Vector3d x_axis = z_axis.cross(up_reference);
  if (x_axis.norm() < 1.0e-8) {
    up_reference = Eigen::Vector3d(0.0, 1.0, 0.0);
    x_axis = z_axis.cross(up_reference);
  }
  x_axis.normalize();

  Eigen::Vector3d y_axis = z_axis.cross(x_axis);
  y_axis.normalize();

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;
  return makeTransform(camera_position, rotation);
}

static Eigen::Vector3d rotationMatrixToRotationVector(
  const Eigen::Matrix3d & rotation,
  const bool output_degrees)
{
  Eigen::AngleAxisd angle_axis(rotation);
  double angle = angle_axis.angle();
  Eigen::Vector3d axis = angle_axis.axis();

  if (!std::isfinite(angle) || !axis.allFinite() || std::abs(angle) < 1.0e-12) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d vector = axis * angle;
  if (output_degrees) {
    vector *= 180.0 / M_PI;
  }

  if (std::abs(std::abs(angle) - M_PI) < 1.0e-8) {
    for (int i = 0; i < 3; ++i) {
      if (std::abs(vector[i]) > 1.0e-9) {
        if (vector[i] < 0.0) {
          vector = -vector;
        }
        break;
      }
    }
  }
  return vector;
}

static std::string matrixToString(const Eigen::Matrix4d & matrix)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << matrix;
  return stream.str();
}

static std::string normalizeFrameId(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

static PoseError calculatePoseError(
  const Eigen::Matrix4d & planned,
  const Eigen::Matrix4d & actual)
{
  PoseError error;
  error.position_m =
    (planned.block<3, 1>(0, 3) - actual.block<3, 1>(0, 3)).norm();

  const Eigen::Matrix3d relative_rotation =
    planned.block<3, 3>(0, 0).transpose() * actual.block<3, 3>(0, 0);
  Eigen::AngleAxisd angle_axis(relative_rotation);
  error.orientation_deg = std::abs(angle_axis.angle()) * 180.0 / M_PI;
  return error;
}

class ScanCaptureNode : public rclcpp::Node
{
public:
  ScanCaptureNode()
  : Node("nrs_scan_capture_node")
  {
    declareParameters();
    readParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    reliable_qos.reliable();
    reliable_qos.durability_volatile();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, reliable_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_cloud_ = std::move(msg);
        ++cloud_sequence_;
      });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, reliable_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_joint_state_ = std::move(msg);
        ++joint_sequence_;
      });

    command_client_ = create_client<y2_rob_motion_interfaces::srv::SingleArmCommand>(
      service_name_);
  }

  int run()
  {
    const auto plans = makePlans();
    printPlans(plans);

    fs::create_directories(output_dir_);
    writePlanFile(plans);

    if (plan_only_) {
      RCLCPP_INFO(get_logger(), "Plan-only mode complete.");
      return 0;
    }

    waitForService();
    waitForSensorsAndTf();

    std::vector<int> indices;
    if (run_all_) {
      for (int i = 0; i < static_cast<int>(plans.size()); ++i) {
        indices.push_back(i);
      }
    } else {
      if (view_index_ < 0 || view_index_ >= static_cast<int>(plans.size())) {
        throw std::runtime_error("view_index is outside the configured view range");
      }
      indices.push_back(view_index_);
    }

    int completed = 0;
    int failed = 0;
    for (const int index : indices) {
      const ViewPlan & plan = plans.at(static_cast<std::size_t>(index));
      try {
        callCommand("PTP", plan.service_pose_mm_spatial);

        if (!waitUntilSettledAndReached(plan)) {
          ++failed;
          callIdlingNoThrow();
          if (!continue_on_failure_) {
            break;
          }
          continue;
        }

        waitWithSpin(capture_delay_after_settle_sec_);
        const std::uint64_t sequence_before_capture = currentCloudSequence();
        auto cloud_message = captureCloudAfter(sequence_before_capture);
        saveView(plan, *cloud_message);
        ++completed;
      } catch (const std::exception & error) {
        ++failed;
        RCLCPP_ERROR(
          get_logger(), "View %s failed: %s", plan.name.c_str(), error.what());
        callIdlingNoThrow();
        if (!continue_on_failure_) {
          break;
        }
      }
    }

    RCLCPP_INFO(
      get_logger(), "DONE completed=%d failed=%d", completed, failed);
    return failed == 0 ? 0 : 2;
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("service_name", "/singleArm_cmd/single_arm_command");
    declare_parameter<std::string>("pointcloud_topic", "/camera");
    declare_parameter<std::string>("joint_state_topic", "/isaac_joint_states");
    declare_parameter<std::string>("output_dir", "/tmp/nrs_scan_cpp");
    declare_parameter<std::string>("spatial_angle_unit", "degree");

    // Actual-pose source. The transform must map the TF camera frame into world.
    // When tf_camera_frame is empty, PointCloud2.header.frame_id is used.
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("tf_camera_frame", "");
    declare_parameter<double>("tf_lookup_timeout_sec", 1.0);
    declare_parameter<bool>("allow_latest_tf_fallback", true);
    declare_parameter<bool>("require_target_reached", true);
    declare_parameter<double>("target_position_tolerance_m", 0.005);
    declare_parameter<double>("target_orientation_tolerance_deg", 1.0);
    declare_parameter<double>("capture_delay_after_settle_sec", 0.20);
    declare_parameter<bool>("save_planned_world_debug", true);

    // T_tf_camera_pointcloud_frame converts the TF-published camera frame into
    // the coordinate frame used by PointCloud2. Keep identity when both frames
    // are the same ROS optical frame. Use Rx(pi) when TF is USD/OpenGL camera
    // (+Y up, -Z forward) and PointCloud2 is ROS optical (+Y down, +Z forward).
    declare_parameter<std::vector<double>>(
      "T_tf_camera_pointcloud_frame",
      {1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0});

    declare_parameter<std::vector<double>>(
      "workpiece_center_m", {0.65, 0.30, 0.06545485});
    declare_parameter<std::vector<double>>(
      "workpiece_size_m", {0.4238959, 0.4238959, 0.1309097});
    declare_parameter<std::vector<double>>(
      "scan_target_offset_from_center_m", {0.0, 0.0, 0.06545485});
    declare_parameter<double>("scan_radius_m", 0.60);
    declare_parameter<double>("bbox_margin_m", 0.02);
    declare_parameter<double>("roi_bottom_offset_m", 0.003);
    declare_parameter<double>("roi_top_margin_m", 0.020);

    declare_parameter<std::vector<double>>(
      "T_ee_tcp_m",
      {-1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.111,
        0.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>(
      "T_ee_camera_opengl_m",
      {1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, -0.150,
        0.0, 0.0, -1.0, -0.020,
        0.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>(
      "T_camera_opengl_camera_ros",
      {1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0, 0.0, 1.0});

    declare_parameter<std::vector<std::string>>(
      "view_names",
      std::vector<std::string>{"top", "upper_225", "lower_180", "lower_270"});
    declare_parameter<std::vector<double>>(
      "view_azimuth_deg", std::vector<double>{0.0, 225.0, 180.0, 270.0});
    declare_parameter<std::vector<double>>(
      "view_elevation_deg", std::vector<double>{90.0, 75.0, 65.0, 65.0});

    declare_parameter<bool>("plan_only", false);
    declare_parameter<bool>("run_all", false);
    declare_parameter<int>("view_index", 0);
    declare_parameter<bool>("continue_on_failure", false);

    declare_parameter<double>("service_wait_timeout_sec", 20.0);
    declare_parameter<double>("service_call_timeout_sec", 45.0);
    declare_parameter<double>("sensor_startup_timeout_sec", 30.0);
    declare_parameter<double>("pointcloud_timeout_sec", 10.0);

    declare_parameter<double>("minimum_settle_sec", 2.0);
    declare_parameter<double>("maximum_settle_sec", 15.0);
    declare_parameter<double>("stable_window_sec", 1.0);
    declare_parameter<double>("joint_position_window_threshold_rad", 0.0015);

    declare_parameter<double>("depth_min_m", 0.05);
    declare_parameter<double>("depth_max_m", 3.0);
    declare_parameter<int>("preview_size_px", 1024);
  }

  static Eigen::Vector3d vector3FromParameter(
    const std::vector<double> & values,
    const std::string & name)
  {
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain exactly three values");
    }
    return {values[0], values[1], values[2]};
  }

  void readParameters()
  {
    service_name_ = get_parameter("service_name").as_string();
    pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    output_dir_ = get_parameter("output_dir").as_string();
    spatial_angle_degrees_ = get_parameter("spatial_angle_unit").as_string() == "degree";

    world_frame_ = normalizeFrameId(get_parameter("world_frame").as_string());
    tf_camera_frame_ = normalizeFrameId(get_parameter("tf_camera_frame").as_string());
    tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
    allow_latest_tf_fallback_ = get_parameter("allow_latest_tf_fallback").as_bool();
    require_target_reached_ = get_parameter("require_target_reached").as_bool();
    target_position_tolerance_m_ = get_parameter("target_position_tolerance_m").as_double();
    target_orientation_tolerance_deg_ =
      get_parameter("target_orientation_tolerance_deg").as_double();
    capture_delay_after_settle_sec_ =
      get_parameter("capture_delay_after_settle_sec").as_double();
    save_planned_world_debug_ = get_parameter("save_planned_world_debug").as_bool();
    t_tf_camera_pointcloud_m_ = matrixFromFlat16(
      get_parameter("T_tf_camera_pointcloud_frame").as_double_array());

    workpiece_center_m_ = vector3FromParameter(
      get_parameter("workpiece_center_m").as_double_array(), "workpiece_center_m");
    workpiece_size_m_ = vector3FromParameter(
      get_parameter("workpiece_size_m").as_double_array(), "workpiece_size_m");
    scan_target_offset_m_ = vector3FromParameter(
      get_parameter("scan_target_offset_from_center_m").as_double_array(),
      "scan_target_offset_from_center_m");
    scan_radius_m_ = get_parameter("scan_radius_m").as_double();
    bbox_margin_m_ = get_parameter("bbox_margin_m").as_double();
    roi_bottom_offset_m_ = get_parameter("roi_bottom_offset_m").as_double();
    roi_top_margin_m_ = get_parameter("roi_top_margin_m").as_double();

    t_ee_tcp_m_ = matrixFromFlat16(get_parameter("T_ee_tcp_m").as_double_array());
    t_ee_camera_gl_m_ = matrixFromFlat16(
      get_parameter("T_ee_camera_opengl_m").as_double_array());
    t_camera_gl_camera_ros_ = matrixFromFlat16(
      get_parameter("T_camera_opengl_camera_ros").as_double_array());

    view_names_ = get_parameter("view_names").as_string_array();
    view_azimuth_deg_ = get_parameter("view_azimuth_deg").as_double_array();
    view_elevation_deg_ = get_parameter("view_elevation_deg").as_double_array();
    if (view_names_.size() != view_azimuth_deg_.size() ||
      view_names_.size() != view_elevation_deg_.size())
    {
      throw std::runtime_error(
              "view_names, view_azimuth_deg, and view_elevation_deg must have equal lengths");
    }

    plan_only_ = get_parameter("plan_only").as_bool();
    run_all_ = get_parameter("run_all").as_bool();
    view_index_ = static_cast<int>(get_parameter("view_index").as_int());
    continue_on_failure_ = get_parameter("continue_on_failure").as_bool();

    service_wait_timeout_sec_ = get_parameter("service_wait_timeout_sec").as_double();
    service_call_timeout_sec_ = get_parameter("service_call_timeout_sec").as_double();
    sensor_startup_timeout_sec_ = get_parameter("sensor_startup_timeout_sec").as_double();
    pointcloud_timeout_sec_ = get_parameter("pointcloud_timeout_sec").as_double();

    minimum_settle_sec_ = get_parameter("minimum_settle_sec").as_double();
    maximum_settle_sec_ = get_parameter("maximum_settle_sec").as_double();
    stable_window_sec_ = get_parameter("stable_window_sec").as_double();
    joint_position_window_threshold_rad_ =
      get_parameter("joint_position_window_threshold_rad").as_double();

    depth_min_m_ = get_parameter("depth_min_m").as_double();
    depth_max_m_ = get_parameter("depth_max_m").as_double();
    preview_size_px_ = static_cast<int>(get_parameter("preview_size_px").as_int());

    if (world_frame_.empty()) {
      throw std::runtime_error("world_frame must not be empty");
    }
    if (target_position_tolerance_m_ <= 0.0 || target_orientation_tolerance_deg_ <= 0.0) {
      throw std::runtime_error("Target pose tolerances must be positive");
    }
  }

  std::vector<ViewPlan> makePlans() const
  {
    const Eigen::Vector3d target = workpiece_center_m_ + scan_target_offset_m_;
    const Eigen::Matrix4d t_ee_camera_ros = t_ee_camera_gl_m_ * t_camera_gl_camera_ros_;
    const Eigen::Matrix4d t_tcp_camera_ros = t_ee_tcp_m_.inverse() * t_ee_camera_ros;

    RCLCPP_INFO(
      get_logger(), "T_tcp_camera_ros_m:\n%s", matrixToString(t_tcp_camera_ros).c_str());

    std::vector<ViewPlan> plans;
    plans.reserve(view_names_.size());

    for (std::size_t i = 0; i < view_names_.size(); ++i) {
      const double azimuth = view_azimuth_deg_[i] * M_PI / 180.0;
      const double elevation = view_elevation_deg_[i] * M_PI / 180.0;

      const Eigen::Vector3d direction(
        std::cos(elevation) * std::cos(azimuth),
        std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation));

      ViewPlan plan;
      plan.index = static_cast<int>(i);
      plan.name = view_names_[i];
      plan.azimuth_deg = view_azimuth_deg_[i];
      plan.elevation_deg = view_elevation_deg_[i];
      plan.camera_position_world_m = target + scan_radius_m_ * direction;
      plan.t_world_camera_ros_m = makeLookAtCameraPoseRos(
        plan.camera_position_world_m, target,
        std::abs(plan.elevation_deg - 90.0) < 1.0e-8);
      plan.t_world_tcp_m = plan.t_world_camera_ros_m * t_tcp_camera_ros.inverse();

      const Eigen::Vector3d rotation_vector = rotationMatrixToRotationVector(
        plan.t_world_tcp_m.block<3, 3>(0, 0), spatial_angle_degrees_);
      const Eigen::Vector3d translation_mm =
        1000.0 * plan.t_world_tcp_m.block<3, 1>(0, 3);

      plan.service_pose_mm_spatial = {
        translation_mm.x(), translation_mm.y(), translation_mm.z(),
        rotation_vector.x(), rotation_vector.y(), rotation_vector.z()};
      plans.push_back(plan);
    }
    return plans;
  }

  void printPlans(const std::vector<ViewPlan> & plans) const
  {
    const Eigen::Vector3d target = workpiece_center_m_ + scan_target_offset_m_;
    RCLCPP_INFO(
      get_logger(),
      "Workpiece center [m]=[%.6f %.6f %.6f], scan target [m]=[%.6f %.6f %.6f]",
      workpiece_center_m_.x(), workpiece_center_m_.y(), workpiece_center_m_.z(),
      target.x(), target.y(), target.z());
    RCLCPP_INFO(
      get_logger(),
      "Actual camera pose source: TF %s -> %s, correction frame matrix enabled",
      world_frame_.c_str(), tf_camera_frame_.empty() ? "<PointCloud frame>" : tf_camera_frame_.c_str());

    for (const auto & plan : plans) {
      const auto & p = plan.service_pose_mm_spatial;
      RCLCPP_INFO(
        get_logger(),
        "[%d] %-24s camera_m=[%.4f %.4f %.4f] tcp_mm_spatial=[%.3f %.3f %.3f %.3f %.3f %.3f]",
        plan.index, plan.name.c_str(),
        plan.camera_position_world_m.x(), plan.camera_position_world_m.y(),
        plan.camera_position_world_m.z(), p[0], p[1], p[2], p[3], p[4], p[5]);
    }
  }

  void writePlanFile(const std::vector<ViewPlan> & plans) const
  {
    const fs::path path = fs::path(output_dir_) / "generated_scan_plan.txt";
    std::ofstream output(path);
    if (!output) {
      throw std::runtime_error("Unable to write plan file: " + path.string());
    }

    output << std::fixed << std::setprecision(9);
    output << "workpiece_center_m " << workpiece_center_m_.transpose() << '\n';
    output << "workpiece_size_m " << workpiece_size_m_.transpose() << '\n';
    output << "scan_target_m " << (workpiece_center_m_ + scan_target_offset_m_).transpose() << '\n';
    output << "scan_radius_m " << scan_radius_m_ << '\n';
    output << "world_frame " << world_frame_ << '\n';
    output << "tf_camera_frame " << (tf_camera_frame_.empty() ? "<PointCloud frame>" : tf_camera_frame_) << '\n';
    output << "T_tf_camera_pointcloud_frame\n" << t_tf_camera_pointcloud_m_ << '\n';
    for (const auto & plan : plans) {
      output << "\nview " << plan.index << ' ' << plan.name << '\n';
      output << "azimuth_deg " << plan.azimuth_deg << '\n';
      output << "elevation_deg " << plan.elevation_deg << '\n';
      output << "camera_position_world_m " << plan.camera_position_world_m.transpose() << '\n';
      output << "service_pose_mm_spatial";
      for (const double value : plan.service_pose_mm_spatial) {
        output << ' ' << value;
      }
      output << '\n';
      output << "T_world_camera_ros_planned_m\n" << plan.t_world_camera_ros_m << '\n';
      output << "T_world_tcp_planned_m\n" << plan.t_world_tcp_m << '\n';
    }
    RCLCPP_INFO(get_logger(), "Plan saved: %s", path.c_str());
  }

  void waitForService()
  {
    const auto timeout = std::chrono::duration<double>(service_wait_timeout_sec_);
    if (!command_client_->wait_for_service(timeout)) {
      throw std::runtime_error("Service unavailable: " + service_name_);
    }
  }

  std::string resolvePointcloudFrame(const std::string & message_frame) const
  {
    const std::string frame = normalizeFrameId(message_frame);
    if (frame.empty()) {
      throw std::runtime_error("PointCloud2.header.frame_id is empty");
    }
    return frame;
  }

  std::string resolveTfCameraFrame(const std::string & pointcloud_frame) const
  {
    return tf_camera_frame_.empty() ? pointcloud_frame : tf_camera_frame_;
  }

  void waitForSensorsAndTf()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(sensor_startup_timeout_sec_);
    std::string last_tf_error;

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      sensor_msgs::msg::PointCloud2::SharedPtr cloud;
      bool joint_ready = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        cloud = latest_cloud_;
        joint_ready = static_cast<bool>(latest_joint_state_);
      }

      if (cloud && joint_ready) {
        try {
          const std::string pointcloud_frame = resolvePointcloudFrame(cloud->header.frame_id);
          const std::string tf_frame = resolveTfCameraFrame(pointcloud_frame);
          const rclcpp::Time latest_time(0, 0, get_clock()->get_clock_type());
          if (tf_buffer_->canTransform(
              world_frame_, tf_frame, latest_time,
              rclcpp::Duration::from_seconds(0.10), &last_tf_error))
          {
            RCLCPP_INFO(
              get_logger(),
              "Sensor and TF ready: cloud=%lu joint=%lu TF=%s <- %s PointCloud=%s",
              static_cast<unsigned long>(currentCloudSequence()),
              static_cast<unsigned long>(currentJointSequence()),
              world_frame_.c_str(), tf_frame.c_str(), pointcloud_frame.c_str());
            return;
          }
        } catch (const std::exception & error) {
          last_tf_error = error.what();
        }
      }
      std::this_thread::sleep_for(20ms);
    }

    std::ostringstream message;
    message << "Sensor/TF startup timeout. Need PointCloud2, JointState, and TF "
            << world_frame_ << " <- "
            << (tf_camera_frame_.empty() ? "PointCloud2.header.frame_id" : tf_camera_frame_);
    if (!last_tf_error.empty()) {
      message << ". Last TF error: " << last_tf_error;
    }
    throw std::runtime_error(message.str());
  }

  void callCommand(const std::string & mode, const std::vector<double> & pose)
  {
    auto request = std::make_shared<y2_rob_motion_interfaces::srv::SingleArmCommand::Request>();
    request->command_mode = mode;
    request->target_pose = pose;
    request->load_file = "";

    std::ostringstream pose_text;
    pose_text << '[';
    for (std::size_t i = 0; i < pose.size(); ++i) {
      if (i > 0) {
        pose_text << ' ';
      }
      pose_text << std::fixed << std::setprecision(3) << pose[i];
    }
    pose_text << ']';
    RCLCPP_INFO(get_logger(), "%s request: %s", mode.c_str(), pose_text.str().c_str());

    auto future = command_client_->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(
      shared_from_this(), future,
      std::chrono::duration<double>(service_call_timeout_sec_));

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error(mode + " service call timed out or failed");
    }
    const auto response = future.get();
    RCLCPP_INFO(
      get_logger(), "%s response: success=%s message=%s",
      mode.c_str(), response->success ? "true" : "false", response->message.c_str());
    if (!response->success) {
      throw std::runtime_error(mode + " service returned failure: " + response->message);
    }
  }

  void callIdlingNoThrow()
  {
    try {
      callCommand("Idling", {});
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Idling command failed: %s", error.what());
    }
  }

  Eigen::Matrix4d transformMessageToMatrix(
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    const Eigen::Isometry3d eigen_transform = tf2::transformToEigen(transform);
    return eigen_transform.matrix();
  }

  ActualPoseResult lookupActualCameraPose(
    const std::string & pointcloud_message_frame,
    const rclcpp::Time & requested_time,
    const bool allow_fallback)
  {
    ActualPoseResult result;
    result.world_frame = world_frame_;
    result.pointcloud_frame = resolvePointcloudFrame(pointcloud_message_frame);
    result.tf_camera_frame = resolveTfCameraFrame(result.pointcloud_frame);

    try {
      result.transform_message = tf_buffer_->lookupTransform(
        result.world_frame, result.tf_camera_frame, requested_time,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException & exact_error) {
      if (!allow_fallback || !allow_latest_tf_fallback_) {
        throw std::runtime_error(
                "Exact TF lookup failed at requested timestamp: " +
                std::string(exact_error.what()));
      }
      const rclcpp::Time latest_time(0, 0, get_clock()->get_clock_type());
      try {
        result.transform_message = tf_buffer_->lookupTransform(
          result.world_frame, result.tf_camera_frame, latest_time,
          rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
        result.used_latest_fallback = true;
        RCLCPP_WARN(
          get_logger(),
          "Exact TF lookup failed; using latest TF because robot is settled. Exact error: %s",
          exact_error.what());
      } catch (const tf2::TransformException & latest_error) {
        throw std::runtime_error(
                "TF lookup failed. Exact: " + std::string(exact_error.what()) +
                " | Latest: " + std::string(latest_error.what()));
      }
    }

    const Eigen::Matrix4d t_world_tf_camera =
      transformMessageToMatrix(result.transform_message);
    result.t_world_pointcloud_m = t_world_tf_camera * t_tf_camera_pointcloud_m_;
    return result;
  }

  ActualPoseResult lookupLatestActualCameraPose()
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      cloud = latest_cloud_;
    }
    if (!cloud) {
      throw std::runtime_error("Cannot resolve actual pose before PointCloud2 is received");
    }
    const rclcpp::Time latest_time(0, 0, get_clock()->get_clock_type());
    return lookupActualCameraPose(cloud->header.frame_id, latest_time, false);
  }

  bool waitUntilSettledAndReached(const ViewPlan & plan)
  {
    using Clock = std::chrono::steady_clock;
    struct PositionSample
    {
      Clock::time_point time;
      std::vector<double> position;
    };

    std::deque<PositionSample> samples;
    const auto start = Clock::now();
    auto last_status_log = start - 10s;
    std::uint64_t last_sequence = 0;
    std::optional<PoseError> last_pose_error;

    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();

      sensor_msgs::msg::JointState::SharedPtr message;
      std::uint64_t sequence = 0;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        message = latest_joint_state_;
        sequence = joint_sequence_;
      }

      if (message && sequence != last_sequence && !message->position.empty()) {
        last_sequence = sequence;
        samples.push_back({now, message->position});
      }

      const auto window_start = now - std::chrono::duration<double>(stable_window_sec_);
      while (!samples.empty() && samples.front().time < window_start) {
        samples.pop_front();
      }

      if (elapsed >= minimum_settle_sec_ && samples.size() >= 2) {
        const double covered_window =
          std::chrono::duration<double>(samples.back().time - samples.front().time).count();
        if (covered_window >= stable_window_sec_ * 0.9) {
          const std::size_t joint_count = samples.front().position.size();
          bool consistent_size = true;
          double maximum_range = 0.0;
          for (std::size_t joint = 0; joint < joint_count; ++joint) {
            double minimum = std::numeric_limits<double>::infinity();
            double maximum = -std::numeric_limits<double>::infinity();
            for (const auto & sample : samples) {
              if (sample.position.size() != joint_count) {
                consistent_size = false;
                break;
              }
              minimum = std::min(minimum, sample.position[joint]);
              maximum = std::max(maximum, sample.position[joint]);
            }
            if (!consistent_size) {
              break;
            }
            maximum_range = std::max(maximum_range, maximum - minimum);
          }

          if (consistent_size && maximum_range <= joint_position_window_threshold_rad_) {
            try {
              const ActualPoseResult actual = lookupLatestActualCameraPose();
              const PoseError pose_error = calculatePoseError(
                plan.t_world_camera_ros_m, actual.t_world_pointcloud_m);
              last_pose_error = pose_error;

              const bool target_reached =
                pose_error.position_m <= target_position_tolerance_m_ &&
                pose_error.orientation_deg <= target_orientation_tolerance_deg_;

              if (!require_target_reached_ || target_reached) {
                RCLCPP_INFO(
                  get_logger(),
                  "Robot settled and target reached: joint_range=%.8f rad, "
                  "camera_position_error=%.6f m, camera_orientation_error=%.4f deg",
                  maximum_range, pose_error.position_m, pose_error.orientation_deg);
                return true;
              }

              if (now - last_status_log >= 1s) {
                last_status_log = now;
                RCLCPP_WARN(
                  get_logger(),
                  "Robot is stationary but target pose is outside tolerance: "
                  "position=%.6f m (limit %.6f), orientation=%.4f deg (limit %.4f)",
                  pose_error.position_m, target_position_tolerance_m_,
                  pose_error.orientation_deg, target_orientation_tolerance_deg_);
              }
            } catch (const std::exception & error) {
              if (now - last_status_log >= 1s) {
                last_status_log = now;
                RCLCPP_WARN(
                  get_logger(), "Robot is stationary but actual camera TF is unavailable: %s",
                  error.what());
              }
            }
          }
        }
      }

      if (elapsed >= maximum_settle_sec_) {
        if (last_pose_error.has_value()) {
          RCLCPP_ERROR(
            get_logger(),
            "Capture forbidden after %.1f s: final camera error position=%.6f m, "
            "orientation=%.4f deg",
            maximum_settle_sec_, last_pose_error->position_m,
            last_pose_error->orientation_deg);
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "Capture forbidden: robot did not settle/reach target within %.1f s",
            maximum_settle_sec_);
        }
        return false;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  void waitWithSpin(const double seconds)
  {
    if (seconds <= 0.0) {
      return;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(10ms);
    }
  }

  sensor_msgs::msg::PointCloud2::SharedPtr captureCloudAfter(
    const std::uint64_t sequence_before_capture)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(pointcloud_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (latest_cloud_ && cloud_sequence_ > sequence_before_capture) {
          RCLCPP_INFO(
            get_logger(), "Using fresh post-settle cloud: sequence=%lu frame=%s stamp=%d.%09u",
            static_cast<unsigned long>(cloud_sequence_), latest_cloud_->header.frame_id.c_str(),
            latest_cloud_->header.stamp.sec, latest_cloud_->header.stamp.nanosec);
          return latest_cloud_;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    throw std::runtime_error("No fresh PointCloud2 received after settle completion");
  }

  sensor_msgs::msg::JointState::SharedPtr latestJointStateSnapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_joint_state_;
  }

  void saveDepthImages(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const fs::path & view_dir) const
  {
    if (cloud.height <= 1 || cloud.width == 0 || cloud.points.size() != cloud.width * cloud.height) {
      RCLCPP_WARN(
        get_logger(),
        "Point cloud is not organized; depth PNG cannot be reconstructed from pixel order.");
      return;
    }

    cv::Mat depth_mm(
      static_cast<int>(cloud.height), static_cast<int>(cloud.width), CV_16UC1, cv::Scalar(0));
    cv::Mat valid_mask(depth_mm.size(), CV_8UC1, cv::Scalar(0));

    for (std::uint32_t row = 0; row < cloud.height; ++row) {
      for (std::uint32_t col = 0; col < cloud.width; ++col) {
        const auto & point = cloud.at(col, row);
        if (!std::isfinite(point.z) || point.z < depth_min_m_ || point.z > depth_max_m_) {
          continue;
        }
        const double millimetres = std::round(static_cast<double>(point.z) * 1000.0);
        depth_mm.at<std::uint16_t>(static_cast<int>(row), static_cast<int>(col)) =
          static_cast<std::uint16_t>(std::clamp(millimetres, 0.0, 65535.0));
        valid_mask.at<std::uint8_t>(static_cast<int>(row), static_cast<int>(col)) = 255;
      }
    }

    const fs::path metric_path = view_dir / "depth_mm.png";
    if (!cv::imwrite(metric_path.string(), depth_mm)) {
      throw std::runtime_error("Failed to save depth image: " + metric_path.string());
    }

    double min_value = 0.0;
    double max_value = 0.0;
    cv::minMaxLoc(depth_mm, &min_value, &max_value, nullptr, nullptr, valid_mask);
    cv::Mat preview(depth_mm.size(), CV_8UC1, cv::Scalar(0));
    if (max_value > min_value) {
      depth_mm.convertTo(
        preview, CV_8UC1,
        -255.0 / (max_value - min_value),
        255.0 * max_value / (max_value - min_value));
      preview.setTo(0, valid_mask == 0);
    }
    cv::Mat colour_preview;
    cv::applyColorMap(preview, colour_preview, cv::COLORMAP_TURBO);
    colour_preview.setTo(cv::Scalar(0, 0, 0), valid_mask == 0);
    const fs::path preview_path = view_dir / "depth_preview.png";
    if (!cv::imwrite(preview_path.string(), colour_preview)) {
      throw std::runtime_error("Failed to save depth preview: " + preview_path.string());
    }
  }

  void saveTopDownPreview(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const Eigen::Vector3d & bbox_min,
    const Eigen::Vector3d & bbox_max,
    const fs::path & output_path) const
  {
    const int size = std::max(256, preview_size_px_);
    cv::Mat height_image(size, size, CV_32FC1, cv::Scalar(-1.0f));

    const double width_x = std::max(1.0e-9, bbox_max.x() - bbox_min.x());
    const double width_y = std::max(1.0e-9, bbox_max.y() - bbox_min.y());
    const double width_z = std::max(1.0e-9, bbox_max.z() - bbox_min.z());

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const int col = static_cast<int>(
        std::round((point.x - bbox_min.x()) / width_x * static_cast<double>(size - 1)));
      const int row = static_cast<int>(
        std::round((bbox_max.y() - point.y) / width_y * static_cast<double>(size - 1)));
      if (row < 0 || row >= size || col < 0 || col >= size) {
        continue;
      }
      const float normalized_height = static_cast<float>(
        std::clamp((point.z - bbox_min.z()) / width_z, 0.0, 1.0));
      height_image.at<float>(row, col) =
        std::max(height_image.at<float>(row, col), normalized_height);
    }

    cv::Mat valid_mask = height_image >= 0.0f;
    cv::Mat grayscale(size, size, CV_8UC1, cv::Scalar(0));
    height_image.convertTo(grayscale, CV_8UC1, 255.0);
    grayscale.setTo(0, valid_mask == 0);

    cv::Mat colour;
    cv::applyColorMap(grayscale, colour, cv::COLORMAP_TURBO);
    colour.setTo(cv::Scalar(0, 0, 0), valid_mask == 0);
    if (!cv::imwrite(output_path.string(), colour)) {
      throw std::runtime_error("Failed to save point-cloud preview PNG");
    }
  }

  void saveView(const ViewPlan & plan, const sensor_msgs::msg::PointCloud2 & message)
  {
    std::ostringstream directory_name;
    directory_name << std::setw(2) << std::setfill('0') << plan.index << '_' << plan.name;
    const fs::path view_dir = fs::path(output_dir_) / "views" / directory_name.str();
    fs::create_directories(view_dir);

    auto cloud_camera = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(message, *cloud_camera);

    const fs::path camera_pcd = view_dir / "cloud_camera.pcd";
    if (pcl::io::savePCDFileBinary(camera_pcd.string(), *cloud_camera) != 0) {
      throw std::runtime_error("Failed to save camera-frame PCD");
    }
    saveDepthImages(*cloud_camera, view_dir);

    const rclcpp::Time cloud_time(message.header.stamp, get_clock()->get_clock_type());
    const ActualPoseResult actual_pose = lookupActualCameraPose(
      message.header.frame_id, cloud_time, true);
    const PoseError capture_error = calculatePoseError(
      plan.t_world_camera_ros_m, actual_pose.t_world_pointcloud_m);

    if (require_target_reached_ &&
      (capture_error.position_m > target_position_tolerance_m_ ||
      capture_error.orientation_deg > target_orientation_tolerance_deg_))
    {
      std::ostringstream error;
      error << std::fixed << std::setprecision(6)
            << "Fresh cloud arrived, but actual camera pose left target tolerance. position="
            << capture_error.position_m << " m, orientation="
            << capture_error.orientation_deg << " deg. Capture rejected.";
      throw std::runtime_error(error.str());
    }

    auto cloud_world = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(
      *cloud_camera, *cloud_world, actual_pose.t_world_pointcloud_m.cast<float>());
    const fs::path world_pcd = view_dir / "cloud_world.pcd";
    if (pcl::io::savePCDFileBinary(world_pcd.string(), *cloud_world) != 0) {
      throw std::runtime_error("Failed to save actual-pose world-frame PCD");
    }

    if (save_planned_world_debug_) {
      auto cloud_world_planned = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::transformPointCloud(
        *cloud_camera, *cloud_world_planned, plan.t_world_camera_ros_m.cast<float>());
      const fs::path planned_debug_pcd = view_dir / "cloud_world_planned_debug.pcd";
      if (pcl::io::savePCDFileBinary(
          planned_debug_pcd.string(), *cloud_world_planned) != 0)
      {
        throw std::runtime_error("Failed to save planned-pose debug PCD");
      }
    }

    const Eigen::Vector3d half_size = 0.5 * workpiece_size_m_;
    Eigen::Vector3d bbox_min = workpiece_center_m_ - half_size;
    Eigen::Vector3d bbox_max = workpiece_center_m_ + half_size;
    bbox_min.x() -= bbox_margin_m_;
    bbox_min.y() -= bbox_margin_m_;
    bbox_max.x() += bbox_margin_m_;
    bbox_max.y() += bbox_margin_m_;
    bbox_min.z() += roi_bottom_offset_m_;
    bbox_max.z() += roi_top_margin_m_;

    auto cloud_roi = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_roi->reserve(cloud_world->size());
    for (const auto & point : cloud_world->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.x < bbox_min.x() || point.x > bbox_max.x() ||
        point.y < bbox_min.y() || point.y > bbox_max.y() ||
        point.z < bbox_min.z() || point.z > bbox_max.z())
      {
        continue;
      }
      cloud_roi->push_back(point);
    }
    cloud_roi->width = static_cast<std::uint32_t>(cloud_roi->size());
    cloud_roi->height = 1;
    cloud_roi->is_dense = false;

    const fs::path roi_pcd = view_dir / "cloud_world_roi.pcd";
    if (pcl::io::savePCDFileBinary(roi_pcd.string(), *cloud_roi) != 0) {
      throw std::runtime_error("Failed to save ROI PCD");
    }

    saveTopDownPreview(
      *cloud_roi, bbox_min, bbox_max, view_dir / "pcd_topdown_preview.png");

    const auto joint_state = latestJointStateSnapshot();
    const Eigen::Vector3d actual_camera_position =
      actual_pose.t_world_pointcloud_m.block<3, 1>(0, 3);

    const fs::path metadata_path = view_dir / "metadata.txt";
    std::ofstream metadata(metadata_path);
    if (!metadata) {
      throw std::runtime_error("Failed to write metadata.txt");
    }
    metadata << std::fixed << std::setprecision(9);
    metadata << "view_index " << plan.index << '\n';
    metadata << "view_name " << plan.name << '\n';
    metadata << "pointcloud_frame_id " << message.header.frame_id << '\n';
    metadata << "pointcloud_stamp_sec " << message.header.stamp.sec << '\n';
    metadata << "pointcloud_stamp_nanosec " << message.header.stamp.nanosec << '\n';
    metadata << "world_frame " << actual_pose.world_frame << '\n';
    metadata << "tf_camera_frame " << actual_pose.tf_camera_frame << '\n';
    metadata << "tf_used_latest_fallback "
             << (actual_pose.used_latest_fallback ? "true" : "false") << '\n';
    metadata << "tf_stamp_sec " << actual_pose.transform_message.header.stamp.sec << '\n';
    metadata << "tf_stamp_nanosec " << actual_pose.transform_message.header.stamp.nanosec << '\n';
    metadata << "organized_width " << cloud_camera->width << '\n';
    metadata << "organized_height " << cloud_camera->height << '\n';
    metadata << "raw_point_slots " << cloud_camera->size() << '\n';
    metadata << "roi_point_count " << cloud_roi->size() << '\n';
    metadata << "camera_position_world_m " << actual_camera_position.transpose() << '\n';
    metadata << "camera_position_planned_world_m "
             << plan.camera_position_world_m.transpose() << '\n';
    metadata << "camera_position_actual_world_m "
             << actual_camera_position.transpose() << '\n';
    metadata << "camera_position_error_m " << capture_error.position_m << '\n';
    metadata << "camera_orientation_error_deg " << capture_error.orientation_deg << '\n';
    metadata << "roi_bbox_min_world_m " << bbox_min.transpose() << '\n';
    metadata << "roi_bbox_max_world_m " << bbox_max.transpose() << '\n';
    metadata << "service_pose_mm_spatial";
    for (const double value : plan.service_pose_mm_spatial) {
      metadata << ' ' << value;
    }
    metadata << '\n';
    metadata << "T_world_camera_ros_planned_m\n" << plan.t_world_camera_ros_m << '\n';
    metadata << "T_world_camera_ros_actual_m\n" << actual_pose.t_world_pointcloud_m << '\n';
    metadata << "T_tf_camera_pointcloud_frame\n" << t_tf_camera_pointcloud_m_ << '\n';

    if (joint_state) {
      metadata << "joint_state_stamp_sec " << joint_state->header.stamp.sec << '\n';
      metadata << "joint_state_stamp_nanosec " << joint_state->header.stamp.nanosec << '\n';
      metadata << "joint_names";
      for (const auto & name : joint_state->name) {
        metadata << ' ' << name;
      }
      metadata << '\n';
      metadata << "joint_positions_rad";
      for (const double position : joint_state->position) {
        metadata << ' ' << position;
      }
      metadata << '\n';
    }

    RCLCPP_INFO(
      get_logger(),
      "CAPTURED %s: raw=%zu roi=%zu actual_pose_error=[%.6f m, %.4f deg] output=%s",
      plan.name.c_str(), cloud_camera->size(), cloud_roi->size(),
      capture_error.position_m, capture_error.orientation_deg, view_dir.c_str());
  }

  std::uint64_t currentCloudSequence() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return cloud_sequence_;
  }

  std::uint64_t currentJointSequence() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return joint_sequence_;
  }

  std::string service_name_;
  std::string pointcloud_topic_;
  std::string joint_state_topic_;
  std::string output_dir_;
  bool spatial_angle_degrees_{true};

  std::string world_frame_{"world"};
  std::string tf_camera_frame_;
  double tf_lookup_timeout_sec_{1.0};
  bool allow_latest_tf_fallback_{true};
  bool require_target_reached_{true};
  double target_position_tolerance_m_{0.005};
  double target_orientation_tolerance_deg_{1.0};
  double capture_delay_after_settle_sec_{0.20};
  bool save_planned_world_debug_{true};
  Eigen::Matrix4d t_tf_camera_pointcloud_m_{Eigen::Matrix4d::Identity()};

  Eigen::Vector3d workpiece_center_m_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d workpiece_size_m_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d scan_target_offset_m_{Eigen::Vector3d::Zero()};
  double scan_radius_m_{0.6};
  double bbox_margin_m_{0.02};
  double roi_bottom_offset_m_{0.003};
  double roi_top_margin_m_{0.020};

  Eigen::Matrix4d t_ee_tcp_m_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d t_ee_camera_gl_m_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d t_camera_gl_camera_ros_{Eigen::Matrix4d::Identity()};

  std::vector<std::string> view_names_;
  std::vector<double> view_azimuth_deg_;
  std::vector<double> view_elevation_deg_;

  bool plan_only_{false};
  bool run_all_{false};
  int view_index_{0};
  bool continue_on_failure_{false};

  double service_wait_timeout_sec_{20.0};
  double service_call_timeout_sec_{45.0};
  double sensor_startup_timeout_sec_{30.0};
  double pointcloud_timeout_sec_{10.0};
  double minimum_settle_sec_{2.0};
  double maximum_settle_sec_{15.0};
  double stable_window_sec_{1.0};
  double joint_position_window_threshold_rad_{0.0015};
  double depth_min_m_{0.05};
  double depth_max_m_{3.0};
  int preview_size_px_{1024};

  mutable std::mutex data_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::uint64_t cloud_sequence_{0};
  std::uint64_t joint_sequence_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Client<y2_rob_motion_interfaces::srv::SingleArmCommand>::SharedPtr command_client_;
};

}  // namespace nrs_scan_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<nrs_scan_cpp::ScanCaptureNode>();
    const int result = node->run();
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    std::cerr << "[FATAL] " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
