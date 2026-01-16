#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <mutex>
#include <cstdint>

#include "vr_calibration_common.hpp"
#include "vr_calibration_solver.hpp"

class VrCalibration : public rclcpp::Node
{
public:
  VrCalibration();
  void run();

private:
  // ----------------------------
  // Time helper (steady clock)
  // ----------------------------
  rclcpp::Time tnow() const { return steady_clock_.now(); } // NOTE: now() is const-safe in Humble
  mutable rclcpp::Clock steady_clock_;

  // ----------------------------
  // Callbacks
  // ----------------------------
  void cbCurrentP(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void cbVR(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cbCalibratedPose(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // ----------------------------
  // Data access / freshness
  // ----------------------------
  bool getLatestData(std::array<double,6>& cp,
                     std::array<double,7>& vr,
                     rclcpp::Time& cp_t,
                     rclcpp::Time& vr_t,
                     uint64_t& cp_seq_out);

  bool isCpFresh(const rclcpp::Time& cp_t) const;
  bool isVrFreshForCapture(const rclcpp::Time& vr_t) const;
  bool isCalPoseFresh() const;

  // ----------------------------
  // Motion detection (stop-hold)
  // ----------------------------
  void resetMotionDetector();
  void updateMotionIfNew(const std::array<double,6>& cp,
                         const rclcpp::Time& cp_time,
                         uint64_t cp_seq_in);
  bool isStoppedNow() const;

  // ----------------------------
  // T_SA one-shot (before capture)
  // ----------------------------
  bool computeTSAOnceBeforeCapture(rclcpp::executors::SingleThreadedExecutor& exec,
                                   Eigen::Matrix4d& T_SA_new_out);

  // ----------------------------
  // Capture + logging
  // ----------------------------
  void captureOnce(size_t target_k, size_t wp_idx,
                   const std::array<double,6>& cp,
                   const std::array<double,7>& vr,
                   double dist_mm, double ang_deg);

private:
  // ----------------------------
  // Params / paths
  // ----------------------------
  std::string waypoint_file_;
  std::string ee_path_;
  std::string vr_path_;

  std::string topic_currentP_;
  std::string topic_raw_pose_;
  std::string topic_calibrated_pose_;

  std::string calib_yaml_path_;

  double pos_enter_mm_ = 2.0;
  double pos_exit_mm_  = 5.0;
  double ori_enter_deg_ = 2.0;
  double ori_exit_deg_  = 5.0;

  double vel_thresh_mms_ = 2.0;
  double angvel_thresh_dps_ = 2.0;
  double hold_time_s_ = 0.15;

  double cp_fresh_s_ = 0.2;
  double vr_capture_age_s_ = 0.2;
  double target_timeout_s_ = 20.0;
  double loop_hz_ = 100.0;

  size_t cp_unit_probe_N_ = 30;

  // T_SA params
  double t_sa_w_des_z_ = 1.57079632679;
  std::string t_sa_side_ = "right";
  double t_sa_wait_timeout_s_ = 8.0;
  double t_sa_hold_s_ = 0.3;
  double t_sa_fresh_s_ = 0.2;

  // ----------------------------
  // Waypoints
  // ----------------------------
  std::vector<vr_calib::Waypoint> waypoints_;
  std::vector<size_t> target_indices_;
  bool wp_rotvec_in_degrees_ = false;

  // ----------------------------
  // Output files
  // ----------------------------
  std::ofstream ee_ofs_;
  std::ofstream vr_ofs_;

  // ----------------------------
  // Solver (ALL math in solver.cpp)
  // ----------------------------
  VrCalibrationSolver solver_;

  // ----------------------------
  // ROS subscriptions
  // ----------------------------
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_currentP_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vr_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cal_pose_;

  // ----------------------------
  // Shared data (callbacks -> loop)
  // ----------------------------
  mutable std::mutex mtx_;

  std::array<double,6> last_cp_{};
  std::array<double,7> last_vr_{};
  std::array<double,6> last_cal_pose_{};

  bool have_cp_ = false;
  bool have_vr_ = false;
  bool have_cal_pose_ = false;

  rclcpp::Time last_cp_time_{0,0,RCL_STEADY_TIME};
  rclcpp::Time last_vr_time_{0,0,RCL_STEADY_TIME};
  rclcpp::Time last_cal_pose_time_{0,0,RCL_STEADY_TIME};

  uint64_t cp_seq_ = 0;

  // ----------------------------
  // Unit detection
  // ----------------------------
  bool cp_rotvec_unit_decided_ = false;
  bool cp_rotvec_in_degrees_ = false;
  size_t cp_probe_cnt_ = 0;
  double cp_probe_max_abs_ = 0.0;

  bool vr_pos_unit_decided_ = false;
  bool vr_pos_in_mm_ = false;

  // ----------------------------
  // Motion detector state
  // ----------------------------
  bool have_prev_motion_ = false;
  uint64_t prev_motion_seq_ = 0;
  std::array<double,6> prev_motion_cp_{};
  rclcpp::Time prev_motion_time_{0,0,RCL_STEADY_TIME};

  double last_vnorm_mms_ = 1e9;
  double last_omega_dps_ = 1e9;
};
