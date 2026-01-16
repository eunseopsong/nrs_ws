#include "vr_calibration.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <algorithm>
#include <chrono>

using std::placeholders::_1;

VrCalibration::VrCalibration()
: Node("vr_calibration_auto_capture"),
  steady_clock_(RCL_STEADY_TIME)
{
  // ----------------------------
  // Parameters (keep defaults)
  // ----------------------------
  this->declare_parameter<std::string>("waypoint_file",
    "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/vr_calibration/for_vr_calibration_point.txt");

  this->declare_parameter<std::string>("ee_out",
    "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/vr_calibration/ur10_ee.txt");
  this->declare_parameter<std::string>("vr_out",
    "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/vr_calibration/ur10_vr.txt");

  this->declare_parameter<std::string>("topic_currentP", "/ur10skku/currentP");
  this->declare_parameter<std::string>("topic_raw_pose", "/raw_pose");
  this->declare_parameter<std::string>("topic_calibrated_pose", "/calibrated_pose");

  const char* home = std::getenv("HOME");
  if (!home) throw std::runtime_error("HOME env not set");
  this->declare_parameter<std::string>("calib_yaml_path",
    std::string(home) + "/nrs_ws/src/vive_tracker_ros2/yaml/calibration_matrix.yaml");

  this->declare_parameter<double>("pos_enter_mm", pos_enter_mm_);
  this->declare_parameter<double>("pos_exit_mm", pos_exit_mm_);
  this->declare_parameter<double>("ori_enter_deg", ori_enter_deg_);
  this->declare_parameter<double>("ori_exit_deg", ori_exit_deg_);
  this->declare_parameter<double>("vel_thresh_mms", vel_thresh_mms_);
  this->declare_parameter<double>("angvel_thresh_dps", angvel_thresh_dps_);
  this->declare_parameter<double>("hold_time_s", hold_time_s_);

  this->declare_parameter<double>("cp_fresh_s", cp_fresh_s_);
  this->declare_parameter<double>("vr_capture_age_s", vr_capture_age_s_);
  this->declare_parameter<double>("target_timeout_s", target_timeout_s_);
  this->declare_parameter<double>("loop_hz", loop_hz_);
  this->declare_parameter<int>("cp_unit_probe_N", static_cast<int>(cp_unit_probe_N_));

  // T_SA
  this->declare_parameter<double>("t_sa_w_des_z", t_sa_w_des_z_);
  this->declare_parameter<std::string>("t_sa_side", t_sa_side_); // "left" or "right"
  this->declare_parameter<double>("t_sa_wait_timeout_s", t_sa_wait_timeout_s_);
  this->declare_parameter<double>("t_sa_hold_s", t_sa_hold_s_);
  this->declare_parameter<double>("t_sa_fresh_s", t_sa_fresh_s_);

  // ----------------------------
  // Get params
  // ----------------------------
  waypoint_file_ = this->get_parameter("waypoint_file").as_string();
  ee_path_       = this->get_parameter("ee_out").as_string();
  vr_path_       = this->get_parameter("vr_out").as_string();

  topic_currentP_       = this->get_parameter("topic_currentP").as_string();
  topic_raw_pose_       = this->get_parameter("topic_raw_pose").as_string();
  topic_calibrated_pose_= this->get_parameter("topic_calibrated_pose").as_string();

  calib_yaml_path_ = this->get_parameter("calib_yaml_path").as_string();

  pos_enter_mm_ = this->get_parameter("pos_enter_mm").as_double();
  pos_exit_mm_  = this->get_parameter("pos_exit_mm").as_double();
  ori_enter_deg_= this->get_parameter("ori_enter_deg").as_double();
  ori_exit_deg_ = this->get_parameter("ori_exit_deg").as_double();
  vel_thresh_mms_ = this->get_parameter("vel_thresh_mms").as_double();
  angvel_thresh_dps_ = this->get_parameter("angvel_thresh_dps").as_double();
  hold_time_s_ = this->get_parameter("hold_time_s").as_double();

  cp_fresh_s_ = this->get_parameter("cp_fresh_s").as_double();
  vr_capture_age_s_ = this->get_parameter("vr_capture_age_s").as_double();
  target_timeout_s_ = this->get_parameter("target_timeout_s").as_double();
  loop_hz_ = this->get_parameter("loop_hz").as_double();
  cp_unit_probe_N_ = static_cast<size_t>(this->get_parameter("cp_unit_probe_N").as_int());

  t_sa_w_des_z_ = this->get_parameter("t_sa_w_des_z").as_double();
  t_sa_side_    = this->get_parameter("t_sa_side").as_string();
  t_sa_wait_timeout_s_ = this->get_parameter("t_sa_wait_timeout_s").as_double();
  t_sa_hold_s_  = this->get_parameter("t_sa_hold_s").as_double();
  t_sa_fresh_s_ = this->get_parameter("t_sa_fresh_s").as_double();

  // normalize side
  std::transform(t_sa_side_.begin(), t_sa_side_.end(), t_sa_side_.begin(), ::tolower);
  if (t_sa_side_ != "left" && t_sa_side_ != "right") {
    throw std::runtime_error("param t_sa_side must be 'left' or 'right'");
  }

  // ----------------------------
  // Load waypoints
  // ----------------------------
  vr_calib::loadWaypointsAndDetectUnits(waypoint_file_, waypoints_, wp_rotvec_in_degrees_);
  target_indices_ = vr_calib::buildTargetIndices(waypoints_);

  // ----------------------------
  // open output files
  // ----------------------------
  ee_ofs_.open(ee_path_, std::ios::out | std::ios::trunc);
  vr_ofs_.open(vr_path_, std::ios::out | std::ios::trunc);
  if (!ee_ofs_.is_open() || !vr_ofs_.is_open())
    throw std::runtime_error("Failed to open output files (truncate mode)");

  // ----------------------------
  // solver init (ALL math in solver.cpp)
  // ----------------------------
  solver_.setYamlPath(calib_yaml_path_);
  solver_.setTSADesiredZ(t_sa_w_des_z_);
  solver_.setTSASide(t_sa_side_);            // ✅ 반드시 vive_tracker와 일치
  solver_.loadExistingYamlConstants();
  solver_.resetSamples();
  solver_.resetRAdj();

  // ----------------------------
  // Subscriptions
  // ----------------------------
  sub_currentP_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_currentP_, 10, std::bind(&VrCalibration::cbCurrentP, this, _1));

  sub_vr_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_raw_pose_, 10, std::bind(&VrCalibration::cbVR, this, _1));

  sub_cal_pose_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_calibrated_pose_, 10, std::bind(&VrCalibration::cbCalibratedPose, this, _1));

  RCLCPP_INFO(get_logger(),
    "Loaded %zu waypoints (%zu target points). wp_rotvec_unit=%s | T_SA_side=%s",
    waypoints_.size(), target_indices_.size(),
    wp_rotvec_in_degrees_ ? "DEG" : "RAD",
    t_sa_side_.c_str());
}

void VrCalibration::cbCurrentP(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 6) return;
  const auto ts = tnow();

  std::lock_guard<std::mutex> lk(mtx_);
  for (int i=0;i<6;i++) last_cp_[i] = msg->data[i];
  have_cp_ = true;
  last_cp_time_ = ts;
  cp_seq_++;

  // auto-detect rotvec unit for currentP
  if (!cp_rotvec_unit_decided_) {
    cp_probe_cnt_++;
    cp_probe_max_abs_ = std::max(cp_probe_max_abs_, std::fabs(last_cp_[3]));
    cp_probe_max_abs_ = std::max(cp_probe_max_abs_, std::fabs(last_cp_[4]));
    cp_probe_max_abs_ = std::max(cp_probe_max_abs_, std::fabs(last_cp_[5]));
    if (cp_probe_cnt_ >= cp_unit_probe_N_) {
      cp_rotvec_in_degrees_ = (cp_probe_max_abs_ > 6.0);
      cp_rotvec_unit_decided_ = true;
      RCLCPP_INFO(get_logger(),
        "currentP rotvec unit decided: %s (max_abs=%.3f over %zu samples)",
        cp_rotvec_in_degrees_ ? "DEG" : "RAD", cp_probe_max_abs_, cp_probe_cnt_);
    }
  }
}

void VrCalibration::cbVR(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto ts = tnow();
  std::lock_guard<std::mutex> lk(mtx_);
  last_vr_[0]=msg->pose.position.x;
  last_vr_[1]=msg->pose.position.y;
  last_vr_[2]=msg->pose.position.z;
  last_vr_[3]=msg->pose.orientation.x;
  last_vr_[4]=msg->pose.orientation.y;
  last_vr_[5]=msg->pose.orientation.z;
  last_vr_[6]=msg->pose.orientation.w;
  have_vr_ = true;
  last_vr_time_ = ts;

  if (!vr_pos_unit_decided_) {
    double mabs = 0.0;
    mabs = std::max(mabs, std::fabs(last_vr_[0]));
    mabs = std::max(mabs, std::fabs(last_vr_[1]));
    mabs = std::max(mabs, std::fabs(last_vr_[2]));
    vr_pos_in_mm_ = (mabs > 10.0); // heuristic
    vr_pos_unit_decided_ = true;
    RCLCPP_INFO(get_logger(),
      "raw_pose position unit decided (heuristic): %s (max_abs=%.3f)",
      vr_pos_in_mm_ ? "MM" : "M", mabs);
  }
}

void VrCalibration::cbCalibratedPose(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 6) return;
  const auto ts = tnow();
  std::lock_guard<std::mutex> lk(mtx_);
  for (int i=0;i<6;i++) last_cal_pose_[i] = msg->data[i];
  last_cal_pose_time_ = ts;
  have_cal_pose_ = true;
}

bool VrCalibration::getLatestData(std::array<double,6>& cp,
                                  std::array<double,7>& vr,
                                  rclcpp::Time& cp_t,
                                  rclcpp::Time& vr_t,
                                  uint64_t& cp_seq_out)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!have_cp_) return false;

  cp = last_cp_;
  cp_t = last_cp_time_;
  cp_seq_out = cp_seq_;

  if (have_vr_) {
    vr = last_vr_;
    vr_t = last_vr_time_;
  } else {
    vr_t = rclcpp::Time(0,0,RCL_STEADY_TIME);
  }
  return true;
}

bool VrCalibration::isCpFresh(const rclcpp::Time& cp_t) const
{
  return (tnow() - cp_t).seconds() <= cp_fresh_s_;
}

bool VrCalibration::isVrFreshForCapture(const rclcpp::Time& vr_t) const
{
  if (!have_vr_) return false;
  return (tnow() - vr_t).seconds() <= vr_capture_age_s_;
}

bool VrCalibration::isCalPoseFresh() const
{
  if (!have_cal_pose_) return false;
  return (tnow() - last_cal_pose_time_).seconds() <= t_sa_fresh_s_;
}

void VrCalibration::resetMotionDetector()
{
  have_prev_motion_ = false;
  prev_motion_seq_  = 0;
  last_vnorm_mms_   = 1e9;
  last_omega_dps_   = 1e9;
}

void VrCalibration::updateMotionIfNew(const std::array<double,6>& cp,
                                      const rclcpp::Time& cp_time,
                                      uint64_t cp_seq_in)
{
  if (cp_seq_in == prev_motion_seq_) return;

  if (!have_prev_motion_) {
    prev_motion_cp_   = cp;
    prev_motion_time_ = cp_time;
    have_prev_motion_ = true;
    prev_motion_seq_  = cp_seq_in;
    return;
  }

  const double dt = (cp_time - prev_motion_time_).seconds();
  if (dt <= 1e-4) {
    prev_motion_cp_   = cp;
    prev_motion_time_ = cp_time;
    prev_motion_seq_  = cp_seq_in;
    return;
  }

  // linear speed (mm/s)
  double v2 = 0.0;
  for (int i=0;i<3;i++){
    double v = (cp[i] - prev_motion_cp_[i]) / dt;
    v2 += v*v;
  }
  last_vnorm_mms_ = std::sqrt(v2);

  // angular speed (deg/s) using rotvec->R
  const bool cp_deg = cp_rotvec_unit_decided_ ? cp_rotvec_in_degrees_ : false;

  Eigen::Vector3d w_prev(prev_motion_cp_[3], prev_motion_cp_[4], prev_motion_cp_[5]);
  Eigen::Vector3d w_cur (cp[3],             cp[4],             cp[5]);
  if (cp_deg) { w_prev *= (M_PI/180.0); w_cur *= (M_PI/180.0); }

  Eigen::Matrix3d Rprev = vr_calib::rotvecToRotMat(w_prev);
  Eigen::Matrix3d Rcur  = vr_calib::rotvecToRotMat(w_cur);

  const double dang = vr_calib::rotAngleBetweenRad(Rprev, Rcur);
  last_omega_dps_ = vr_calib::rad2deg(dang) / dt;

  prev_motion_cp_   = cp;
  prev_motion_time_ = cp_time;
  prev_motion_seq_  = cp_seq_in;
}

bool VrCalibration::isStoppedNow() const
{
  return (last_vnorm_mms_ <= vel_thresh_mms_) && (last_omega_dps_ <= angvel_thresh_dps_);
}

bool VrCalibration::computeTSAOnceBeforeCapture(rclcpp::executors::SingleThreadedExecutor& exec,
                                                Eigen::Matrix4d& T_SA_new_out)
{
  RCLCPP_INFO(get_logger(),
    "[T_SA] waiting /calibrated_pose(fresh<=%.2fs) AND robot STOP hold(%.2fs), timeout=%.1fs | side=%s",
    t_sa_fresh_s_, t_sa_hold_s_, t_sa_wait_timeout_s_, t_sa_side_.c_str());

  const rclcpp::Time t0 = tnow();
  bool hold_active = false;
  rclcpp::Time hold_start = tnow();

  resetMotionDetector();
  rclcpp::Rate rate(std::min(200.0, loop_hz_));

  while (rclcpp::ok() && (tnow() - t0).seconds() < t_sa_wait_timeout_s_) {
    exec.spin_some();

    std::array<double,6> cp;
    std::array<double,7> vr;
    rclcpp::Time cp_t, vr_t;
    uint64_t cp_seq = 0;

    if (!getLatestData(cp, vr, cp_t, vr_t, cp_seq)) { rate.sleep(); continue; }
    if (!isCpFresh(cp_t)) { rate.sleep(); continue; }

    updateMotionIfNew(cp, cp_t, cp_seq);
    if (!isStoppedNow()) { hold_active = false; rate.sleep(); continue; }

    if (!isCalPoseFresh()) { hold_active = false; rate.sleep(); continue; }

    if (!hold_active) {
      hold_active = true;
      hold_start = tnow();
    }

    if ((tnow() - hold_start).seconds() >= t_sa_hold_s_) {
      // w_meas(rad) from calibrated_pose: [x y z wx wy wz]
      Eigen::Vector3d w_meas(last_cal_pose_[3], last_cal_pose_[4], last_cal_pose_[5]);
      Eigen::Matrix3d R_total = vr_calib::rotvecToRotMat(w_meas);

      T_SA_new_out = solver_.computeTSAFromCalibratedPoseRotation(R_total);

      RCLCPP_INFO(get_logger(),
        "[T_SA_DONE] w_meas(rad)=[%.6f %.6f %.6f] -> computed T_SA (side=%s)",
        w_meas[0], w_meas[1], w_meas[2], t_sa_side_.c_str());
      return true;
    }

    rate.sleep();
  }

  RCLCPP_WARN(get_logger(),
    "[T_SA_WARN] timeout. Will keep old T_SA in yaml (no update).");
  return false;
}

void VrCalibration::captureOnce(size_t target_k, size_t wp_idx,
                                const std::array<double,6>& cp,
                                const std::array<double,7>& vr,
                                double dist_mm, double ang_deg)
{
  const bool cp_deg = cp_rotvec_unit_decided_ ? cp_rotvec_in_degrees_ : false;

  // --- EE rotation from rotvec ---
  Eigen::Vector3d w(cp[3], cp[4], cp[5]);
  if (cp_deg) w *= (M_PI/180.0);
  Eigen::Matrix3d R_ab = vr_calib::rotvecToRotMat(w);

  // --- EE position (mm -> m) ---
  const double cp_x_m = cp[0] * 1e-3;
  const double cp_y_m = cp[1] * 1e-3;
  const double cp_z_m = cp[2] * 1e-3;

  // --- VR position (meters by default) ---
  double vr_x = vr[0], vr_y = vr[1], vr_z = vr[2];
  if (vr_pos_unit_decided_ && vr_pos_in_mm_) {
    vr_x *= 1e-3; vr_y *= 1e-3; vr_z *= 1e-3;
  }

  // --- write files ---
  ee_ofs_
    << R_ab(0,0)<<" "<<R_ab(0,1)<<" "<<R_ab(0,2)<<" "<<cp_x_m<<" "
    << R_ab(1,0)<<" "<<R_ab(1,1)<<" "<<R_ab(1,2)<<" "<<cp_y_m<<" "
    << R_ab(2,0)<<" "<<R_ab(2,1)<<" "<<R_ab(2,2)<<" "<<cp_z_m<<"\n";

  vr_ofs_
    << vr_x<<" "<<vr_y<<" "<<vr_z<<" "
    << vr[3]<<" "<<vr[4]<<" "<<vr[5]<<" "<<vr[6]<<"\n";

  ee_ofs_.flush();
  vr_ofs_.flush();

  // --- build sample transforms (meters) ---
  Eigen::Vector3d p_ab(cp_x_m, cp_y_m, cp_z_m);
  Eigen::Matrix4d T_AB = vr_calib::makeT(R_ab, p_ab);

  Eigen::Quaterniond q_vr(vr[6], vr[3], vr[4], vr[5]); // (w,x,y,z)
  q_vr.normalize();
  Eigen::Matrix3d R_dc = q_vr.toRotationMatrix();
  Eigen::Vector3d p_dc(vr_x, vr_y, vr_z);
  Eigen::Matrix4d T_DC = vr_calib::makeT(R_dc, p_dc);

  // solver stores everything
  solver_.pushSample(T_AB, T_DC);
  solver_.feedVRQuaternionForRAdj(q_vr);

  RCLCPP_INFO(get_logger(),
    "[CAPTURE] target %zu/%zu (wp line %zu) | dist=%.2fmm ang=%.2fdeg | v=%.2fmm/s w=%.2fdeg/s",
    target_k+1, target_indices_.size(), wp_idx+1,
    dist_mm, ang_deg, last_vnorm_mms_, last_omega_dps_);
}

void VrCalibration::run()
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(shared_from_this());

  if (target_indices_.empty()) {
    RCLCPP_WARN(get_logger(), "No target points (flag!=0). Nothing to capture.");
    return;
  }

  // ==========================================================
  // (0) Pre-phase: compute T_SA once before capture
  // ==========================================================
  Eigen::Matrix4d T_SA_new = Eigen::Matrix4d::Identity();
  const bool t_sa_computed = computeTSAOnceBeforeCapture(exec, T_SA_new);

  // ==========================================================
  // Capture loop
  // ==========================================================
  enum class State { WAIT_ENTER, IN_REGION };
  State state = State::WAIT_ENTER;

  size_t target_k = 0;
  rclcpp::Time target_start_time = tnow();

  resetMotionDetector();
  bool hold_active = false;
  rclcpp::Time hold_start_time = tnow();

  rclcpp::Rate rate(loop_hz_);

  while (rclcpp::ok() && target_k < target_indices_.size()) {
    exec.spin_some();

    std::array<double,6> cp;
    std::array<double,7> vr;
    rclcpp::Time cp_t, vr_t;
    uint64_t cp_seq = 0;

    if (!getLatestData(cp, vr, cp_t, vr_t, cp_seq)) { rate.sleep(); continue; }
    if (!isCpFresh(cp_t)) { rate.sleep(); continue; }

    const size_t wp_idx = target_indices_[target_k];
    const auto& target_pose = waypoints_[wp_idx].pose;

    if ((tnow() - target_start_time).seconds() > target_timeout_s_) {
      RCLCPP_WARN(get_logger(),
        "[TIMEOUT] target %zu/%zu (wp line %zu). Skipping.",
        target_k+1, target_indices_.size(), wp_idx+1);
      target_k++;
      state = State::WAIT_ENTER;
      target_start_time = tnow();
      resetMotionDetector();
      hold_active = false;
      rate.sleep();
      continue;
    }

    const double dist_mm = vr_calib::posDistMm(cp, target_pose);
    const bool cp_deg = cp_rotvec_unit_decided_ ? cp_rotvec_in_degrees_ : false;
    const double ang_deg = vr_calib::oriErrDeg(cp, cp_deg, target_pose, wp_rotvec_in_degrees_);

    if (state == State::WAIT_ENTER) {
      if (dist_mm <= pos_enter_mm_ && ang_deg <= ori_enter_deg_) {
        state = State::IN_REGION;
        hold_active = false;
        resetMotionDetector();
        target_start_time = tnow();

        RCLCPP_INFO(get_logger(),
          "[IN] target %zu/%zu (wp line %zu) | dist=%.2fmm ang=%.2fdeg",
          target_k+1, target_indices_.size(), wp_idx+1, dist_mm, ang_deg);
      }
      rate.sleep();
      continue;
    }

    // OUT region
    if (dist_mm >= pos_exit_mm_ || ang_deg >= ori_exit_deg_) {
      state = State::WAIT_ENTER;
      hold_active = false;
      resetMotionDetector();
      RCLCPP_WARN(get_logger(),
        "[OUT] left region -> WAIT_ENTER | dist=%.2fmm ang=%.2fdeg",
        dist_mm, ang_deg);
      rate.sleep();
      continue;
    }

    // hold condition
    updateMotionIfNew(cp, cp_t, cp_seq);

    const bool stopped_now = isStoppedNow();
    if (!(dist_mm <= pos_enter_mm_ && ang_deg <= ori_enter_deg_)) {
      hold_active = false;
    } else if (!stopped_now) {
      hold_active = false;
    } else {
      if (!hold_active) {
        hold_active = true;
        hold_start_time = tnow();
      }
    }

    if (hold_active) {
      const double held = (tnow() - hold_start_time).seconds();
      if (held >= hold_time_s_) {

        if (!isVrFreshForCapture(vr_t)) {
          RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000,
            "[WAIT_VR] VR too old (age=%.2fs). Waiting...",
            (tnow() - vr_t).seconds());
          rate.sleep();
          continue;
        }

        captureOnce(target_k, wp_idx, cp, vr, dist_mm, ang_deg);

        target_k++;
        state = State::WAIT_ENTER;
        target_start_time = tnow();
        resetMotionDetector();
        hold_active = false;
      }
    }

    rate.sleep();
  }

  RCLCPP_INFO(get_logger(), "All target waypoints processed. Finalizing...");

  // ==========================================================
  // Final: solve + write YAML ONCE
  // ==========================================================
  try {
    solver_.finalizeAndSave(t_sa_computed, T_SA_new);

    RCLCPP_INFO(get_logger(),
      "[YAML_SAVED] T_AD, T_BC, R_Adj(%s), T_SA(%s, side=%s) -> %s",
      solver_.haveRAdj() ? "computed" : "IDENTITY(fallback)",
      t_sa_computed ? "computed" : "kept old",
      t_sa_side_.c_str(),
      calib_yaml_path_.c_str());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "finalizeAndSave failed: %s", e.what());
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VrCalibration>();
  node->run();
  rclcpp::shutdown();
  return 0;
}