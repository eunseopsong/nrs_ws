#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>
#include <string>
#include <sstream>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>
#include <cmath>

#include "Arm_class.h"
#include "Kinematics.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

class IKControlNode : public rclcpp::Node {
public:
  IKControlNode() : Node("ik_control")
  {
    // ---- Parameters ----
    topic_        = this->declare_parameter<std::string>("topic", "/isaac_joint_commands");
    hz_           = this->declare_parameter<double>("hz", 100.0);
    use_degrees_  = this->declare_parameter<bool>("use_degrees", false); // r,p,y 단위 선택
    tool_z_       = this->declare_parameter<double>("tool_z", 0.248);    // EE +Z → TCP [m]
    interp_s_     = this->declare_parameter<double>("interp_s", 0.0);    // 보간 시간(초)

    // ---- Publisher ----
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(topic_, rclcpp::QoS(10).reliable());
    joint_state_.name = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    joint_state_.position.assign(6, 0.0);

    // ---- IK structs ----
    init_arm_state_();

    // 홈 자세 (rad) : (0, -90, -90, -90, 90, 0) deg
    home_q_ << 0.0, -M_PI/2, -M_PI/2, -M_PI/2, +M_PI/2, 0.0;

    // ✅ 초기 상태를 "홈 자세 유지"로 설정
    {
      std::lock_guard<std::mutex> lk(mtx_);
      current_q_ = home_q_;
      target_q_  = home_q_;
      interp_i_  = 0;
      interp_N_  = 0;
    }

    // ---- Timer for publishing / interpolation ----
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&IKControlNode::on_timer_, this));

    // ---- CLI guide ----
    RCLCPP_INFO(this->get_logger(),
      "IK control ready. Type:  x y z r p y  (rpy in %s).",
      use_degrees_ ? "deg" : "rad");
    RCLCPP_INFO(this->get_logger(), "Special commands:  home  |  quit");
    RCLCPP_INFO(this->get_logger(), "Params: topic=%s, hz=%.1f, interp_s=%.2f, tool_z=%.3f",
                topic_.c_str(), hz_, interp_s_, tool_z_);
    RCLCPP_INFO(this->get_logger(),
      "Initial state set to HOME: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] (rad)",
      home_q_(0), home_q_(1), home_q_(2), home_q_(3), home_q_(4), home_q_(5));

    // ---- stdin thread ----
    run_input_.store(true);
    input_thread_ = std::thread(&IKControlNode::stdin_loop_, this);
  }

  ~IKControlNode() override {
    run_input_.store(false);
    if (input_thread_.joinable()) input_thread_.join();
  }

private:
  // ===== ROS =====
  std::string topic_;
  double hz_;
  bool use_degrees_;
  double tool_z_;
  double interp_s_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_state_;

  // ===== IK / Robot =====
  Kinematic_func kin_;
  CArm arm_;
  Eigen::Matrix<double,6,1> home_q_;

  // ===== State / Interpolation =====
  std::mutex mtx_;
  Eigen::Matrix<double,6,1> current_q_;
  Eigen::Matrix<double,6,1> target_q_;
  int interp_i_{0}, interp_N_{0};

  // ===== Input thread =====
  std::thread input_thread_;
  std::atomic<bool> run_input_{false};

  // ---- Helpers ----
  void init_arm_state_() {
    arm_.qc = Eigen::Vector<double, 6>::Zero();
    arm_.qd = Eigen::Vector<double, 6>::Zero();
    arm_.q  = Eigen::Vector<double, 6*8>::Zero();
    arm_.Td = Matrix4d::Identity();
    arm_.Tc = Matrix4d::Identity();
    arm_.R2E_init_flag = false;
    arm_.pre_thc = Eigen::Vector3d::Zero();
    arm_.thc     = Eigen::Vector3d::Zero();
  }

  static void trim_(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
      [](unsigned char ch){ return !std::isspace(ch); }));
    s.erase(std::find_if(s.rbegin(), s.rend(),
      [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
  }

  static bool iequals_(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    for (size_t i=0;i<a.size();++i)
      if (std::tolower((unsigned char)a[i]) != std::tolower((unsigned char)b[i])) return false;
    return true;
  }

  bool pose_to_ik_(double x, double y, double z, double r, double p, double yaw,
                   Eigen::Matrix<double,6,1>& q_out)
  {
    if (use_degrees_) {
      constexpr double D2R = M_PI/180.0;
      r   *= D2R; p *= D2R; yaw *= D2R;
    }

    // RPY -> R
    Matrix3d R;
    {
      Vector3d th; th << r, p, yaw;
      kin_.EulerAngle2Rotation(R, th);
    }

    // TCP 목표
    Matrix4d Td_tcp = Matrix4d::Identity();
    Td_tcp.block<3,3>(0,0) = R;
    Td_tcp(0,3) = x; Td_tcp(1,3) = y; Td_tcp(2,3) = z;

    // EE→TCP 오프셋 (EE +Z로 tool_z_)
    Matrix4d EE2TCP = Matrix4d::Identity();
    EE2TCP(2,3) = tool_z_;

    // IK는 EE 기준
    arm_.Td = Td_tcp * EE2TCP.inverse();

    // 현재 q 근처 해 선택
    {
      std::lock_guard<std::mutex> lk(mtx_);
      arm_.qc = current_q_;
    }
    int nsol = kin_.InverseK_min(&arm_);
    if (nsol <= 0) return false;

    for (int i = 0; i < 6; ++i) q_out(i) = arm_.qd(i);
    return true;
  }

  void start_interp_(const Eigen::Matrix<double,6,1>& q_goal) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (interp_s_ <= 0.0) {
      current_q_ = q_goal;
      target_q_  = q_goal;
      interp_i_  = 0; interp_N_ = 0;
    } else {
      target_q_ = q_goal;
      interp_i_ = 0;
      interp_N_ = std::max(1, (int)std::round(interp_s_ * hz_));
    }
  }

  void on_timer_() {
    Eigen::Matrix<double,6,1> q_pub;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (interp_N_ > 0 && interp_i_ < interp_N_) {
        double t = double(interp_i_ + 1) / double(interp_N_);
        t = std::clamp(t, 0.0, 1.0);
        q_pub = (1.0 - t) * current_q_ + t * target_q_;
        current_q_ = q_pub;
        ++interp_i_;
      } else {
        q_pub = current_q_;
      }
    }

    joint_state_.header.stamp = this->now();
    for (int i = 0; i < 6; ++i) joint_state_.position[i] = q_pub(i);
    pub_->publish(joint_state_);
  }

  void stdin_loop_() {
    std::string line;
    while (run_input_.load()) {
      if (!std::getline(std::cin, line)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      trim_(line);
      if (line.empty()) continue;

      if (iequals_(line, "quit") || iequals_(line, "exit")) {
        RCLCPP_INFO(this->get_logger(), "Quit requested.");
        run_input_.store(false);
        rclcpp::shutdown();
        break;
      }
      if (iequals_(line, "home")) {
        RCLCPP_INFO(this->get_logger(), "[HOME] interp_s=%.2f", interp_s_);
        start_interp_(home_q_);
        continue;
      }

      // 쉼표 허용
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream iss(line);
      double x,y,z,r,p,yaw;
      if (!(iss >> x >> y >> z >> r >> p >> yaw)) {
        RCLCPP_WARN(this->get_logger(), "Parse failed. Need: x y z r p y  | got: \"%s\"", line.c_str());
        continue;
      }

      Eigen::Matrix<double,6,1> q_sol;
      if (!pose_to_ik_(x,y,z,r,p,yaw, q_sol)) {
        RCLCPP_WARN(this->get_logger(),
                    "IK failed for pose: x=%.6f y=%.6f z=%.6f r=%.6f p=%.6f y=%.6f",
                    x,y,z,r,p,yaw);
        continue;
      }

      RCLCPP_INFO(this->get_logger(),
                  "IK OK → q = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f] (interp_s=%.2f)",
                  q_sol(0), q_sol(1), q_sol(2), q_sol(3), q_sol(4), q_sol(5), interp_s_);
      start_interp_(q_sol);
    }
  }
};

// ===== main =====
int main(int argc, char* argv[]) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "mlockall failed");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Memory locked.");
  }

  struct sched_param sp;
  sp.sched_priority = 80;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to set real-time scheduling");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKControlNode>());
  rclcpp::shutdown();
  return 0;
}
