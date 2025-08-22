#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
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

class InteractiveIKNode : public rclcpp::Node {
public:
  InteractiveIKNode()
  : Node("interactive_ik_node")
  {
    // Parameters
    hz_           = this->declare_parameter<double>("hz", 100.0);
    duration_s_   = this->declare_parameter<double>("duration_s", 2.0);
    use_degrees_  = this->declare_parameter<bool>("use_degrees", false);
    interactive_  = this->declare_parameter<bool>("interactive", true);

    // target pose params (⚠️ yaw는 'yaw'로!)
    x_   = this->declare_parameter<double>("x", 0.0);
    y_   = this->declare_parameter<double>("y", 0.0);     // position Y
    z_   = this->declare_parameter<double>("z", 0.0);
    r_   = this->declare_parameter<double>("r", 0.0);
    p_   = this->declare_parameter<double>("p", 0.0);
    yaw_ = this->declare_parameter<double>("yaw", 0.0);   // ✅ was "y" → "yaw"

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/isaac_joint_commands", rclcpp::QoS(10).reliable());

    joint_state_.name = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    joint_state_.position.assign(6, 0.0);

    init_arm_state_();

    if (interactive_) {
      std::cout << "[interactive_ik_node] Enter x y z r p yaw : ";
      std::cout.flush();
      if (!(std::cin >> x_ >> y_ >> z_ >> r_ >> p_ >> yaw_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read x y z r p yaw from terminal.");
        throw std::runtime_error("stdin read failed");
      }
    }

    if (use_degrees_) {
      constexpr double D2R = M_PI/180.0;
      r_   *= D2R;
      p_   *= D2R;
      yaw_ *= D2R;
    }

    RCLCPP_INFO(this->get_logger(),
      "Target (x y z r p yaw) = [%.6f %.6f %.6f %.6f %.6f %.6f] (rad)",
      x_, y_, z_, r_, p_, yaw_);

    if (!compute_once_and_prepare()) {
      RCLCPP_ERROR(this->get_logger(), "IK failed. Not publishing.");
      throw std::runtime_error("IK failed");
    }

    start_time_ = std::chrono::high_resolution_clock::now();
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&InteractiveIKNode::publish_cmd, this)
    );
  }

private:
  // Params
  double hz_, duration_s_;
  bool use_degrees_, interactive_;

  // target pose
  double x_, y_, z_, r_, p_, yaw_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_state_;

  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;

  // Kinematics
  Kinematic_func kin_;
  CArm arm_;

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

  bool compute_once_and_prepare() {
    Vector3d th; th << r_, p_, yaw_;
    Matrix3d R;
    kin_.EulerAngle2Rotation(R, th);

    arm_.Td.setIdentity();
    arm_.Td.block<3,3>(0,0) = R;
    arm_.Td(0,3) = x_;
    arm_.Td(1,3) = y_;
    arm_.Td(2,3) = z_;

    arm_.qc = arm_.qd;
    int nsol = kin_.InverseK_min(&arm_);
    if (nsol <= 0) {
      RCLCPP_WARN(this->get_logger(), "InverseK_min returned no solution.");
      return false;
    }

    joint_state_.position[0] = arm_.qd(0);
    joint_state_.position[1] = arm_.qd(1);
    joint_state_.position[2] = arm_.qd(2);
    joint_state_.position[3] = arm_.qd(3);
    joint_state_.position[4] = arm_.qd(4);
    joint_state_.position[5] = arm_.qd(5);

    RCLCPP_INFO(this->get_logger(),
      "IK q = [%.6f %.6f %.6f %.6f %.6f %.6f]",
      arm_.qd(0), arm_.qd(1), arm_.qd(2), arm_.qd(3), arm_.qd(4), arm_.qd(5));
    return true;
  }

  void publish_cmd() {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();

    joint_state_.header.stamp = this->now();
    pub_->publish(joint_state_);

    if (elapsed_s >= duration_s_) {
      RCLCPP_INFO(this->get_logger(), "Done publishing for %.2f s.", duration_s_);
      timer_->cancel();
    }
  }
};

int main(int argc, char* argv[]) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "mlockall failed");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Memory locked.");
  }

  struct sched_param sp{ .sched_priority = 80 };
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to set real-time scheduling");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InteractiveIKNode>());
  rclcpp::shutdown();
  return 0;
}
