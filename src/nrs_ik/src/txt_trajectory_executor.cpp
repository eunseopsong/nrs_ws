#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <sys/mman.h>
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

// try fix 2025.06.29

class TxtTrajectoryExecutor : public rclcpp::Node
{
public:
  TxtTrajectoryExecutor() : Node("txt_trajectory_executor")
  {
    // QoS 수정: reliable()로 설정 (Isaac Sim 호환)
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/isaac_joint_commands", rclcpp::QoS(10).reliable());

    joint_state_.name = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };
    joint_state_.position.resize(6, 0.0);

    load_waypoints("/home/eunseop/nrs_ws/src/nrs_path2/data/geodesic_waypoints.txt");

    start_time_ = last_time_ = std::chrono::high_resolution_clock::now();

    // 500Hz → 2ms 주기
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&TxtTrajectoryExecutor::publish_next_point, this));
  }

private:
  const double d1 = 0.1273;
  const double a2 = -0.612;
  const double a3 = -0.5723;
  const double d4 = 0.1639;
  const double d5 = 0.1157;
  const double d6 = 0.0922;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_state_;
  std::vector<std::vector<double>> waypoints_;
  size_t current_idx_ = 0;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_, last_time_;

  void load_waypoints(const std::string& filepath)
  {
    std::ifstream infile(filepath);
    std::string line;
    std::vector<std::vector<double>> raw_points;

    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      std::vector<double> values((std::istream_iterator<double>(iss)), std::istream_iterator<double>());
      if (values.size() >= 3)
        raw_points.push_back({values[0], values[1], values[2]});
    }

    // 5배 선형 보간
    for (size_t i = 0; i + 1 < raw_points.size(); ++i) {
      Vector3d p1(raw_points[i][0], raw_points[i][1], raw_points[i][2]);
      Vector3d p2(raw_points[i+1][0], raw_points[i+1][1], raw_points[i+1][2]);
      for (int j = 0; j < 5; ++j) {
        double t = static_cast<double>(j) / 5.0;
        Vector3d p = (1 - t) * p1 + t * p2;
        waypoints_.push_back({p[0], p[1], p[2]});
      }
    }

    if (!raw_points.empty()) {
      waypoints_.push_back(raw_points.back());
    }

    RCLCPP_INFO(this->get_logger(),
      "Loaded %zu original waypoints, interpolated to %zu points.",
      raw_points.size(), waypoints_.size());
  }

  void publish_next_point()
  {
    if (current_idx_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints executed.");
      timer_->cancel();
      return;
    }

    auto t_now = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_now - start_time_).count();
    double delta_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_now - last_time_).count();
    last_time_ = t_now;

    auto& wp = waypoints_[current_idx_++];
    double x = wp[0], y = wp[1], z = wp[2] + 0.2;
    Vector3d p(x, y, z);
    Matrix3d R = Matrix3d::Identity();

    Vector3d pwc = p - d6 * R.col(2);
    double xc = pwc(0), yc = pwc(1), zc = pwc(2);
    double q1 = atan2(yc, xc);
    double r = sqrt(xc*xc + yc*yc), s = zc - d1;
    double D = (r*r + s*s - a2*a2 - a3*a3) / (2*a2*a3);

    if (D < -1 || D > 1) {
      RCLCPP_WARN(this->get_logger(), "Unreachable point at idx %zu", current_idx_ - 1);
      return;
    }

    double q3 = atan2(-sqrt(1 - D*D), D);
    double q2 = atan2(s, r) - atan2(a3*sin(q3), a2 + a3*cos(q3));
    double q4 = -M_PI / 2, q5 = M_PI / 2, q6 = 0.0;

    joint_state_.position = {q1, q2, q3, q4, q5, q6};
    joint_state_.header.stamp = this->now();
    pub_->publish(joint_state_);

    RCLCPP_INFO(this->get_logger(),
      "step %zu: elapsed %.3f ms, delta %.3f ms",
      current_idx_, elapsed_ms, delta_ms);
  }

  Matrix4d dh_transform(double a, double d, double alpha, double theta)
  {
    Matrix4d T;
    T << cos(theta), -sin(theta), 0, a,
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d,
         0, 0, 0, 1;
    return T;
  }
};

int main(int argc, char* argv[])
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"),
      "mlockall failed: insufficient privileges or limits. Page faults may occur.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Memory locked: no page faults.");
  }

  struct sched_param schedp{.sched_priority = 80};
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedp) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to set real-time scheduling: %m");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TxtTrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
