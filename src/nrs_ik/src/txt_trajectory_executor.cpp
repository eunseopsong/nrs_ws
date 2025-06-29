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
using Eigen::Vector3d;
using std::sin, std::cos;

class TxtTrajectoryExecutor : public rclcpp::Node
{
public:
  TxtTrajectoryExecutor() : Node("txt_trajectory_executor")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/isaac_joint_commands", rclcpp::QoS(10).reliable());

    joint_state_.name = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    joint_state_.position.resize(6, 0.0);

    load_waypoints("/home/eunseop/nrs_ws/src/nrs_path2/data/geodesic_waypoints.txt");

    start_time_ = last_time_ = std::chrono::high_resolution_clock::now();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(300),
      std::bind(&TxtTrajectoryExecutor::publish_next_point, this));
  }

private:
  const double d1 = 0.1273;
  const double a2 = -0.612;
  const double a3 = -0.5723;
  const double d4 = 0.1639;
  const double d5 = 0.1157;
  const double d6 = 0.0922 + 0.186;  // spindle 포함

  const double roll = 0.0;
  const double pitch = M_PI;
  const double yaw = 0.0;

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

    for (size_t i = 0; i + 1 < raw_points.size(); ++i) {
      Vector3d p1(raw_points[i][0], raw_points[i][1], raw_points[i][2]);
      Vector3d p2(raw_points[i+1][0], raw_points[i+1][1], raw_points[i+1][2]);
      // for (int j = 0; j < 5; ++j) {
      //   double t = static_cast<double>(j) / 5.0;
      //   Vector3d p = (1 - t) * p1 + t * p2;
      //   waypoints_.push_back({p[0], p[1], p[2]});
      // }
      for (int j = 0; j < 10; ++j) {
      double t = static_cast<double>(j) / 10.0;
      Vector3d p = (1 - t) * p1 + t * p2;
      waypoints_.push_back({p[0], p[1], p[2]});
      }
    }

    if (!raw_points.empty())
      waypoints_.push_back(raw_points.back());

    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", waypoints_.size());
  }

  Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw)
  {
    Matrix3d Rx, Ry, Rz;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;
    return Rz * Ry * Rx;
  }

  void publish_next_point()
  {
    if (current_idx_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints done.");
      timer_->cancel();
      return;
    }

    auto t_now = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_now - start_time_).count();
    double delta_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_now - last_time_).count();
    last_time_ = t_now;

    auto& wp = waypoints_[current_idx_++];
    double x = wp[0], y = wp[1], z = wp[2];
    Vector3d p(x, y, z);

    Matrix3d R_ee = rpyToRotationMatrix(roll, pitch, yaw);

    Vector3d pwc = p - d6 * R_ee.col(2);
    double xc = pwc(0), yc = pwc(1), zc = pwc(2);

    double q1 = atan2(yc, xc);
    double r = sqrt(xc * xc + yc * yc), s = zc - d1;
    double D = (r * r + s * s - a2 * a2 - a3 * a3) / (2 * a2 * a3);

    if (D < -1 || D > 1) {
      RCLCPP_WARN(this->get_logger(), "Unreachable point at idx %zu", current_idx_ - 1);
      return;
    }

    double q3 = atan2(-sqrt(1 - D * D), D);
    double q2 = atan2(s, r) - atan2(a3 * sin(q3), a2 + a3 * cos(q3));

    Matrix3d R01, R12, R23;
    R01 << cos(q1), 0, sin(q1),
           sin(q1), 0, -cos(q1),
           0, 1, 0;

    R12 << cos(q2), -sin(q2), 0,
           0, 0, -1,
           sin(q2), cos(q2), 0;

    R23 << cos(q3), -sin(q3), 0,
           sin(q3), cos(q3), 0,
           0, 0, 1;

    Matrix3d R03 = R01 * R12 * R23;
    Matrix3d R36 = R03.transpose() * R_ee;

    double q4 = atan2(R36(2,1), R36(2,2));
    double q5 = atan2(sqrt(R36(2,1)*R36(2,1) + R36(2,2)*R36(2,2)), R36(2,0));
    double q6 = atan2(R36(1,0), R36(0,0));

    joint_state_.position = {q1, q2, q3, q4, q5, q6};
    joint_state_.header.stamp = this->now();
    pub_->publish(joint_state_);

    RCLCPP_INFO(this->get_logger(), "step %zu: elapsed %.3f ms, delta %.3f ms", current_idx_, elapsed_ms, delta_ms);
  }
};

int main(int argc, char* argv[])
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    RCLCPP_WARN(rclcpp::get_logger("main"), "mlockall failed");
  else
    RCLCPP_INFO(rclcpp::get_logger("main"), "Memory locked.");

  struct sched_param schedp{.sched_priority = 80};
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedp) != 0)
    RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to set real-time scheduling");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TxtTrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
