#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

class TxtTrajectoryExecutor : public rclcpp::Node
{
public:
  TxtTrajectoryExecutor() : Node("txt_trajectory_executor")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands", 10);

    joint_state_.name = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };
    joint_state_.position.resize(6, 0.0);

    timer_ = this->create_wall_timer(100ms, std::bind(&TxtTrajectoryExecutor::publish_next_point, this));

    load_waypoints("/home/eunseop/nrs_ws/src/nrs_path2/data/geodesic_waypoints.txt");
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

  void load_waypoints(const std::string& filepath)
  {
    std::ifstream infile(filepath);
    std::string line;
    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      std::vector<double> values((std::istream_iterator<double>(iss)), std::istream_iterator<double>());
      if (values.size() >= 3)
        waypoints_.push_back(values);  // 3개 이상이면 받아줌
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", waypoints_.size());
  }

  void publish_next_point()
  {
    if (current_idx_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints executed.");
      timer_->cancel();
      return;
    }

    auto& wp = waypoints_[current_idx_++];
    Vector3d p(wp[0], wp[1], wp[2]);
    Matrix3d R = Matrix3d::Identity();  // orientation은 고정 (rpy = 0)

    Vector3d pwc = p - d6 * R.col(2);  // Wrist center
    double xc = pwc(0), yc = pwc(1), zc = pwc(2);
    double q1 = atan2(yc, xc);

    double r = sqrt(xc*xc + yc*yc);
    double s = zc - d1;

    double D = (r*r + s*s - a2*a2 - a3*a3) / (2*a2*a3);
    if (D < -1 || D > 1) {
      RCLCPP_WARN(this->get_logger(), "Unreachable point at idx %zu", current_idx_ - 1);
      return;
    }

    double q3 = atan2(-sqrt(1 - D*D), D); // elbow-down
    double q2 = atan2(s, r) - atan2(a3*sin(q3), a2 + a3*cos(q3));

    Matrix4d T01 = dh_transform(0, d1, 0, q1);
    Matrix4d T12 = dh_transform(a2, 0, 0, q2);
    Matrix4d T23 = dh_transform(a3, 0, 0, q3);
    Matrix4d T03 = T01 * T12 * T23;
    Matrix3d R03 = T03.block<3,3>(0,0);
    Matrix3d R36 = R03.transpose() * R;

    double q5 = atan2(sqrt(R36(0,2)*R36(0,2) + R36(1,2)*R36(1,2)), R36(2,2));
    double q4 = atan2(R36(1,2), R36(0,2));
    double q6 = atan2(R36(2,1), -R36(2,0));

    joint_state_.position = {q1, q2, q3, q4, q5, q6};
    joint_state_.header.stamp = this->now();
    pub_->publish(joint_state_);

    RCLCPP_INFO(this->get_logger(), "[%lu] Published joint: [%.3f %.3f %.3f %.3f %.3f %.3f]",
      current_idx_, q1, q2, q3, q4, q5, q6);
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

// === main 함수 ===
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TxtTrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
