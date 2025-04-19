#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using std::placeholders::_1;
using std::vector;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::AngleAxisd;

class InverseKinematicsNode : public rclcpp::Node
{
public:
  InverseKinematicsNode() : Node("inverse_kinematics_node")
  {
    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "planned_trajectory", 10, std::bind(&InverseKinematicsNode::ik_callback, this, _1));

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/isaac_joint_commands", 10);

    joint_state_.name = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };
    joint_state_.position.resize(6, 0.0);
  }

private:
  // DH 파라미터 (UR10 기준)
  const double d1 = 0.1273;
  const double a2 = -0.612;
  const double a3 = -0.5723;
  const double d4 = 0.1639;
  const double d5 = 0.1157;
  const double d6 = 0.0922;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  sensor_msgs::msg::JointState joint_state_;

  void ik_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      RCLCPP_WARN(this->get_logger(), "Invalid input. Expected 6 values (x,y,z,r,p,y)");
      return;
    }

    double x = msg->data[0];
    double y = msg->data[1];
    double z = msg->data[2];
    double roll  = msg->data[3];
    double pitch = msg->data[4];
    double yaw   = msg->data[5];

    // 1. 목표 pose를 Transformation Matrix로 변환
    Matrix3d R = (AngleAxisd(yaw, Vector3d::UnitZ()) *
                  AngleAxisd(pitch, Vector3d::UnitY()) *
                  AngleAxisd(roll, Vector3d::UnitX())).toRotationMatrix();

    Vector3d p(x, y, z);
    Matrix4d T06 = Matrix4d::Identity();
    T06.block<3,3>(0,0) = R;
    T06.block<3,1>(0,3) = p;

    // 2. Wrist Center 계산
    Vector3d pwc = p - d6 * R.col(2); // z-axis 방향

    double xc = pwc(0), yc = pwc(1), zc = pwc(2);

    // 3. q1
    double q1 = atan2(yc, xc);

    // 4. q2, q3 (planar 2R arm IK)
    double r = sqrt(xc*xc + yc*yc) - 0; // a1 = 0
    double s = zc - d1;

    double D = (r*r + s*s - a2*a2 - a3*a3) / (2*a2*a3);
    if (D < -1 || D > 1) {
      RCLCPP_WARN(this->get_logger(), "Unreachable position");
      return;
    }

    double q3 = atan2(-sqrt(1 - D*D), D); // Elbow down
    double q2 = atan2(s, r) - atan2(a3*sin(q3), a2 + a3*cos(q3));

    // 5. q4~q6 계산
    Matrix4d T01 = dh_transform(0, d1, 0, q1);
    Matrix4d T12 = dh_transform(a2, 0, 0, q2);
    Matrix4d T23 = dh_transform(a3, 0, 0, q3);
    Matrix4d T03 = T01 * T12 * T23;
    Matrix3d R03 = T03.block<3,3>(0,0);
    Matrix3d R36 = R03.transpose() * R;

    double q5 = atan2(sqrt(R36(0,2)*R36(0,2) + R36(1,2)*R36(1,2)), R36(2,2));
    double q4 = atan2(R36(1,2), R36(0,2));
    double q6 = atan2(R36(2,1), -R36(2,0));

    // 6. 발행
    joint_state_.position = {q1, q2, q3, q4, q5, q6};
    joint_state_.header.stamp = this->get_clock()->now();
    pub_->publish(joint_state_);

    RCLCPP_INFO(this->get_logger(),
      "Published joint angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      q1, q2, q3, q4, q5, q6);
  }

  // DH 변환 행렬 생성 함수
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
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
