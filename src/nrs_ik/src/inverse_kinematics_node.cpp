#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <array>
#include <cmath>

using std::placeholders::_1;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class InverseKinematicsNode : public rclcpp::Node
{
public:
  InverseKinematicsNode()
  : Node("inverse_kinematics_node")
  {
    sub_ = create_subscription<Float64MultiArray>(
      "planned_trajectory", 10,
      std::bind(&InverseKinematicsNode::ik_callback, this, _1));

    pub_ = create_publisher<Float64MultiArray>(
      "joint_commands", 10);

    RCLCPP_INFO(get_logger(), "InverseKinematicsNode started and listening to planned_trajectory.");
  }

private:
  void ik_callback(const Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) return;
    double x = msg->data[0];
    double y = msg->data[1];
    double z = msg->data[2];
    double roll = msg->data[3];
    double pitch = msg->data[4];
    double yaw = msg->data[5];

    // Geometric IK for UR-10 (simplified 6 DOF planar assumption)
    // This is **not** a full implementation. Replace with full UR-10 geometry if needed.

    std::vector<double> joint_angles(6, 0.0);

    double L1 = 0.1273;  // link lengths based on URDF
    double L2 = 0.612;
    double L3 = 0.5723;

    double wx = x;
    double wy = y;
    double wz = z;

    double q1 = atan2(wy, wx);
    double r = sqrt(wx*wx + wy*wy);
    double s = wz - L1;

    double D = (r*r + s*s - L2*L2 - L3*L3) / (2 * L2 * L3);
    if (D < -1.0 || D > 1.0) {
      RCLCPP_WARN(get_logger(), "Position unreachable. D = %.2f", D);
      return;
    }

    double q3 = atan2(-sqrt(1 - D*D), D);
    double q2 = atan2(s, r) - atan2(L3 * sin(q3), L2 + L3 * cos(q3));

    // Basic approximation: keep wrist orientation aligned with RPY = 0
    double q4 = 0.0;
    double q5 = 0.0;
    double q6 = 0.0;

    joint_angles[0] = q1;
    joint_angles[1] = q2;
    joint_angles[2] = q3;
    joint_angles[3] = q4;
    joint_angles[4] = q5;
    joint_angles[5] = q6;

    Float64MultiArray out;
    out.data = joint_angles;
    pub_->publish(out);

    RCLCPP_INFO(get_logger(),
      "Published joint angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      joint_angles[0], joint_angles[1], joint_angles[2],
      joint_angles[3], joint_angles[4], joint_angles[5]);
  }

  rclcpp::Subscription<Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
