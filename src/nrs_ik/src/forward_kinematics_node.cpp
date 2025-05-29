#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <array>
#include <cmath>

// push test from home (2025.05.29 16:16)

using std::placeholders::_1;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class ForwardKinematicsNode : public rclcpp::Node
{
public:
  ForwardKinematicsNode()
  : Node("forward_kinematics_node")
  {
    sub_ = create_subscription<Float64MultiArray>(
      "joint_commands", 10,
      std::bind(&ForwardKinematicsNode::fk_callback, this, _1));

    RCLCPP_INFO(get_logger(), "ForwardKinematicsNode is running.");
  }

private:
  void fk_callback(const Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      RCLCPP_WARN(get_logger(), "Expected 6 joint angles, but got %zu", msg->data.size());
      return;
    }

    // Read all 6 joint angles
    const double q1 = msg->data[0];
    const double q2 = msg->data[1];
    const double q3 = msg->data[2];
    const double q4 = msg->data[3];
    const double q5 = msg->data[4];
    const double q6 = msg->data[5];

    // UR-10 DH parameters (simplified)
    double d1 = 0.1273;
    double a2 = -0.612;
    double a3 = -0.5723;
    double d4 = 0.163941;
    double d5 = 0.1157;
    double d6 = 0.0922;

    // Transformation matrices using DH convention
    auto cosq = [](double q) { return cos(q); };
    auto sinq = [](double q) { return sin(q); };

    // Forward Kinematics to calculate end-effector position (simplified position only)
    // This can be replaced with a full transformation matrix if orientation is needed

    // Just placeholder for now: use a simplified version to estimate position
    double x = cosq(q1) * (a2 * cosq(q2) + a3 * cosq(q2 + q3));
    double y = sinq(q1) * (a2 * cosq(q2) + a3 * cosq(q2 + q3));
    double z = d1 + a2 * sinq(q2) + a3 * sinq(q2 + q3);

    RCLCPP_INFO(get_logger(), "[FK Result] x=%.3f y=%.3f z=%.3f (from q1~q6)", x, y, z);
  }

  rclcpp::Subscription<Float64MultiArray>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForwardKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
