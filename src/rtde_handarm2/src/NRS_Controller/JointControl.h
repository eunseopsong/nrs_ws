// #ifndef JOINTCONTROL_H
// #define JOINTCONTROL_H

// #include "func.h"

// // using namespace std;
// // using namespace Eigen;
// // using namespace std::chrono_literals;

// class JointControl : public rclcpp::Node
// {
// public:
//     JointControl();

// private:
//     void CalculateAndPublishTorque();

//     // // Subscriber declarations
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodypose;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodypos;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodyvel;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_imu;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_contact;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_command;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_distance;
//     // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_link_force;

//     // // Publisher declarations
//     // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torque;
//     // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_targetpos;

//     // // Timer
//     // rclcpp::TimerBase::SharedPtr timer_;
//     // double _count;
//     // array<double, 3> previous_command; // save the previous command value
// };

// #endif // JOINTCONTROL_H