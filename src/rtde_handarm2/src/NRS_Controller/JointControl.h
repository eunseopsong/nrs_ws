#ifndef JOINTCONTROL_H
#define JOINTCONTROL_H

#include "func.h"
// #include "ROS_callbacks.cpp"

class JointControl : public rclcpp::Node
{
public:
    JointControl();
    ~JointControl();

private:
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr UR10e_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr PB_iter_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr VR_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    // rclcpp::Subscription<rtde_handarm::msg::FtsensorMsg>::SharedPtr ft_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr YSurfN_Fext_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr UR10e_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_Jangle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_wrench_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Callbacks
    // void cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void PbIterCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void JointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    // void VRdataCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    // void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Loop
    void CalculateAndPublishJoint();
};

#endif // JOINTCONTROL_H
