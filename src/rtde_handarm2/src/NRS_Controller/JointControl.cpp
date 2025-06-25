#include "JointControl.h"
#include <iostream>
#include <memory>

JointControl::JointControl()
: Node("joint_control_node")
{
    // Publishers
    YSurfN_Fext_pub_ = this->create_publisher<std_msgs::msg::Float64>("YSurfN_Fext", 20);
    UR10e_mode_pub_ = this->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 20);
    UR10_Jangle_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_Jangle", 20);
    UR10_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_pose", 20);
    UR10_wrench_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_wrench", 20);

    // Subscribers
    // UR10e_mode_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
    //     "/Yoon_UR10e_mode", 20,
    //     std::bind(&JointControl::cmdModeCallback, this, std::placeholders::_1));

    // joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    //     "/yoon_UR10e_joint_cmd", 100,
    //     std::bind(&JointControl::jointCmdCallback, this, std::placeholders::_1));

    PB_iter_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/Yoon_PbNum_cmd", 100,
        std::bind(&JointControl::PbIterCallback, this, std::placeholders::_1));

    // VR_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //     "/vive/pos0", 100,
    //     std::bind(&JointControl::VRdataCallback, this, std::placeholders::_1));

    // joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    //     "/isaac_joint_states", rclcpp::SensorDataQoS(),
    //     std::bind(&JointControl::jointStateCallback, this, std::placeholders::_1));

    // // Timer
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(2),
    //     std::bind(&JointControl::CalculateAndPublishJoint, this));
}

JointControl::~JointControl() {}




//// void PbIterCallback(const std_msgs::UInt16::ConstPtr& msg)
void JointControl::PbIterCallback(std_msgs::msg::UInt16::SharedPtr msg)
{
    PB_iter_cmd = msg->data;
    PB_iter_cur = 1; // 1 is right
}



void JointControl::CalculateAndPublishJoint()
{
    // 작업 수행에 따라 ctrl 값을 기준으로 제어 분기 가능
    // ex) if(ctrl == 1) { ... }
}
