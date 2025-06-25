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

    PB_iter_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/Yoon_PbNum_cmd", 100,
        std::bind(&JointControl::PbIterCallback, this, std::placeholders::_1));

    joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/yoon_UR10e_joint_cmd", 100,
        std::bind(&JointControl::JointCmdCallback, this, std::placeholders::_1));

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


void JointControl::PbIterCallback(std_msgs::msg::UInt16::SharedPtr msg)
{
    PB_iter_cmd = msg->data;
    PB_iter_cur = 1; // 1 is right
}

void JointControl::JointCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    mjoint_cmd = msg->data;
    printf("\nSelected joint: %1.0f, Target relative joint angle: %4f \n", mjoint_cmd[0],mjoint_cmd[1]);

    double Tar_pos[] = {0.0};
    double Tar_vel[] = {0.0}; 
    double Waiting_time[] = {0,0}; // Waiting time, unit : s, 0이 아닌 값을 넣으려면 동일 array 위치의 pos와 vel은 0 

    double Vel_set = 0.1; // rad/s

    // Set the joint relative target angle
    Tar_pos[0] = fabs(mjoint_cmd[1]); 
    // Set the joint target velocity
    if(mjoint_cmd[1]>=0) Tar_vel[0] = Vel_set;
    else Tar_vel[0] = -Vel_set;

    Joint_path_start <<RArm.qc(0),RArm.qc(1),RArm.qc(2),RArm.qc(3),RArm.qc(4),RArm.qc(5);

    Path_point_num = J_single.Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));
    if(Path_point_num != -1)
    {
        ctrl = 1;
        memcpy(message_status,path_gen_done,sizeof(path_gen_done));
        path_done_flag = true;

    }
}



void JointControl::CalculateAndPublishJoint()
{
    // 작업 수행에 따라 ctrl 값을 기준으로 제어 분기 가능
    // ex) if(ctrl == 1) { ... }
}
