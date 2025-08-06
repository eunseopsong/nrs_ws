#ifndef JOINTCONTROL_H
#define JOINTCONTROL_H

#include "var_ur10e_main.h"

class JointControl : public std::enable_shared_from_this<JointControl>
{
public:
    JointControl(const rclcpp::Node::SharedPtr& node);  // Consructor
    ~JointControl();

    void CalculateAndPublishJoint();
    // void getActualQ();  // Isaac Sim에서 받은 joint 값을 RArm.qc로 저장
    void getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg); // fix on 2025.08.05

private:
    //////// ROS2 Node 및 동기화 자료 ////////
    rclcpp::Node::SharedPtr node_;

    std::array<double, 6> joint_pos;

    //////// Subscribers ////////
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            UR10e_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            PB_iter_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  VR_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_states_sub_;
    //// rclcpp::Subscription<rtde_handarm::msg::FtsensorMsg>::SharedPtr ft_sub_;

    //////// Publishers ////////
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           YSurfN_Fext_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr            UR10e_mode_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_Jangle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_wrench_pub_;


    // for isaac_joint_commands
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     joint_commands_pub_;
    sensor_msgs::msg::JointState joint_state_;



    bool loadFirstTrajectoryPoint(
        const std::string& filepath,
        float& LD_X, float& LD_Y, float& LD_Z,
        float& LD_Roll, float& LD_Pitch, float& LD_Yaw,
        float& LD_CFx, float& LD_CFy, float& LD_CFz);



    std_msgs::msg::UInt16 UR10e_mode_msg_;
    // std_msgs::msg::Float64MultiArray UR10_Jangle_msg_;
    std_msgs::msg::Float64MultiArray UR10_pose_msg_;
    std_msgs::msg::Float64MultiArray UR10_wrench_msg_;
    std_msgs::msg::Float64 YSurfN_Fext_msg_;

    //////// Timer ////////
    rclcpp::TimerBase::SharedPtr timer_;

    //////// 콜백 함수 ////////
    void cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void PbIterCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void JointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void VRdataCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    // void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);  // <-- Isaac Sim에서 /isaac_joint_states

    //////// 상태 변수 및 제어 파라미터 ////////
    bool running = true;

    std::unique_ptr<nrs_msgmonitoring2::MsgMonitoring> AdaptiveK_msg_;
    std::unique_ptr<nrs_msgmonitoring2::MsgMonitoring> FAAC3step_msg_;

    int priority;

    std::chrono::system_clock::time_point start;
    std::chrono::duration<double> pre_now;

    VectorXd Init_qc;
    VectorXd qd_pre;
    VectorXd qc_pre;
    VectorXd dqd_pre;

    int path_exe_counter = 0;

    Eigen::Matrix<double, 6, 1> Hadmit_M;
    Eigen::Matrix<double, 6, 1> Hadmit_D;
    Eigen::Matrix<double, 6, 1> Hadmit_K;

    VectorXd Hspring_mode_init_pos;

    int key_MODE;

    FILE* path_recording_joint;
    FILE* EXPdata1;

    double _count;
};

#endif  // JOINTCONTROL_H
