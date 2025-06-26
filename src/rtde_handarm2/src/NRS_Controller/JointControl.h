#ifndef JOINTCONTROL_H
#define JOINTCONTROL_H

#include "func.h"

class JointControl : public rclcpp::Node, public std::enable_shared_from_this<JointControl>
{
public:
    JointControl(const rclcpp::Node::SharedPtr& node);  // 생성자 인자로 node 받기
    ~JointControl();

    void CalculateAndPublishJoint();

private:
    //////// parameters ////////
    bool running = true;

    rclcpp::Node::SharedPtr node_;  // 노드를 멤버로 보관

    std::unique_ptr<nrs_msgmonitoring2::MsgMonitoring> AdaptiveK_msg_;
    std::unique_ptr<nrs_msgmonitoring2::MsgMonitoring> FAAC3step_msg_;

    // 실시간 우선순위 설정 관련
    int priority;

    // 시간 측정용
    std::chrono::system_clock::time_point start;
    std::chrono::duration<double> pre_now;

    // 로봇 팔 상태
    VectorXd Init_qc;
    VectorXd qd_pre;
    VectorXd qc_pre;
    VectorXd dqd_pre;

    int path_exe_counter = 0;

    // admittance 제어 관련
    Eigen::Matrix<double, 6, 1> Hadmit_M;
    Eigen::Matrix<double, 6, 1> Hadmit_D;
    Eigen::Matrix<double, 6, 1> Hadmit_K;

    VectorXd Hspring_mode_init_pos;

    // key 입력 (MODE 선택)
    int key_MODE;

    // 데이터 저장용
    FILE* path_recording_joint;
    FILE* EXPdata1;

    //////// Node Declaration ////////
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            UR10e_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            PB_iter_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  VR_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_state_sub_;
    //// rclcpp::Subscription<rtde_handarm::msg::FtsensorMsg>::SharedPtr ft_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           YSurfN_Fext_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr            UR10e_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_Jangle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr UR10_wrench_pub_;

    std_msgs::msg::UInt16 UR10e_mode_msg_;
    std_msgs::msg::Float64MultiArray UR10_Jangle_msg_;
    std_msgs::msg::Float64MultiArray UR10_pose_msg_;
    std_msgs::msg::Float64MultiArray UR10_wrench_msg_;
    std_msgs::msg::Float64 YSurfN_Fext_msg_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Callbacks
    void cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void PbIterCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void JointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void VRdataCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Loop
    // void CalculateAndPublishJoint();
};

#endif // JOINTCONTROL_H
