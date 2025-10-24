#ifndef JOINTCONTROL_H
#define JOINTCONTROL_H

#include "var_ur10e_main.h"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <atomic>
#include <memory>
#include <array>
#include <vector>
#include <chrono>

// ============================================================================
//  JointControl Class (UR10e ROS2 main control node)
// ============================================================================
class JointControl : public std::enable_shared_from_this<JointControl>
{
public:
    explicit JointControl(const rclcpp::Node::SharedPtr& node);
    ~JointControl();

    // 주 제어 루프 (Timer callback 내부 호출)
    void CalculateAndPublishJoint();

    // 실제 JointState 수신 콜백
    void getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
    // --------------------------- //
    // ROS2 Node 및 기본 구성요소
    // --------------------------- //
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool running = true;

    // --------------------------- //
    // Subscribers
    // --------------------------- //
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            UR10e_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            PB_iter_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  VR_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           ft_sub_; // 외력 (Contact Force)

    // --------------------------- //
    // Publishers
    // --------------------------- //
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              YSurfN_Fext_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr               UR10e_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    UR10_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    UR10_wrench_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_commands_pub_;

    // --------------------------- //
    // 메시지 객체 (재사용)
    // --------------------------- //
    std_msgs::msg::UInt16               UR10e_mode_msg_;
    std_msgs::msg::Float64              YSurfN_Fext_msg_;
    std_msgs::msg::Float64MultiArray    UR10_pose_msg_;
    std_msgs::msg::Float64MultiArray    UR10_wrench_msg_;
    sensor_msgs::msg::JointState        joint_state_;

    // --------------------------- //
    // 콜백 함수
    // --------------------------- //
    void cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void PbIterCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void JointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void VRdataCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void FtCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // --------------------------- //
    // 내부 상태 변수
    // --------------------------- //
    std::array<double, 6> joint_pos{};   // 현재 조인트 각도
    double contact_force = 0.0;          // 접촉 힘
    int key_MODE = 0;
    int priority = 0;

    // --------------------------- //
    // 시간 관련
    // --------------------------- //
    std::chrono::system_clock::time_point start;
    std::chrono::duration<double> pre_now{0.0};
    double milisec = 0.0;

    // --------------------------- //
    // 제어 변수 및 파라미터
    // --------------------------- //
    Eigen::VectorXd Init_qc;
    Eigen::VectorXd qd_pre;
    Eigen::VectorXd qc_pre;
    Eigen::VectorXd dqd_pre;
    Eigen::Matrix<double, 6, 1> Hadmit_M;
    Eigen::Matrix<double, 6, 1> Hadmit_D;
    Eigen::Matrix<double, 6, 1> Hadmit_K;
    Eigen::VectorXd Hspring_mode_init_pos;

    // --------------------------- //
    // Trajectory 및 데이터 로깅
    // --------------------------- //
    int path_exe_counter = 0;
    std::vector<std::vector<double>> joint_trajectory_;
    FILE* path_recording_joint = nullptr;
    FILE* EXPdata1 = nullptr;

    // --------------------------- //
    // 모니터링 객체 (AdaptiveK, FAAC)
    // --------------------------- //
    std::unique_ptr<nrs_msgmonitoring2::MsgMonitoring> AdaptiveK_msg_;
    std::unique_ptr<nrs_msgmonitoring2::MsgMonitoring> FAAC3step_msg_;
};

#endif  // JOINTCONTROL_H
