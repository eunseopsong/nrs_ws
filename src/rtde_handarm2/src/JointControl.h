#ifndef JOINTCONTROL_H
#define JOINTCONTROL_H

// ============================================================================
//  [JointControl.h]
//  UR10e ROS2 제어 노드 헤더
//  ---------------------------------------------------------------------------
//  주요 기능:
//   1) Joint & EE 상태 구독 및 퍼블리시
//   2) Cartesian Admittance / Force Control 수행
//   3) Trajectory Playback (InitMove → PathFollow → ReturnHomePose)
//   4) /calibrated_pose + /ftsensor/measured_Cvalue 로부터 텔레옵 값을 받아
//      control_mode == 4 에서 PathFollow와 동일한 힘제어 체인 실행
// ============================================================================

// ROS2 core
#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>   // FT sensor (/ftsensor/measured_Cvalue)

// std
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Dense>

// ament
#include <ament_index_cpp/get_package_share_directory.hpp>

// project
#include "var_ur10e_main.h"
#include "func_ur10e_main.h"

// admittance
#include "rtde_handarm2/ForceControl/admittance_control.hpp"

class JointControl : public std::enable_shared_from_this<JointControl>
{
public:
    explicit JointControl(const rclcpp::Node::SharedPtr& node);
    ~JointControl();

    // 메인 제어 루프
    void CalculateAndPublishJoint();

    // subscribers
    void getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg);
    void cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void PbIterCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void JointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void FtCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // 텔레옵 pose 콜백 (/calibrated_pose → x y z r p yaw)
    void calibratedPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // 텔레옵 force 콜백 (/ftsensor/measured_Cvalue → fx fy fz)
    void ftSensorCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    // state / trajectory
    void UpdateState();
    bool InitMove(double dt_s);
    bool PathFollow(double dt_s);
    bool ReturnHomePose(double dt_s);

    // control_mode 3/4 공통 힘제어 체인 (step 2~7)
    void runCartesianForceChain(
        const Eigen::Vector3d& Xd,
        const Eigen::Vector3d& RPYd,
        const Eigen::Vector3d& Fd,
        double dt_s);

private:
    // rad → [-π, π] wrap
    inline double wrapToPi(double a) const {
        const double two_pi = 2.0 * M_PI;
        a = std::fmod(a + M_PI, two_pi);
        if (a < 0) a += two_pi;
        return a - M_PI;
    }

    // 홈 복귀 관련
    bool   return_active_   = false;
    double return_elapsed_  = 0.0;
    double return_duration_ = 0.0;
    Eigen::Matrix<double, 6, 1> return_start_q_;

    // ROS2 core
    rclcpp::Node::SharedPtr       node_;
    rclcpp::TimerBase::SharedPtr  timer_;
    bool                          running = true;

    // subscribers
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            UR10e_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            PB_iter_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           ft_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr calibrated_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr       ftsensor_sub_;

    // publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    force_ext_base_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr               ur10e_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    UR10_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    UR10_wrench_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_commands_pub_;

    // reusable msgs
    std_msgs::msg::UInt16               UR10e_mode_msg_;
    std_msgs::msg::Float64              YSurfN_Fext_msg_;
    std_msgs::msg::Float64MultiArray    UR10_pose_msg_;
    std_msgs::msg::Float64MultiArray    UR10_wrench_msg_;
    sensor_msgs::msg::JointState        joint_state_;

    // robot state
    std::array<double, 6> joint_pos{};
    Eigen::Matrix4d       T_current;
    Eigen::Vector3d       pos_current;
    Eigen::Vector3d       rpy_current;
    double                contact_force = 0.0;
    int                   key_MODE      = 0;

    // 텔레옵에서 받은 pose
    bool            teleop_pose_valid_ = false;
    Eigen::Vector3d teleop_xyz_        = Eigen::Vector3d::Zero();
    Eigen::Vector3d teleop_rpy_        = Eigen::Vector3d::Zero();

    // 텔레옵에서 받은 force (FT sensor)
    bool            teleop_force_valid_ = false;
    Eigen::Vector3d teleop_force_       = Eigen::Vector3d::Zero();

    // IK까지 끝난 조인트 버전 (fallback 용)
    Eigen::Matrix<double, 6, 1> teleop_qd_        = Eigen::Matrix<double, 6, 1>::Zero();
    bool                        teleop_qd_valid_  = false;

    // time
    std::chrono::system_clock::time_point start;
    std::chrono::duration<double> pre_now{0.0};
    double milisec = 0.0;

    // admittance params
    Eigen::VectorXd             qd_pre;
    Eigen::Matrix<double, 6, 1> Hadmit_M;
    Eigen::Matrix<double, 6, 1> Hadmit_D;
    Eigen::Matrix<double, 6, 1> Hadmit_K;

    // trajectory
    int path_exe_counter = 0;
    std::vector<std::vector<double>> joint_trajectory_;

    // debug pubs
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step3_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step4_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step5_pub_;
};

#endif  // JOINTCONTROL_H
