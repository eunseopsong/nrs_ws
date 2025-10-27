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
// ============================================================================


// ========================= ROS2 기본 헤더 =========================
#include <rclcpp/rclcpp.hpp>

// ========================= ROS 메시지 타입 =========================
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// ========================= 표준 라이브러리 =========================
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <sys/stat.h>

// ========================= 수학 / 선형대수 =========================
#include <Eigen/Dense>

// ========================= ROS2 유틸 헤더 =========================
#include <ament_index_cpp/get_package_share_directory.hpp>

// ========================= 프로젝트 내부 헤더 =========================
#include "var_ur10e_main.h"



// ForceControl (admittance & FAAC)
#include "rtde_handarm2/ForceControl/admittance_control.hpp"



// ============================================================================
//  JointControl Class
//  ---------------------------------------------------------------------------
//  • ROS2 기반 UR10e 제어 노드
//  • Isaac Sim / 실제 RTDE 통신 환경 모두 호환
// ============================================================================
class JointControl : public std::enable_shared_from_this<JointControl>
{
public:
    // ------------------------------------------------------------------------
    //  생성자 / 소멸자
    // ------------------------------------------------------------------------
    explicit JointControl(const rclcpp::Node::SharedPtr& node);
    ~JointControl();

    // ------------------------------------------------------------------------
    //  주 제어 루프 (Timer Callback 내부 호출)
    // ------------------------------------------------------------------------
    void CalculateAndPublishJoint();

    // ------------------------------------------------------------------------
    //  ROS Subscriber 콜백 함수
    // ------------------------------------------------------------------------
    void getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg);
    void cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void PbIterCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void JointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void FtCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // ------------------------------------------------------------------------
    //  로봇 상태 갱신 / 제어 알고리즘
    // ------------------------------------------------------------------------
    void UpdateState();      // Forward Kinematics 기반 EE 상태 갱신

    // ------------------------------------------------------------------------
    //  Trajectory Playback 관련 함수
    // ------------------------------------------------------------------------
    bool InitMove(double dt_s);          // 현재 위치 → TXT 첫 포인트 보간 이동
    bool PathFollow(double dt_s);        // TXT trajectory 추종
    bool ReturnHomePose(double dt_s);    // 홈 자세 복귀

private:
    // =========================================================================
    //  홈 자세 복귀 관리 변수
    // =========================================================================
    bool return_active_ = false;
    double return_elapsed_ = 0.0;
    double return_duration_ = 0.0;
    Eigen::Matrix<double, 6, 1> return_start_q_;

    // =========================================================================
    //  ROS2 구성요소
    // =========================================================================
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool running = true;

    // -------------------------------------------------------------------------
    //  ROS2 Subscribers
    // -------------------------------------------------------------------------
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            UR10e_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            PB_iter_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           ft_sub_;

    // -------------------------------------------------------------------------
    //  ROS2 Publishers
    // -------------------------------------------------------------------------
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    force_ext_base_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr               UR10e_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    UR10_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    UR10_wrench_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_commands_pub_;

    // -------------------------------------------------------------------------
    //  ROS 메시지 객체 (재사용)
    // -------------------------------------------------------------------------
    std_msgs::msg::UInt16               UR10e_mode_msg_;
    std_msgs::msg::Float64              YSurfN_Fext_msg_;
    std_msgs::msg::Float64MultiArray    UR10_pose_msg_;
    std_msgs::msg::Float64MultiArray    UR10_wrench_msg_;
    sensor_msgs::msg::JointState        joint_state_;

    // =========================================================================
    //  로봇 상태 변수
    // =========================================================================
    std::array<double, 6> joint_pos{};
    Eigen::Matrix4d       T_current;
    Eigen::Vector3d       pos_current;
    Eigen::Vector3d       rpy_current;
    double contact_force = 0.0;
    int key_MODE = 0;

    // =========================================================================
    //  시간 관리
    // =========================================================================
    std::chrono::system_clock::time_point start;
    std::chrono::duration<double> pre_now{0.0};
    double milisec = 0.0;

    // =========================================================================
    //  제어 파라미터 (Admittance Control)
    // =========================================================================
    Eigen::VectorXd qd_pre;
    Eigen::Matrix<double, 6, 1> Hadmit_M;
    Eigen::Matrix<double, 6, 1> Hadmit_D;
    Eigen::Matrix<double, 6, 1> Hadmit_K;

    // =========================================================================
    //  Trajectory 관리 변수
    // =========================================================================
    int path_exe_counter = 0;
    std::vector<std::vector<double>> joint_trajectory_;

    // =========================================================================
    //  디버그용 publishers
    // =========================================================================
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step4_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_step5_pub_;

};

#endif  // JOINTCONTROL_H
