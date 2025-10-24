#ifndef JOINTCONTROL_H
#define JOINTCONTROL_H

// ============================================================================
//  [헤더 개요]
//  - UR10e 매니퓰레이터 제어용 ROS2 C++ 클래스
//  - Isaac Sim 및 실제 RTDE 통신 환경 모두 호환 가능
//  - 주요 기능:
//      1) 조인트 상태 갱신 및 퍼블리시
//      2) Cartesian Admittance/Force Control
//      3) Trajectory Playback (InitMove → PathFollow → ReturnHomePose)
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

// ========================= 수학/선형대수 =========================
#include <Eigen/Dense>

// ========================= ROS2 유틸 헤더 =========================
#include <ament_index_cpp/get_package_share_directory.hpp>

// ========================= 프로젝트 내부 헤더 =========================
#include "var_ur10e_main.h"


// ============================================================================
//  JointControl Class
//  --------------------------------------------------------------------------
//  • ROS2 기반 UR10e 주 제어 노드
//  • Mode별 제어 로직을 수행 (예: Joint/EE 제어, Force 제어, Trajectory 재생 등)
// ============================================================================
class JointControl : public std::enable_shared_from_this<JointControl>
{
public:
    // ------------------------------------------------------------------------
    //  Constructor / Destructor
    // ------------------------------------------------------------------------
    explicit JointControl(const rclcpp::Node::SharedPtr& node);
    ~JointControl();

    // ------------------------------------------------------------------------
    //  Main Control Loop
    // ------------------------------------------------------------------------
    /**
     * @brief Main loop called periodically by ROS2 timer.
     *        - 각 모드(control_mode)에 따라 제어 알고리즘 분기
     *        - Joint command 계산 및 퍼블리시 수행
     */
    void CalculateAndPublishJoint();

    // ------------------------------------------------------------------------
    //  Subscriber Callbacks
    // ------------------------------------------------------------------------
    /** @brief JointState subscriber callback (현재 로봇 상태 수신) */
    void getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg);

    /** @brief Commanded control mode callback (외부에서 제어 모드 변경 시 호출) */
    void cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg);

    /** @brief Playback iteration callback (외부 반복 명령 수신 시) */
    void PbIterCallback(const std_msgs::msg::UInt16::SharedPtr msg);

    /** @brief Joint command callback (수동 명령 수신 시) */
    void JointCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    /** @brief Force/Torque sensor callback (외력 측정 데이터 수신) */
    void FtCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // ------------------------------------------------------------------------
    //  State Update
    // ------------------------------------------------------------------------
    /**
     * @brief Forward Kinematics를 이용해 EE 위치 및 자세 갱신
     *        (현재 조인트 상태 → pos_current, rpy_current)
     */
    void UpdateState();

    // ------------------------------------------------------------------------
    //  Force / Admittance Control
    // ------------------------------------------------------------------------
    /**
     * @brief Cartesian Admittance Control or FAAC(Force Adaptive Admittance Control)
     *        - 외력 피드백 기반으로 EE 위치/자세를 조정
     *        - 접촉 환경에서의 compliant motion 수행
     */
    void ControlForce();

    // ------------------------------------------------------------------------
    //  Trajectory Playback 관련 함수
    // ------------------------------------------------------------------------

    /**
     * @brief InitMove : 초기 위치 → TXT 첫 waypoint까지 선형보간 이동
     * @param dt_s      : 제어 주기 [s]
     * @return true면 이동 완료, false면 이동 중
     *
     * 기능:
     * - 현재 로봇 위치(RArm.xc)에서 txt trajectory 첫 포인트까지 부드럽게 이동
     * - Quaternion slerp 기반 회전 보간
     * - 도착 시 rewind() 호출 및 PathFollow 단계로 전환
     */
    bool InitMove(double dt_s);

    /**
     * @brief PathFollow : TXT 파일 trajectory 추종
     * @param dt_s : 제어 주기 [s]
     * @return true면 추종 중, false면 종료
     *
     * 기능:
     * - (x,y,z,r,p,y,fx,fy,fz) 데이터를 txt에서 순차적으로 읽어
     *   End-effector pose 및 접촉힘 정보를 반영
     * - IK 계산 후 joint command 퍼블리시
     * - EOF 도달 시 ReturnHomePose 단계로 전환
     */
    bool PathFollow(double dt_s);

    /**
     * @brief ReturnHomePose : Trajectory 종료 후 홈 자세 복귀
     * @param dt_s : 제어 주기 [s]
     * @return true면 복귀 중, false면 완료
     *
     * 기능:
     * - PathFollow 종료 후 4초간 선형 보간으로 초기 자세 복귀
     * - smooth home motion (q_start → q_home)
     */
    bool ReturnHomePose(double dt_s);

private:
    // =========================================================================
    //  [Trajectory Return 관리 변수]
    // =========================================================================
    bool return_active_ = false;                     // 홈 복귀 중 여부
    double return_elapsed_ = 0.0;                    // 복귀 경과 시간 [s]
    double return_duration_ = 0.0;                   // 복귀 총 소요 시간 [s]
    Eigen::Matrix<double, 6, 1> return_start_q_;     // 복귀 시작 joint 각도 벡터

    // =========================================================================
    //  ROS2 구성요소
    // =========================================================================
    rclcpp::Node::SharedPtr node_;                   // ROS2 Node handle
    rclcpp::TimerBase::SharedPtr timer_;             // 주 제어 루프용 타이머
    bool running = true;                             // 루프 실행 상태 flag

    // -------------------------------------------------------------------------
    //  ROS2 Subscribers
    // -------------------------------------------------------------------------
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            UR10e_mode_sub_;   // 모드 명령 구독
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;    // 수동 Joint 명령
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr            PB_iter_sub_;      // Playback 반복 구독
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  VR_sub_;           // VR Tracker Pose
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_states_sub_; // 실제 Joint 상태
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           ft_sub_;           // 외력 센서 데이터

    // -------------------------------------------------------------------------
    //  ROS2 Publishers
    // -------------------------------------------------------------------------
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              YSurfN_Fext_pub_;  // 외력 N 축 값 퍼블리시
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr               UR10e_mode_pub_;   // 제어 모드 브로드캐스트
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    UR10_pose_pub_;    // EE pose 퍼블리시
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    UR10_wrench_pub_;  // 힘/토크 퍼블리시
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_commands_pub_; // Joint 명령 퍼블리시

    // -------------------------------------------------------------------------
    //  ROS 메시지 객체 (재사용 목적)
    // -------------------------------------------------------------------------
    std_msgs::msg::UInt16               UR10e_mode_msg_;
    std_msgs::msg::Float64              YSurfN_Fext_msg_;
    std_msgs::msg::Float64MultiArray    UR10_pose_msg_;
    std_msgs::msg::Float64MultiArray    UR10_wrench_msg_;
    sensor_msgs::msg::JointState        joint_state_;

    // =========================================================================
    //  로봇 상태 변수
    // =========================================================================
    std::array<double, 6> joint_pos{};               // 현재 Joint 각도 [rad]
    Eigen::Matrix4d       T_current;                 // 현재 EE Homogeneous Transform
    Eigen::Vector3d       pos_current;               // 현재 EE 위치 [m]
    Eigen::Vector3d       rpy_current;               // 현재 EE 자세 [rad]
    double contact_force = 0.0;                      // 현재 접촉 힘 [N]
    int key_MODE = 0;                                // 현재 제어 모드 상태
    int priority = 0;                                // 내부 우선순위 관리 변수

    // =========================================================================
    //  시간 관리
    // =========================================================================
    std::chrono::system_clock::time_point start;     // 루프 시작 시간
    std::chrono::duration<double> pre_now{0.0};      // 이전 루프와의 시간차
    double milisec = 0.0;                            // 밀리초 단위 루프 주기 계산용

    // =========================================================================
    //  제어 관련 파라미터 (Admittance 등)
    // =========================================================================
    Eigen::VectorXd Init_qc;                         // 초기 조인트 명령
    Eigen::VectorXd qd_pre;                          // 이전 desired joint 값
    Eigen::VectorXd qc_pre;                          // 이전 current joint 값
    Eigen::VectorXd dqd_pre;                         // 이전 desired joint 속도
    Eigen::Matrix<double, 6, 1> Hadmit_M;            // Admittance 질량 행렬
    Eigen::Matrix<double, 6, 1> Hadmit_D;            // Admittance 감쇠 행렬
    Eigen::Matrix<double, 6, 1> Hadmit_K;            // Admittance 강성 행렬
    Eigen::VectorXd Hspring_mode_init_pos;           // Spring mode 초기 위치 저장

    // =========================================================================
    //  Trajectory & Data Logging
    // =========================================================================
    int path_exe_counter = 0;                        // Trajectory 실행 횟수 카운터
    std::vector<std::vector<double>> joint_trajectory_; // Joint trajectory 저장 벡터
    FILE* path_recording_joint = nullptr;            // Joint trajectory 기록 파일 포인터
    FILE* EXPdata1 = nullptr;                        // 실험 데이터 로깅 파일 포인터

    // =========================================================================
    //  모니터링 객체 (AdaptiveK, FAAC)
    // =========================================================================
    std::unique_ptr<nrs_msgmonitoring2::MsgMonitoring> AdaptiveK_msg_; // Adaptive gain monitor
    std::unique_ptr<nrs_msgmonitoring2::MsgMonitoring> FAAC3step_msg_; // FAAC 3-step control monitor
};

#endif  // JOINTCONTROL_H
