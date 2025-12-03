// JointControl.cpp (v2025-rt-safe, TCP-only version)
// - 모든 외부/내부 좌표를 EE/TCP 기준(= Ycontact TCP)으로 완전 통일
// - dt 측정, 조인트 소스 단일화, 힘 변환/퍼블리시 최적화
// - Path/Teleop/IK 모드에서 Xd, Act_XYZ 모두 TCP 좌표 사용
// - InitMove/PathFollow/ReturnHomePose/runCartesianForceChain 의 실제 구현은
//   func_ur10e_main.cpp 에 있음

#include "JointControl.h"
#include "func_ur10e_main.h"   // set_status, yaml_get_path, readDoublesFromStdin, InitMove/PathFollow/...

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <filesystem>
#include <iostream>
#include <limits>

#include <geometry_msgs/msg/wrench.hpp>

constexpr int DOF = 6;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// ============================================================================
// 생성자 / 소멸자
// ============================================================================
JointControl::JointControl(const rclcpp::Node::SharedPtr& node)
: node_(node), milisec(0.0)
{
  // Debug Publishers
  debug_step1_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/debug_step1", 10);
  debug_step2_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/debug_step2", 10);
  debug_step3_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/debug_step3", 10);
  debug_step4_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/debug_step4", 10);
  debug_step5_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/debug_step5", 10);

  // Publishers
  force_ext_base_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("force_ext_base", 20);
  ur10e_mode_pub_     = node_->create_publisher<std_msgs::msg::UInt16>("ur10e_mode_cmd", 20);
  UR10_pose_pub_      = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_pose", 20);
  UR10_wrench_pub_    = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_wrench", 20);
  joint_commands_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands" , 20);

  // JointState 고정 필드
  joint_state_.name = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
  joint_state_.position.resize(DOF, 0.0);

  // Subscribers
  UR10e_mode_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
    "ur10e_mode_cmd", 20,
    std::bind(&JointControl::cmdModeCallback, this, std::placeholders::_1));

  PB_iter_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
    "/Yoon_PbNum_cmd", 100,
    std::bind(&JointControl::PbIterCallback, this, std::placeholders::_1));

  joint_cmd_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/yoon_UR10e_joint_cmd", 100,
    std::bind(&JointControl::JointCmdCallback, this, std::placeholders::_1));

  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/isaac_joint_states", rclcpp::QoS(10),
    std::bind(&JointControl::getActualQ, this, std::placeholders::_1));

  ft_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/contact/force_magnitude", rclcpp::SensorDataQoS(),
    std::bind(&JointControl::FtCallback, this, std::placeholders::_1)
  );

  calibrated_pose_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/calibrated_pose", 10,
    std::bind(&JointControl::calibratedPoseCallback, this, std::placeholders::_1));

  ftsensor_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
    "/ftsensor/measured_Cvalue", 10,
    std::bind(&JointControl::ftSensorCallback, this, std::placeholders::_1));

  // 🔹 ACT inference node output 구독
  act_joints_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/action_joints", 10,
    std::bind(&JointControl::actJointsCallback, this, std::placeholders::_1));

  act_force_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
    "/action_force", 10,
    std::bind(&JointControl::actForceCallback, this, std::placeholders::_1));

  // Timer (2 ms)
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(2),
    std::bind(&JointControl::CalculateAndPublishJoint, this));

  // 파일 핸들 정리
  if (hand_g_recording)    { std::fclose(hand_g_recording);    hand_g_recording    = nullptr; }
  if (Discre_P_recording)  { std::fclose(Discre_P_recording);  Discre_P_recording  = nullptr; }
  if (VRCali_UR10CB_EE)    { std::fclose(VRCali_UR10CB_EE);    VRCali_UR10CB_EE    = nullptr; }
  if (VRCali_UR10CB_VR)    { std::fclose(VRCali_UR10CB_VR);    VRCali_UR10CB_VR    = nullptr; }
  if (Hand_G_playback)     { std::fclose(Hand_G_playback);     Hand_G_playback     = nullptr; }
  if (path_recording_pos)  { std::fclose(path_recording_pos);  path_recording_pos  = nullptr; }
  if (path_recording_joint){ std::fclose(path_recording_joint);path_recording_joint = nullptr; }
  if (EXPdata1)            { std::fclose(EXPdata1);            EXPdata1            = nullptr; }

  // 상태 초기화
  set_status(message_status, "Motion stop");
  Desired_XYZ.setZero();
  Desired_RPY.setZero();
  Contact_Rot_force.setZero();

  teleop_pose_valid_  = false;
  teleop_force_valid_ = false;
  teleop_xyz_.setZero();
  teleop_rpy_.setZero();
  teleop_force_.setZero();

  // 🔹 ACT action 저장 변수 초기화
  act_joints_valid_ = false;
  act_force_valid_  = false;
  act_joints_.setZero();
  act_force_.setZero();
}

JointControl::~JointControl() {
  if (hand_g_recording)     std::fclose(hand_g_recording);
  if (Discre_P_recording)   std::fclose(Discre_P_recording);
  if (VRCali_UR10CB_EE)     std::fclose(VRCali_UR10CB_EE);
  if (VRCali_UR10CB_VR)     std::fclose(VRCali_UR10CB_VR);
  if (Hand_G_playback)      std::fclose(Hand_G_playback);
  if (path_recording_pos)   std::fclose(path_recording_pos);
  if (path_recording_joint) std::fclose(path_recording_joint);
  if (EXPdata1)             std::fclose(EXPdata1);
}

// ============================================================================
// Main control loop
// ============================================================================
void JointControl::CalculateAndPublishJoint() {
  // dt 계산
  static auto t_prev = std::chrono::steady_clock::now();
  const auto t_now   = std::chrono::steady_clock::now();
  double dt_s = std::chrono::duration<double>(t_now - t_prev).count();
  t_prev = t_now;
  if (dt_s <= 0.0 || dt_s > 0.2) dt_s = 0.01;

  milisec += 2; // Simulation time (ms 단위 카운터)

  // 속도/가속도 초기화
  for (int i = 0; i < DOF; i++) {
    RArm.ddqd(i) = 0;
    RArm.dqd(i)  = 0;
    RArm.dqc(i)  = 0;
  }
  RArm.qd = RArm.qc;
  RArm.qt = RArm.qc;

  // =====================================================================
  // FK: 항상 Base -> TCP (Ycontact 기준) 으로 통일 (현재 qc 기준)
  // =====================================================================
  AKin.Ycontact_ForwardK_T(&RArm);   // Base -> TCP
  AKin.Rotation2EulerAngle(&RArm);   // Tc(TCP) -> thc (RPY)
  RArm.Td = RArm.Tc;                 // 기본적으로 Td 도 현재 TCP pose 로 맞춰둠

  // 가시화용
  for (int i = 0; i < DOF; ++i) joint_pos[i] = RArm.qc(i);

  // TCP->Base 힘 변환 (디버그용)
  Eigen::Vector3d F_base_dbg = Eigen::Vector3d::Zero();
  {
    const Eigen::Matrix3d R_base_TCP = RArm.Tc.block<3,3>(0,0);
    const Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);
    Eigen::Vector3d F_base_raw = R_base_TCP * F_TCP;
    F_base_dbg = -F_base_raw; // 제어에서는 F_ext = -F_base_raw 사용
  }

  const int control_mode     = ctrl.load(std::memory_order_relaxed);
  const int pre_control_mode = pre_ctrl.load(std::memory_order_relaxed);

  // 디버그 프린트
  if (printer_counter >= print_period) {
#if RT_printing
    Eigen::Vector3d tcp_act = RArm.xc;  // 실제 TCP 위치

    printf("======================================== \n");
    printf("Simulation time : %d ms\n", (int)milisec);
    printf("RUN MODE %d (prev %d)\n", control_mode, pre_control_mode);

    printf("q  : %.6f %.6f %.6f %.6f %.6f %.6f\n",
          RArm.qc(0), RArm.qc(1), RArm.qc(2),
          RArm.qc(3), RArm.qc(4), RArm.qc(5));
    printf("qd : %.6f %.6f %.6f %.6f %.6f %.6f\n",
          RArm.qd(0), RArm.qd(1), RArm.qd(2),
          RArm.qd(3), RArm.qd(4), RArm.qd(5));

    printf("Act_XYZ: %.6f %.6f %.6f | Act_RPY: %.6f %.6f %.6f\n",
          tcp_act(0), tcp_act(1), tcp_act(2),
          RArm.thc(0), RArm.thc(1), RArm.thc(2));

    printf("Des_XYZ: %.6f %.6f %.6f | Des_RPY: %.6f %.6f %.6f\n",
          Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2),
          Desired_RPY(0), Desired_RPY(1), Desired_RPY(2));

    printf("Contact Fz: %.2f -> Base: %.6f %.6f %.6f\n",
          contact_force,
          F_base_dbg(0), F_base_dbg(1), F_base_dbg(2));
#endif
    printer_counter = 0;
  } else {
    printer_counter++;
  }

  // Pose / Wrench 토픽 퍼블리시 (TCP 기준)
  UR10_pose_msg_.data.resize(6);
  UR10_wrench_msg_.data.resize(6);
  Eigen::Vector3d tcp_act = RArm.xc;

  for(int i = 0; i < 6; i++){
    if (i < 3) {
      UR10_pose_msg_.data[i] = tcp_act(i);
    } else {
      UR10_pose_msg_.data[i] = RArm.thc(i-3);
    }
    UR10_wrench_msg_.data[i] = ftS2(i);
  }
  UR10_pose_pub_->publish(UR10_pose_msg_);
  UR10_wrench_pub_->publish(UR10_wrench_msg_);

  // =====================================================================
  // 모드별 제어
  // =====================================================================

  // 0) Home pose 유지
  if (control_mode == 0) {
    speedmode = 0;
    static const double HOME_Q[6] =
        { 0.0, -M_PI/2.0, -M_PI/2.0, -M_PI/2.0, +M_PI/2.0, 0.0 };

    for (int i = 0; i < DOF; ++i) {
      RArm.qd(i)   = HOME_Q[i];
      joint_pos[i] = HOME_Q[i];
    }
    RArm.qt  = RArm.qd;
    RArm.dqc << 0,0,0,0,0,0;
    pause_cnt = 0;

    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i)
      joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 1) FK mode: joint space 명령
  if (control_mode == 1) {
    static bool     fk_target_set = false;
    static Vector6d fk_target_q   = Vector6d::Zero();

    if (pre_control_mode != control_mode || !fk_target_set) {
      double qdeg[6];
      if (!readDoublesFromStdin(
              "\n[FK mode] Enter 6 joint angles [deg]: ",
              6, qdeg)) {
        std::cerr << "[FK mode] invalid input. Keep current pose.\n";
        pre_ctrl.store(control_mode, std::memory_order_relaxed);
        return;
      }
      for (int i = 0; i < DOF; ++i)
        fk_target_q(i) = qdeg[i] * M_PI / 180.0;
      fk_target_set = true;
    }

    for (int i = 0; i < DOF; ++i)
      RArm.qd(i) = fk_target_q(i);

    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i)
      joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 2) IK mode: EE/TCP pose 명령 (항상 TCP 기준)
  if (control_mode == 2) {
    static bool            ik_target_set = false;
    static Eigen::Vector3d ik_target_xyz = Eigen::Vector3d::Zero();
    static Eigen::Vector3d ik_target_rpy = Eigen::Vector3d::Zero();

    if (pre_control_mode != control_mode || !ik_target_set) {
      double buf[6];
      if (!readDoublesFromStdin(
              "\n[IK mode] Enter EE pose [x y z r p y] (m, rad): ",
              6, buf)) {
        std::cerr << "[IK mode] invalid input. Keep current pose.\n";
        pre_ctrl.store(control_mode, std::memory_order_relaxed);
        return;
      }

      ik_target_xyz << buf[0], buf[1], buf[2];  // TCP 기준 목표
      ik_target_rpy << buf[3], buf[4], buf[5];

      ik_target_set = true;
    }

    Desired_XYZ = ik_target_xyz;
    Desired_RPY = ik_target_rpy;

    Eigen::Vector3d rpy_copy = ik_target_rpy;
    Eigen::Matrix3d Rd;
    AKin.EulerAngle2Rotation(Rd, rpy_copy);

    // Td 는 Base->TCP 로 가정 (툴 오프셋은 Kinematics 쪽에서 처리)
    RArm.Td <<
        Rd(0,0), Rd(0,1), Rd(0,2), ik_target_xyz(0),
        Rd(1,0), Rd(1,1), Rd(1,2), ik_target_xyz(1),
        Rd(2,0), Rd(2,1), Rd(2,2), ik_target_xyz(2),
        0,       0,       0,       1;

    // 항상 Base->TCP (Ycontact) 기준 IK
    AKin.Ycontact_InverseK_min(&RArm);

    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i)
      joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 3) IK + Admittance mode:
  //    - 터미널에서 [x y z r p y fx fy fz] 한 번 입력
  //    - 1단계: 현재 qc -> 목표 EE pose의 IK(q_target)까지 joint-space로 부드럽게 이동(InitMove처럼)
  //    - 2단계: 목표에 도달 후 runCartesianForceChain으로 어드미턴스+힘제어 수행
  if (control_mode == 3) {
    // ---- 상태 변수들 (이 모드에서만 사용) ----
    static bool            ik_force_cmd_set   = false;  // 입력 9개 받았는지
    static bool            init_phase_active  = false;  // InitMove 단계 진행중인지
    static double          init_interp_s      = 0.0;    // 0~1 보간 인자
    static Vector6d        q_start            = Vector6d::Zero();
    static Vector6d        q_target           = Vector6d::Zero();
    static Eigen::Vector3d ik_force_xyz       = Eigen::Vector3d::Zero();
    static Eigen::Vector3d ik_force_rpy       = Eigen::Vector3d::Zero();
    static Eigen::Vector3d ik_force_Fd        = Eigen::Vector3d::Zero();

    // 모드가 바뀌면 상태 초기화
    if (pre_control_mode != control_mode) {
      ik_force_cmd_set  = false;
      init_phase_active = false;
      init_interp_s     = 0.0;
    }

    // ---------- ① 터미널에서 목표 pose + 힘 입력 ----------
    if (!ik_force_cmd_set) {
      double buf[9];
      if (!readDoublesFromStdin(
              "\n[IK + Admittance mode] Enter [x y z r p y fx fy fz] (m, rad, N): ",
              9, buf)) {
        std::cerr << "[IK + Admittance mode] invalid input. Keep current pose.\n";
        pre_ctrl.store(control_mode, std::memory_order_relaxed);
        return;
      }

      ik_force_xyz << buf[0], buf[1], buf[2];   // TCP 기준 위치
      ik_force_rpy << buf[3], buf[4], buf[5];   // RPY
      ik_force_Fd  << buf[6], buf[7], buf[8];   // 원하는 힘 (N)

      // 디버그용 Desired 값 갱신
      Desired_XYZ = ik_force_xyz;
      Desired_RPY = ik_force_rpy;

      // ---------- ② 목표 EE pose에 대한 IK로 q_target 계산 ----------
      Eigen::Vector3d rpy_tmp = ik_force_rpy;
      Eigen::Matrix3d Rd;
      AKin.EulerAngle2Rotation(Rd, rpy_tmp);

      RArm.Td <<
          Rd(0,0), Rd(0,1), Rd(0,2), ik_force_xyz(0),
          Rd(1,0), Rd(1,1), Rd(1,2), ik_force_xyz(1),
          Rd(2,0), Rd(2,1), Rd(2,2), ik_force_xyz(2),
          0,       0,       0,       1;

#if TCP_standard == 0
      AKin.InverseK_min(&RArm);
#else
      AKin.Ycontact_InverseK_min(&RArm);
#endif
      // IK 결과를 q_target으로 저장
      for (int i = 0; i < DOF; ++i) {
        q_target(i) = RArm.qd(i);
      }

      // 시작 조인트는 현재 qc
      for (int i = 0; i < DOF; ++i) {
        q_start(i) = RArm.qc(i);
      }

      // InitMove 단계 시작
      init_phase_active = true;
      init_interp_s     = 0.0;
      ik_force_cmd_set  = true;
    }

    // ---------- ③ InitMove 단계: q_start -> q_target 부드럽게 이동 ----------
    if (init_phase_active) {
      // 총 이동 시간을 T_move [s]로 가정 (필요하면 파라미터로 빼도 됨)
      const double T_move = 5.0;  // 2초 동안 서서히 이동
      double ds = dt_s / T_move;
      if (ds > 1.0) ds = 1.0;

      init_interp_s += ds;
      if (init_interp_s >= 1.0) {
        init_interp_s   = 1.0;
        init_phase_active = false;  // 보간 끝 → 다음부터 힘제어 단계
      }

      // 선형 보간: q(t) = (1-s)*q_start + s*q_target
      for (int i = 0; i < DOF; ++i) {
        RArm.qd(i) = (1.0 - init_interp_s) * q_start(i)
                     + init_interp_s * q_target(i);
      }

      // 조인트 명령 퍼블리시
      joint_state_.header.stamp = node_->now();
      for (int i = 0; i < DOF; ++i) {
        joint_state_.position[i] = RArm.qd(i);
      }
      joint_commands_pub_->publish(joint_state_);

      // 아직 InitMove 진행중이면 여기서 종료 (힘제어 X)
      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
    }

    // ---------- ④ InitMove 완료 후: Admittance + Force control 실행 ----------
    // 이 시점부터는 항상 같은 목표 Xd/RPYd/Fd 를 기준으로 어드미턴스 제어
    runCartesianForceChain(ik_force_xyz, ik_force_rpy, ik_force_Fd, dt_s);

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }


  // 4) Playback mode: InitMove -> PathFollow -> ReturnHomePose
  if (control_mode == 4) {
    static bool init_done   = false;
    static bool follow_done = false;

    if (pre_control_mode != control_mode) {
      init_done   = false;
      follow_done = false;
    }

    if (!init_done) {
      bool just_finished = InitMove(dt_s);   // func_ur10e_main.cpp
      if (!just_finished) {
        pre_ctrl.store(control_mode, std::memory_order_relaxed);
        return;
      }
      init_done = true;
    }

    if (!follow_done) {
      if (PathFollow(dt_s)) {               // func_ur10e_main.cpp
        pre_ctrl.store(control_mode, std::memory_order_relaxed);
        return;
      } else {
        follow_done = true;
      }
    }

    // PathFollow 끝나면 home 으로 복귀
    ReturnHomePose(dt_s);                   // func_ur10e_main.cpp
    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 5) Teleop wo/ Force Control mode: /calibrated_pose -> AKin.Ycontact_InverseK_min(&RArm);
  if (control_mode == 5) {
    if (teleop_pose_valid_) {
      // ① VR /calibrated_pose 에서 받은 TCP 기준 pose
      Eigen::Vector3d Xd  = teleop_xyz_;   // [x y z]
      Eigen::Vector3d RPY = teleop_rpy_;   // [r p y]

      // 디버그용 Desired 저장 (위 printf 에서 같이 확인 가능)
      Desired_XYZ = Xd;
      Desired_RPY = RPY;

      // ② EE 목표 pose(Td) 구성: Base -> TCP (Ycontact 기준)
      Eigen::Vector3d rpy_tmp = RPY;
      Eigen::Matrix3d Rd;
      AKin.EulerAngle2Rotation(Rd, rpy_tmp);

      RArm.Td <<
          Rd(0,0), Rd(0,1), Rd(0,2), Xd(0),
          Rd(1,0), Rd(1,1), Rd(1,2), Xd(1),
          Rd(2,0), Rd(2,1), Rd(2,2), Xd(2),
          0,       0,       0,       1;

      // ③ 항상 Base->TCP (Ycontact) 기준 IK
#if TCP_standard == 0
      AKin.InverseK_min(&RArm);
#else
      AKin.Ycontact_InverseK_min(&RArm);
#endif

      // ④ qd 를 joint command 로 퍼블리시
      joint_state_.header.stamp = node_->now();
      for (int i = 0; i < DOF; ++i) {
        joint_state_.position[i] = RArm.qd(i);
      }
      joint_commands_pub_->publish(joint_state_);
    } else {
      // pose 아직 안 들어왔으면 현재 qc 유지
      joint_state_.header.stamp = node_->now();
      for (int i = 0; i < DOF; ++i) {
        joint_state_.position[i] = RArm.qc(i);
      }
      joint_commands_pub_->publish(joint_state_);
    }

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }



  // 6) Teleop w/ Force Control mode: /calibrated_pose + /ftsensor → 공통 force chain
  if (control_mode == 6) {
    if (teleop_pose_valid_) {
      Eigen::Vector3d Xd  = teleop_xyz_;   // TCP 기준
      Eigen::Vector3d RPY = teleop_rpy_;
      Eigen::Vector3d Fd  = teleop_force_valid_ ?
                            teleop_force_ : Eigen::Vector3d::Zero();

      std_msgs::msg::Float64MultiArray dbg;
      dbg.data.resize(9);
      dbg.data[0] = Xd(0);
      dbg.data[1] = Xd(1);
      dbg.data[2] = Xd(2);
      dbg.data[3] = RPY(0);
      dbg.data[4] = RPY(1);
      dbg.data[5] = RPY(2);
      dbg.data[6] = Fd(0);
      dbg.data[7] = Fd(1);
      dbg.data[8] = Fd(2);
      debug_step1_pub_->publish(dbg);

      // runCartesianForceChain 은 Xd/Act_XYZ 모두 TCP 기준으로 가정
      runCartesianForceChain(Xd, RPY, Fd, dt_s);  // func_ur10e_main.cpp
    } else {
      joint_state_.header.stamp = node_->now();
      for (int i = 0; i < DOF; ++i)
        joint_state_.position[i] = RArm.qc(i);
      joint_commands_pub_->publish(joint_state_);
    }

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 7) ACT Policy Inference mode
  //    - ACT inference node: /action_joints (JointState), /action_force (Wrench)
  //    - joints 6개 → FK로 Xd, RPY 계산
  //    - force 3개 → Fd = [fx, fy, fz]
  //    - runCartesianForceChain(Xd, RPY, Fd, dt_s) 호출 후 qd 퍼블리시
  if (control_mode == 7) {
    // joints 값이 아직 안 들어왔으면 현재 자세 유지
    if (!act_joints_valid_) {
      joint_state_.header.stamp = node_->now();
      for (int i = 0; i < DOF; ++i)
        joint_state_.position[i] = RArm.qc(i);
      joint_commands_pub_->publish(joint_state_);

      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
    }

    // --------- 1) ACT joints → FK로 Xd, RPY 계산 ---------
    // RArm 구조를 건드리지 않기 위해 백업 후 복원 방식 사용
    Vector6d qc_backup = RArm.qc;
    auto     Tc_backup = RArm.Tc;
    auto     xc_backup = RArm.xc;
    auto     thc_backup = RArm.thc;

    // ACT가 준 joints를 "가상의 qc"로 넣어서 FK
    for (int i = 0; i < DOF; ++i) {
      RArm.qc(i) = act_joints_(i);
    }

    AKin.Ycontact_ForwardK_T(&RArm);
    AKin.Rotation2EulerAngle(&RArm);

    Eigen::Vector3d Xd  = RArm.xc;   // ACT 기준 desired TCP position
    Eigen::Vector3d RPY = RArm.thc;  // ACT 기준 desired RPY

    // RArm을 다시 실제 qc 기준 상태로 복원 (실제 Act_XYZ는 위쪽에서 이미 계산됨)
    RArm.qc  = qc_backup;
    RArm.Tc  = Tc_backup;
    RArm.xc  = xc_backup;
    RArm.thc = thc_backup;

    // --------- 2) force: ACT가 준 force 3개 사용 (없으면 0) ---------
    Eigen::Vector3d Fd = act_force_valid_ ? act_force_
                                          : Eigen::Vector3d::Zero();

    // 디버그용 Desired 값 갱신
    Desired_XYZ = Xd;
    Desired_RPY = RPY;

    // --------- 3) 공통 어드미턴스 체인 호출 ---------
    runCartesianForceChain(Xd, RPY, Fd, dt_s);

    // runCartesianForceChain 내부에서 RArm.qd 갱신된다고 가정
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i) {
      joint_state_.position[i] = RArm.qd(i);
    }
    joint_commands_pub_->publish(joint_state_);

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }


  // 예외: 보호
  speedmode = 0;
  RArm.qd = RArm.qc;
  RArm.qt = RArm.qc;
  RArm.dqc << 0,0,0,0,0,0;
  pause_cnt = 0;

  pre_ctrl.store(control_mode, std::memory_order_relaxed);
}
