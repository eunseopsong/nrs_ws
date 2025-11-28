// JointControl.cpp (v2025-rt-safe)
// - 기존 기능을 유지하면서 실시간성/일관성/안정성 개선
// - 주요 수정점: dt 측정, 조인트 소스 단일화, 힘 변환 간소화, 퍼블리시 최적화
// - 2025-11-06: control_mode 1/2 공통 force/admittance/IK 체인 함수(runCartesianForceChain)로 통합
//                /ftsensor/measured_CValue 구독 추가 → control_mode 2 에서 fx fy fz 사용
//                /calibrated_pose 콜백은 pose만 저장, 실제 계산은 메인루프에서 수행
// push from LAB 2025-11-04 (refactored 2025-11-06)

#include "JointControl.h"
#include "func_ur10e_main.h"   // set_status, yaml_get_path, readDoublesFromStdin 등 유틸 선언

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <filesystem>
#include <iostream>   // 터미널 입력용
#include <limits>     // 터미널 입력 오류 처리용

#include <geometry_msgs/msg/wrench.hpp>

constexpr int DOF = 6;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// ===================== 유틸 & 전역 =====================
static std::mutex g_cmdmode_mtx;   // 모드 변경 보호용

// ================================================================================
// ===================== JointControl 생성/소멸자 =====================
// ================================================================================
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
  UR10e_mode_pub_     = node_->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 20);
  UR10_pose_pub_      = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_pose", 20);
  UR10_wrench_pub_    = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_wrench", 20);
  joint_commands_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands" , 20);

  // JointState 메시지 초기화(고정 필드 사전 세팅)
  joint_state_.name = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
  joint_state_.position.resize(DOF, 0.0);

  // Subscribers
  UR10e_mode_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
    "Yoon_UR10e_mode", 20, std::bind(&JointControl::cmdModeCallback, this, std::placeholders::_1));

  PB_iter_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
    "/Yoon_PbNum_cmd", 100, std::bind(&JointControl::PbIterCallback, this, std::placeholders::_1));

  joint_cmd_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/yoon_UR10e_joint_cmd", 100, std::bind(&JointControl::JointCmdCallback, this, std::placeholders::_1));

  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/isaac_joint_states", rclcpp::QoS(10), std::bind(&JointControl::getActualQ, this, std::placeholders::_1));

  ft_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/contact/force_magnitude", rclcpp::SensorDataQoS(),
    std::bind(&JointControl::FtCallback, this, std::placeholders::_1)
  );

  // /calibrated_pose 에서 [x,y,z,r,p,yaw] 받기
  calibrated_pose_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/calibrated_pose",
    10,
    std::bind(&JointControl::calibratedPoseCallback, this, std::placeholders::_1)
  );

  // VR 트래커에 달린 FT sensor 값 받기
  ftsensor_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
      "/ftsensor/measured_Cvalue",
      10,
      std::bind(&JointControl::ftSensorCallback, this, std::placeholders::_1)
  );

  // Timer (2ms). 메인루프에서 실제 dt는 steady_clock으로 산출.
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

// ================================================================================
// ===================== Callbacks =====================
// ================================================================================

// 모드 명령 콜백
void JointControl::cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(g_cmdmode_mtx);
  try {
    mode_cmd = msg->data;
    printf("[DEBUG] cmdModeCallback called. mode_cmd=%u\n", mode_cmd);

    if (mode_cmd == FK_control_mode_cmd) {
      ctrl.store(1, std::memory_order_release);
    }
    else if (mode_cmd == IK_control_mode_cmd) {
      ctrl.store(2, std::memory_order_release);
    }
    // ===================== Playback Mode for Path(.txt) Execution ===================== //
    else if (mode_cmd == Playback_mode_cmd) {
      auto hand_path = yaml_get_path(NRS_recording, "hand_g_recording", node_->get_logger());
      if (hand_path.empty() || !std::filesystem::exists(hand_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory file not found: '%s'", hand_path.c_str());
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, Motion_stop_mode);
        return;
      }
      if (Hand_G_playback) { std::fclose(Hand_G_playback); Hand_G_playback = nullptr; }
      Hand_G_playback = std::fopen(hand_path.c_str(), "rt");
      if (!Hand_G_playback) {
        RCLCPP_ERROR(node_->get_logger(), "open for read failed: '%s' (%s)", hand_path.c_str(), std::strerror(errno));
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, Motion_stop_mode);
        return;
      }
      set_status(message_status, ST_path_gen_done);
      ctrl.store(3, std::memory_order_release);
      pre_ctrl.store(0, std::memory_order_relaxed); // 다음 사이클에서 init 감지되도록
    }
    // ================= Continuous Recording Mode for Teleoperation using VR tracker ================= //
    else if (mode_cmd == Continuous_reording_start) {
      ctrl.store(4, std::memory_order_release);
      set_status(message_status, Data_recording_on);
    }
    else if (mode_cmd == Continusous_recording_end) {
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, Data_recording_off);
    }
    // ===================== Motion Stop ===================== //
    else if (mode_cmd == Motion_stop_cmd) {
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, Motion_stop_mode);
      if (Hand_G_playback)     { std::fclose(Hand_G_playback);     Hand_G_playback     = nullptr; }
      if (hand_g_recording)    { std::fclose(hand_g_recording);    hand_g_recording    = nullptr; }
      if (path_recording_pos)  { std::fclose(path_recording_pos);  path_recording_pos  = nullptr; }
    }

  } catch (const std::exception& e) {
    RCLCPP_FATAL(node_->get_logger(), "cmdModeCallback exception: %s", e.what());
  }
}

// 수동 재생 인덱스 콜백
void JointControl::PbIterCallback(std_msgs::msg::UInt16::SharedPtr msg) {
  PB_iter_cmd = msg->data;
  PB_iter_cur = 1; // 1 is right
}

// 단일 조인트 증분 경로 생성 콜백
void JointControl::JointCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  mjoint_cmd = msg->data;
  printf("\nSelected joint: %1.0f, Target relative joint angle: %4f \n", mjoint_cmd[0],mjoint_cmd[1]);

  double Tar_pos[] = { std::fabs(mjoint_cmd[1]) };
  double Tar_vel[] = { (mjoint_cmd[1]>=0) ? 0.1 : -0.1 };
  double Waiting_time[] = {0,0}; // s

  Joint_path_start <<RArm.qc(0),RArm.qc(1),RArm.qc(2),RArm.qc(3),RArm.qc(4),RArm.qc(5);
  Path_point_num = J_single.Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));

  if(Path_point_num != -1) {
    ctrl.store(1, std::memory_order_release);
    set_status(message_status, path_gen_done);
    path_done_flag = true;
  }
}

// 실제 조인트 상태 콜백
void JointControl::getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (int i = 0; i < DOF && i < (int)msg->position.size(); ++i){
    RArm.qc[i] = msg->position[i];
  }
}

// TCP z축 힘 콜백
void JointControl::FtCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    contact_force = msg->data;

    // TCP 기준 힘(스칼라만 올 때의 가정: [0,0,Fz])
    const Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);

    // 가장 최근 FK(메인루프/여기서 계산된)의 회전 사용
    const Eigen::Matrix3d R_TCP_base = RArm.Tc.block<3,3>(0,0);
    const Eigen::Vector3d F_base = R_TCP_base * F_TCP;

    (void)F_base;  // 여기서는 저장만, 실제 변환/LPF 등은 runCartesianForceChain 쪽에서 처리
}

// 텔레옵 pose 콜백
void JointControl::calibratedPoseCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 6) return;

    teleop_xyz_ <<
        msg->data[0],
        msg->data[1],
        msg->data[2];

    teleop_rpy_ <<
        wrapToPi(msg->data[3]),
        wrapToPi(msg->data[4]),
        wrapToPi(msg->data[5]);

    teleop_pose_valid_ = true;
}

// 텔레옵 FT 콜백
void JointControl::ftSensorCallback(
    const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    teleop_force_ <<
        msg->force.x,
        msg->force.y,
        msg->force.z;
    teleop_force_valid_ = true;
}

// ================================================================================
// ===================== Main Control Loop =====================
// ================================================================================
//
// 역할: 주기(dt) 산출 → 최신 FK/상태 갱신 → 모드별 제어/퍼블리시
//
void JointControl::CalculateAndPublishJoint() {
  // dt 측정(타이머 지터/일시정지 대비)
  static auto t_prev = std::chrono::steady_clock::now();
  const auto t_now = std::chrono::steady_clock::now();
  double dt_s = std::chrono::duration<double>(t_now - t_prev).count();
  t_prev = t_now;
  if (dt_s <= 0.0 || dt_s > 0.2) dt_s = 0.01;  // 안전망 (10ms)

  // milisec += dt_s * 1000.0;
  milisec += 1; // Simulation time in ms

  // 가속/속도/증분 초기화
  for (int i = 0; i < DOF; i++) {
    RArm.ddqd(i) = 0;
    RArm.dqd(i)  = 0;
    RArm.dqc(i)  = 0;
  }
  RArm.qd = RArm.qc;
  RArm.qt = RArm.qc;

  // 최신 FK & RPY (EE/TCP 기준)
#if TCP_standard == 0
  AKin.ForwardK_T(&RArm);
#else
  AKin.Ycontact_ForwardK_T(&RArm);
#endif
  AKin.Rotation2EulerAngle(&RArm); // Tc -> thc
  RArm.Td = RArm.Tc;

  // 클래스 보조 상태 동기화(가시화용)
  for (int i = 0; i < DOF; ++i) joint_pos[i] = RArm.qc(i);

  // TCP→Base 힘 변환 (for debug print)
  Eigen::Vector3d F_base_dbg = Eigen::Vector3d::Zero();
  {
      const Eigen::Matrix3d R_TCP_base = RArm.Tc.block<3,3>(0,0);  // Base<-TCP 회전
      const Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);        // TCP z축 힘
      Eigen::Vector3d F_base_raw = R_TCP_base * F_TCP;             // 변환된 힘 (Base 기준)

      // 제어 쪽에서 외력은 F_ext = -F_base_raw 를 쓰고 있으므로
      F_base_dbg = -F_base_raw;
  }

  // 상태 로드
  const int control_mode     = ctrl.load(std::memory_order_relaxed);
  const int pre_control_mode = pre_ctrl.load(std::memory_order_relaxed);

  // ====== 디버그 출력(주기 제한) ======
  if (printer_counter >= print_period) {
  #if RT_printing
      Eigen::Vector3d tcp_act = RArm.xc;

      printf("======================================== \n");
      printf("Simulation time : %d ms\n", (int)milisec);
      printf("RUN MODE %d (prev %d)\n", control_mode, pre_control_mode);

      printf("q  : %.6f %.6f %.6f %.6f %.6f %.6f\n",
            RArm.qc(0), RArm.qc(1), RArm.qc(2), RArm.qc(3), RArm.qc(4), RArm.qc(5));
      printf("qd : %.6f %.6f %.6f %.6f %.6f %.6f\n",
            RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5));

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

  // ====== 토픽 퍼블리시(비차단/재사용 버퍼) ======
  UR10_pose_msg_.data.resize(6);
  UR10_wrench_msg_.data.resize(6);

  // Pose는 EE/TCP 기준으로 퍼블리시
  Eigen::Vector3d tcp_act = RArm.xc;
  for(int i=0; i<6; i++){
    if (i < 3) {
      UR10_pose_msg_.data[i] = tcp_act(i);
    } else {
      UR10_pose_msg_.data[i] = RArm.thc(i-3);
    }
    UR10_wrench_msg_.data[i] = ftS2(i);
  }
  UR10_pose_pub_->publish(UR10_pose_msg_);
  UR10_wrench_pub_->publish(UR10_wrench_msg_);

  // ====== 모드 처리 ======
  // 0) 홈자세 고정
  if (control_mode == 0) {
      speedmode = 0;
      static const double HOME_Q[6] = { 0.0, -M_PI/2.0, -M_PI/2.0, -M_PI/2.0, +M_PI/2.0, 0.0 };

      for (int i = 0; i < DOF; ++i) {
          RArm.qd(i)   = HOME_Q[i];
          joint_pos[i] = HOME_Q[i];
      }

      RArm.qt = RArm.qd;
      RArm.dqc << 0, 0, 0, 0, 0, 0;
      pause_cnt = 0;

      joint_state_.header.stamp = node_->now();
      for (int i = 0; i < DOF; ++i) joint_state_.position[i] = RArm.qd(i);
      joint_commands_pub_->publish(joint_state_);

      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
  }

  // 1) FK Control Mode (Joint Control)
  if (control_mode == 1) {
      // FK: 사용자가 터미널에 입력한 조인트(deg)를 그대로 명령
      static bool     fk_target_set = false;
      static Vector6d fk_target_q   = Vector6d::Zero();

      // 모드 진입 시 한 번만 입력
      if (pre_control_mode != control_mode || !fk_target_set) {
          double qdeg[6];
          if (!readDoublesFromStdin(
                  "\n[FK mode] Enter 6 joint angles [deg]: ",
                  6, qdeg)) {
              std::cerr << "[FK mode] invalid input. Keep current pose.\n";
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          }

          for (int i = 0; i < DOF; ++i) {
              fk_target_q(i) = qdeg[i] * M_PI / 180.0;  // deg -> rad
          }
          fk_target_set = true;
      }

      for (int i = 0; i < DOF; ++i) {
          RArm.qd(i) = fk_target_q(i);
      }

      joint_state_.header.stamp = node_->now();
      for (int i = 0; i < DOF; ++i) {
          joint_state_.position[i] = RArm.qd(i);
      }
      joint_commands_pub_->publish(joint_state_);

      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
  }

  // 2) IK Control Mode (EE Position Control)
  if (control_mode == 2) {
      // IK: 사용자가 터미널에 입력한 EE pose [x y z r p y] (m, rad)을 IK로 변환
      static bool            ik_target_set  = false;
      static Eigen::Vector3d ik_target_xyz  = Eigen::Vector3d::Zero();
      static Eigen::Vector3d ik_target_rpy  = Eigen::Vector3d::Zero();

      if (pre_control_mode != control_mode || !ik_target_set) {
          double buf[6];
          if (!readDoublesFromStdin(
                  "\n[IK mode] Enter EE pose [x y z r p y] (m, rad): ",
                  6, buf)) {
              std::cerr << "[IK mode] invalid input. Keep current pose.\n";
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          }

          ik_target_xyz << buf[0], buf[1], buf[2];  // EE/TCP 기준 목표
          ik_target_rpy << buf[3], buf[4], buf[5];

          ik_target_set = true;
      }

      // 디버그용 Desired 값 업데이트 (EE/TCP 기준)
      Desired_XYZ = ik_target_xyz;
      Desired_RPY = ik_target_rpy;

      Eigen::Vector3d rpy_copy = ik_target_rpy; // EulerAngle2Rotation이 non-const ref 요구
      Eigen::Matrix3d Rd;
      AKin.EulerAngle2Rotation(Rd, rpy_copy);

      RArm.Td <<
          Rd(0,0), Rd(0,1), Rd(0,2), ik_target_xyz(0),
          Rd(1,0), Rd(1,1), Rd(1,2), ik_target_xyz(1),
          Rd(2,0), Rd(2,1), Rd(2,2), ik_target_xyz(2),
          0,       0,       0,       1;

#if TCP_standard == 0
      AKin.InverseK_min(&RArm);
#else
      AKin.Ycontact_InverseK_min(&RArm);
#endif

      joint_state_.header.stamp = node_->now();
      for (int i = 0; i < DOF; ++i) {
          joint_state_.position[i] = RArm.qd(i);
      }
      joint_commands_pub_->publish(joint_state_);

      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
  }

  // 3) Playback: InitMove → PathFollow → ReturnHomePose
  if (control_mode == 3) {
      static bool init_done = false;
      static bool follow_done = false;

      // 모드가 변경될 때만 init/follow 상태 리셋
      if (pre_control_mode != control_mode) {
          init_done   = false;
          follow_done = false;
      }

      if (!init_done) {
          bool just_finished = InitMove(dt_s);   // 구현은 func_ur10e_main.cpp
          if (!just_finished) {
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          }
          init_done = true;
      }

      if (!follow_done) {
          if (PathFollow(dt_s)) {               // 구현은 func_ur10e_main.cpp
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          } else {
              follow_done = true;
          }
      }

      // PathFollow 끝난 이후에는 ReturnHomePose를 매 주기 호출
      ReturnHomePose(dt_s);                     // 구현은 func_ur10e_main.cpp
      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
  }

  // 4) Teleop mode: /calibrated_pose + /ftsensor → 공통 force chain
  if (control_mode == 4) {
      if (teleop_pose_valid_) {
          Eigen::Vector3d Xd  = teleop_xyz_;   // EE/TCP 기준
          Eigen::Vector3d RPY = teleop_rpy_;
          Eigen::Vector3d Fd  = teleop_force_valid_ ? teleop_force_
                                                    : Eigen::Vector3d::Zero();

          {
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
          }

          runCartesianForceChain(Xd, RPY, Fd, dt_s);  // 구현은 func_ur10e_main.cpp
      } else {
          joint_state_.header.stamp = node_->now();
          for (int i = 0; i < DOF; ++i) {
              joint_state_.position[i] = RArm.qc(i);
          }
          joint_commands_pub_->publish(joint_state_);
      }

      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
  }

  // 그 외(보호)
  speedmode = 0;
  RArm.qd = RArm.qc;
  RArm.qt = RArm.qc;
  RArm.dqc << 0,0,0,0,0,0;
  pause_cnt=0;

  pre_ctrl.store(control_mode, std::memory_order_relaxed);
}
