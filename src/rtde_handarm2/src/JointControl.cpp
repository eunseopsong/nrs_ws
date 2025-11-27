// JointControl.cpp (v2025-rt-safe)
// - 기존 기능을 유지하면서 실시간성/일관성/안정성 개선
// - 주요 수정점: dt 측정, TOOL_Z 일관화, 조인트 소스 단일화, 힘 변환 간소화, 퍼블리시 최적화
// - 2025-11-06: control_mode 1/2 공통 force/admittance/IK 체인 함수(runCartesianForceChain)로 통합
//                /ftsensor/measured_CValue 구독 추가 → control_mode 2 에서 fx fy fz 사용
//                /calibrated_pose 콜백은 pose만 저장, 실제 계산은 메인루프에서 수행
// push from LAB 2025-11-04 (refactored 2025-11-06)

#include "JointControl.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <filesystem>
#include <iostream>   // ★ 추가: 터미널 입력용
#include <limits>     // ★ 추가: 터미널 입력 오류 처리용

#include <geometry_msgs/msg/wrench.hpp>

constexpr int DOF = 6;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// ================================================================================
// ===================== 외부에서 선언·정의된 타입 및 인스턴스 가정 =====================
// ================================================================================
//
// [1] 로봇 및 운동학 관련 객체
//  - AKin : Kinematics helper (ForwardK_T, Ycontact_ForwardK_T, InverseK_min, Rotation2EulerAngle, …)
//  - RArm : Robot arm state holder
//      qc : 현재 조인트(rad)
//      qd : 목표 조인트(rad)
//      xc : 현재 EE 위치 (x,y,z) [m]
//      thc: 현재 EE RPY (r,p,y) [rad]
//      Tc/Td : Homogeneous Transform (현재/목표)
//
// [2] Trajectory/Playback
//  - Hand_G_playback : TXT 파일 핸들 (x y z r p y fx fy fz)
//  - path_exe_counter/Path_point_num 등 실행 인덱스
//
// [3] 제어/상태 관련 글로벌들
//  - ctrl, pre_ctrl (std::atomic<int>)
//  - mode_cmd, path_done_flag, pause_cnt, speedmode, printer_counter, print_period 등
//  - 메시지 문자열: message_status, set_status()
//  - 파일 핸들: hand_g_recording, Discre_P_recording, VRCali_UR10CB_EE, VRCali_UR10CB_VR, path_recording_pos, path_recording_joint, EXPdata1
//
// [4] 상수/모드 상수 (Yoon_UR10e_cmd.h)
//  - Hand_guiding_mode_cmd, Motion_stop_cmd, Playback_mode_cmd, Joint_control_mode_cmd,
//    EE_Posture_control_mode_cmd, Continuous_reording_start, Continusous_recording_end, …
//
// ================================================================================

// ===================== 유틸 & 전역 =====================
static std::mutex g_cmdmode_mtx;

template <size_t N>
static inline void set_status(char (&dst)[N], const char* s) {
  std::snprintf(dst, N, "%s", s ? s : "");
}

static std::string trim_path(std::string s) {
  auto notspace = [](unsigned char c){ return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
  while (!s.empty() && (s.back()=='\r' || s.back()=='\n' || std::isspace((unsigned char)s.back()))) {
    s.pop_back();
  }
  return s;
}

// YAML에서 파일 경로 안전 취득
static std::string yaml_get_path(const YAML::Node& root, const char* key, const rclcpp::Logger& logger) {
  try {
    if (!root || !root.IsMap()) {
      RCLCPP_ERROR(logger, "NRS_recording is not a map (key='%s')", key);
      return "";
    }
    YAML::Node n = root[key];
    if (!n || !n.IsDefined()) {
      RCLCPP_ERROR(logger, "YAML key '%s' is missing/undefined.", key);
      return "";
    }
    if (n.IsScalar()) {
      return trim_path(n.as<std::string>());
    }
    if (n.IsSequence()) {
      std::filesystem::path p;
      for (std::size_t i = 0; i < n.size(); ++i) {
        if (!n[i].IsScalar()) {
          RCLCPP_ERROR(logger, "YAML key '%s' has non-scalar element in sequence.", key);
          return "";
        }
        p /= n[i].as<std::string>();
      }
      return trim_path(p.string());
    }
    RCLCPP_ERROR(logger, "YAML key '%s' must be a scalar or sequence.", key);
    return "";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "YAML get path error for key '%s': %s", key, e.what());
    return "";
  }
}

// 파일 부모 폴더 보장
static void ensure_parent_dir(const std::string& filepath, const rclcpp::Logger& logger) {
  if (filepath.empty()) return;
  std::error_code ec;
  auto parent = std::filesystem::path(filepath).parent_path();
  if (!parent.empty() && !std::filesystem::exists(parent)) {
    if (!std::filesystem::create_directories(parent, ec)) {
      if (ec) {
        RCLCPP_WARN(logger, "Failed to create parent dir '%s': %s",
                    parent.string().c_str(), ec.message().c_str());
      }
    }
  }
}

// EE +Z → TCP 오프셋(모든 FK/IK에서 동일 사용)
static constexpr double TOOL_Z = 0.325;  // [m]
// static constexpr double TOOL_Z = 0.343;  // [m]

// ★ 플랜지 <-> TCP 변환 헬퍼 (z방향 오프셋만)
static inline Eigen::Vector3d flangeToTcp(const Eigen::Vector3d& flange_xyz) {
  Eigen::Vector3d tcp = flange_xyz;
  tcp(2) -= TOOL_Z;
  return tcp;
}

static inline Eigen::Vector3d tcpToFlange(const Eigen::Vector3d& tcp_xyz) {
  Eigen::Vector3d flange = tcp_xyz;
  flange(2) += TOOL_Z;
  return flange;
}

// ★ 추가: 터미널에서 double 배열 읽기 유틸
static bool readDoublesFromStdin(const char* prompt, int n, double* out) {
  std::cout << prompt << std::flush;
  for (int i = 0; i < n; ++i) {
    if (!(std::cin >> out[i])) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      return false;
    }
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return true;
}



// ===================== JointControl =====================
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

  // ★ 추가: VR 트래커에 달린 FT sensor 값 받기
  ftsensor_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
      "/ftsensor/measured_Cvalue",
      10,
      std::bind(&JointControl::ftSensorCallback, this, std::placeholders::_1)
  );


  // Timer (1ms). 메인루프에서 실제 dt는 steady_clock으로 산출.
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

// ===================== Mode Callback =====================
// 역할: 외부 모드 명령을 수신하여 상태/파일/레코딩을 전환하고, ctrl 원자 변수 갱신
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
      // control_mode = 2 진입
      ctrl.store(4, std::memory_order_release);
      set_status(message_status, Data_recording_on);
    }
    else if (mode_cmd == Continusous_recording_end) {
      // control_mode = 0 복귀
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

// ===================== Other Callbacks =====================
// 역할: 수동 재생 인덱스/단일 조인트 블렌딩 명령 수신
void JointControl::PbIterCallback(std_msgs::msg::UInt16::SharedPtr msg) {
  PB_iter_cmd = msg->data;
  PB_iter_cur = 1; // 1 is right
}

// 역할: 단일 조인트 증분 경로 생성 및 모드1 실행 트리거
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

// 역할: Isaac joint_state 구독 → RArm.qc 최신화(진리 소스)
void JointControl::getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (int i = 0; i < DOF && i < (int)msg->position.size(); ++i){
    RArm.qc[i] = msg->position[i];
  }
}

// ===================== UpdateState() =====================
// 역할: 현재 조인트(RArm.qc) 기반 FK 계산 → TCP Pose(pos_current, rpy_current) 업데이트
void JointControl::UpdateState()
{
    // 최신 조인트는 콜백에서 RArm.qc에 유지됨
    Eigen::Matrix<double, DOF, 1> q = RArm.qc;

    // EE +Z → TCP 오프셋을 포함한 FK
    T_current = ur10e_forward(q, TOOL_Z);

    pos_current = T_current.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_TCP = T_current.block<3, 3>(0, 0);

    // XYZ-fixed RPY
    Eigen::Vector3d rpy;
    rpy(0) = std::atan2(R_TCP(2,1), R_TCP(2,2));
    rpy(1) = std::asin(-R_TCP(2,0));
    rpy(2) = std::atan2(R_TCP(1,0), R_TCP(0,0));
    rpy_current = rpy;
}

// ===================== Force Callback =====================
// 역할: TCP축 힘(Fz)을 수신하고, 최신 FK 회전으로 Base 프레임 힘 벡터를 계산
void JointControl::FtCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    contact_force = msg->data;

    // TCP 기준 힘(스칼라만 올 때의 가정: [0,0,Fz])
    const Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);

    // 가장 최근 FK(메인루프/여기서 계산된)의 회전 사용
    const Eigen::Matrix3d R_TCP_base = RArm.Tc.block<3,3>(0,0);
    const Eigen::Vector3d F_base = R_TCP_base * F_TCP;

    (void)F_base;
}

// ===================== calibratedPose Callback =====================
// 여기서는 값만 저장하고, control_mode==2 에서 공통 체인을 타면서 IK+힘제어를 한다.
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

// ===================== ftsensor Callback =====================
void JointControl::ftSensorCallback(
    const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    teleop_force_ <<
        msg->force.x,
        msg->force.y,
        msg->force.z;
    teleop_force_valid_ = true;
}


// ===================== InitMove() =====================
// 역할: 재생 시작 시 현재 위치→TXT 첫 포즈로 시간기반 보간 이동(SLERP+선형)
bool JointControl::InitMove(double dt_s)
{
    static constexpr double LIN_VEL   = 0.20;  // m/s
    static constexpr double ANG_VEL   = 1.0;   // rad/s
    static constexpr double MIN_DUR   = 0.8;   // s
    static constexpr double MAX_DUR   = 4.0;   // s

    static bool active = false, finished = false;
    static double elapsed = 0.0, duration = 0.0;
    static Eigen::Vector3d start_xyz, goal_xyz;
    static Eigen::Matrix3d start_rot, goal_rot;
    static FILE* last_handle = nullptr;

    if (Hand_G_playback != last_handle) {
        active = false; finished = false; elapsed = 0.0; duration = 0.0;
        last_handle = Hand_G_playback;
    }

    if (!active && !finished) {
        if (!Hand_G_playback) return false;

        float x, y, z, r, p, yw, fx, fy, fz;
        char buf[2048];
        bool valid = false;
        std::rewind(Hand_G_playback);
        while (std::fgets(buf, sizeof(buf), Hand_G_playback)) {
            if (buf[0] == '#') continue;
            int n = std::sscanf(buf, "%f %f %f %f %f %f %f %f %f",
                                &x,&y,&z,&r,&p,&yw,&fx,&fy,&fz);
            if (n == 9) { valid = true; break; }
        }
        if (!valid) {
            RCLCPP_ERROR(node_->get_logger(), "[InitMove] no valid first line in TXT.");
            return false;
        }

        // 현재 실제 위치는 플랜지 기준 → TCP 기준으로 변환해서 사용
        start_xyz = flangeToTcp(RArm.xc);
        // TXT 파일에서 읽은 값은 TCP 기준으로 사용
        goal_xyz  = Eigen::Vector3d(x, y, z);

        start_rot = RArm.Tc.block<3,3>(0,0);
        Eigen::Vector3d goal_rpy(r,p,yw);
        AKin.EulerAngle2Rotation(goal_rot, goal_rpy);

        const double lin_dist = (goal_xyz - start_xyz).norm();
        const Eigen::Quaterniond q0(start_rot), q1(goal_rot);
        const double ang_dist = std::acos(std::clamp(q0.normalized().dot(q1.normalized()), -1.0, 1.0)) * 2.0;

        double t_lin = (LIN_VEL > 1e-6) ? lin_dist / LIN_VEL : 0.0;
        double t_ang = (ANG_VEL > 1e-6) ? ang_dist / ANG_VEL : 0.0;

        duration = std::max(MIN_DUR, std::min(MAX_DUR, std::max(t_lin, t_ang)));

        elapsed = 0.0;
        active = true;
        finished = false;

        printf("[InitMove] dist=%.3f ang=%.3f rad -> dur=%.2fs\n", lin_dist, ang_dist, duration);
    }

    if (!active) return finished;

    elapsed += dt_s;
    const double alpha = std::clamp(elapsed / std::max(1e-6, duration), 0.0, 1.0);

    // TCP 기준에서 보간
    const Eigen::Vector3d xyz_interp = (1.0 - alpha) * start_xyz + alpha * goal_xyz;
    Desired_XYZ = xyz_interp;   // TCP 기준 Desired

    Eigen::Quaterniond q0(start_rot), q1(goal_rot);
    if (q0.dot(q1) < 0.0) q1.coeffs() *= -1.0;
    const Eigen::Quaterniond q_interp = q0.slerp(alpha, q1).normalized();
    const Eigen::Matrix3d R_interp = q_interp.toRotationMatrix();
    Desired_RPY = R_interp.eulerAngles(0,1,2);

    // IK 변환: TCP → 플랜지 (z에 TOOL_Z 보정)
    Eigen::Vector3d flange_xyz = tcpToFlange(Desired_XYZ);

    RArm.Td << R_interp(0,0),R_interp(0,1),R_interp(0,2),flange_xyz(0),
                R_interp(1,0),R_interp(1,1),R_interp(1,2),flange_xyz(1),
                R_interp(2,0),R_interp(2,1),R_interp(2,2),flange_xyz(2),
                0,0,0,1;
#if TCP_standard == 0
    AKin.InverseK_min(&RArm);
#else
    AKin.Ycontact_InverseK_min(&RArm);
#endif

    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    if (alpha >= 1.0 - 1e-6) {
        active = false;
        finished = true;
        std::rewind(Hand_G_playback);
        printf("[InitMove] completed.\n");
    }
    return finished;
}

void JointControl::runCartesianForceChain(
    const Eigen::Vector3d& Xd,
    const Eigen::Vector3d& RPYd,
    const Eigen::Vector3d& Fd,
    double dt_s)
{
    // =========================================================================
    // STEP 2) 외력 추정 F_ext (LPF + saturation)
    //   - 여기서는 기존 PathFollow()에서 하던 것과 동일하게,
    //     현재 qc 기준 FK를 한 번 더 돌려서 TCP->Base 변환행렬을 만든 다음
    //     contact_force 를 z축 힘으로 보고 base로 변환한다.
    // =========================================================================
    Eigen::Matrix4d T_base_TCP_cur = ur10e_forward(RArm.qc, 0); // 센서가 달린 플랜지 기준
    Eigen::Matrix3d R_base_TCP     = T_base_TCP_cur.block<3,3>(0,0);
    Eigen::Matrix3d R_TCP_base     = R_base_TCP.transpose();

    // contact_force 는 TCP z 로 들어온 값이라고 가정
    Eigen::Vector3d F_TCP(0.0, 0.0, -contact_force);
    Eigen::Vector3d F_base = R_TCP_base * F_TCP;
    Eigen::Vector3d F_ext  = F_base;

    // LPF
    {
        static Eigen::Vector3d F_lp = Eigen::Vector3d::Zero();
        static bool first_f = true;

        const double fc = 15.0;                                // Hz
        const double Ts = (dt_s > 0.0 ? dt_s : 0.001);
        const double alpha = (2.0 * M_PI * fc * Ts) / (1.0 + 2.0 * M_PI * fc * Ts);

        if (first_f) {
            F_lp    = F_ext;
            first_f = false;
        } else {
            F_lp = F_lp + alpha * (F_ext - F_lp);
        }
        F_ext = F_lp;
    }

    // Saturation
    {
        const double FEXT_SAT = 30.0; // N
        for (int k = 0; k < 3; ++k) {
            if (F_ext(k) >  FEXT_SAT) F_ext(k) =  FEXT_SAT;
            if (F_ext(k) < -FEXT_SAT) F_ext(k) = -FEXT_SAT;
        }
    }

    // 현재 실제 EE 위치 (플랜지 → TCP로 변환해서 사용)
    Eigen::Vector3d X_act = flangeToTcp(RArm.xc);

    // debug step2
    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(7);
        dbg.data[0] = contact_force;
        dbg.data[1] = F_ext(0);
        dbg.data[2] = F_ext(1);
        dbg.data[3] = F_ext(2);
        dbg.data[4] = X_act(0);
        dbg.data[5] = X_act(1);
        dbg.data[6] = X_act(2);
        debug_step2_pub_->publish(dbg);
    }

    // =========================================================================
    // STEP 3) RPYd → 회전행렬 → axis-angle
    // =========================================================================
    auto rotFromRPY = [](const Eigen::Vector3d &rpy)->Eigen::Matrix3d {
        const double cr = std::cos(rpy(0));
        const double sr = std::sin(rpy(0));
        const double cp = std::cos(rpy(1));
        const double sp = std::sin(rpy(1));
        const double cy = std::cos(rpy(2));
        const double sy = std::sin(rpy(2));

        Eigen::Matrix3d Rz;
        Rz << cy,-sy,0,
              sy, cy,0,
              0 , 0 ,1;
        Eigen::Matrix3d Ry;
        Ry << cp,0,sp,
              0 ,1,0 ,
             -sp,0,cp;
        Eigen::Matrix3d Rx;
        Rx << 1,0 ,0 ,
              0,cr,-sr,
              0,sr, cr;
        return Rz * Ry * Rx;
    };

    auto rotLog = [](const Eigen::Matrix3d &R)->Eigen::Vector3d {
        double cos_theta = (R.trace() - 1.0) * 0.5;
        cos_theta = std::clamp(cos_theta, -1.0, 1.0);
        double theta = std::acos(cos_theta);
        if (theta < 1e-9) {
            return Eigen::Vector3d::Zero();
        }
        Eigen::Vector3d omega;
        omega << R(2,1) - R(1,2),
                 R(0,2) - R(2,0),
                 R(1,0) - R(0,1);
        omega *= 0.5 / std::sin(theta);
        return theta * omega;
    };

    Eigen::Matrix3d Rd_R = rotFromRPY(RPYd);
    Eigen::Vector3d Wd   = rotLog(Rd_R);

    // debug step3
    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(7);
        dbg.data[0] = RPYd(0);
        dbg.data[1] = RPYd(1);
        dbg.data[2] = RPYd(2);
        dbg.data[3] = Wd(0);
        dbg.data[4] = Wd(1);
        dbg.data[5] = Wd(2);
        dbg.data[6] = Wd.norm();
        debug_step3_pub_->publish(dbg);
    }

    // =========================================================================
    // STEP 4) 어드미턴스 + FAAC
    // =========================================================================
    static bool fc_init = false;
    static Yadmittance_control AControl[6] = {
        Yadmittance_control(0.001), Yadmittance_control(0.001), Yadmittance_control(0.001),
        Yadmittance_control(0.001), Yadmittance_control(0.001), Yadmittance_control(0.001)
    };
    static std::unique_ptr<Nrs3StepFAAC> FAAC3step[3];
    static bool   FAAC_flag[3] = {false,false,false};
    static double AC_pose_pos[3] = {0.0,0.0,0.0};
    static double AC_pose_ori[3] = {0.0,0.0,0.0};
    static double FC_MASS[6]      = {1.0,   1.0,   1.0,   0.05, 0.05, 0.05};
    static double FC_DAMPER[6]    = {6000., 6000., 6000., 10.0, 10.0, 10.0};
    static double FC_STIFFNESS[6] = {2000., 2000., 2000., 20.0, 20.0, 20.0};
    static Eigen::Vector3d Fd_cmd = Eigen::Vector3d::Zero();

    // 원하는 힘을 부드럽게 램프
    {
        const double alpha_up   = 0.02;
        const double alpha_down = 0.20;
        for (int k = 0; k < 3; ++k) {
            double alpha = (std::fabs(Fd(k)) > std::fabs(Fd_cmd(k))) ? alpha_up : alpha_down;
            Fd_cmd(k) += alpha * (Fd(k) - Fd_cmd(k));
        }
        const double FDES_SAT = 30.0;
        for (int k=0; k<3; ++k) {
            if (Fd_cmd(k) >  FDES_SAT) Fd_cmd(k) =  FDES_SAT;
            if (Fd_cmd(k) < -FDES_SAT) Fd_cmd(k) = -FDES_SAT;
        }
    }

    if (!fc_init) {
        // admittance 초기화
        for (int i = 0; i < 6; ++i) {
            AControl[i].adm_1D_MDK(
                FC_MASS[i],
                FC_DAMPER[i],
                FC_STIFFNESS[i]
            );
        }
        // FAAC 초기화
        std::vector<double> proc_noise = {0.1,0.1,0.1};
        std::vector<double> meas_noise = {10.0,10.0,10.0};
        double dt_for_faac = (dt_s > 0.0 ? dt_s : 0.001);
        for (int ax = 0; ax < 3; ++ax) {
            FAAC3step[ax] = std::make_unique<Nrs3StepFAAC>(
                FC_MASS[ax],
                FC_DAMPER[ax],
                FC_STIFFNESS[ax],
                dt_for_faac,
                proc_noise,
                meas_noise
            );
            FAAC_flag[ax] = false;
        }
        // 초기 기준
        AC_pose_pos[0] = Xd(0);
        AC_pose_pos[1] = Xd(1);
        AC_pose_pos[2] = Xd(2);
        AC_pose_ori[0] = Wd(0);
        AC_pose_ori[1] = Wd(1);
        AC_pose_ori[2] = Wd(2);
        fc_init = true;
    }

    Eigen::Vector3d Xc_cmd = Xd;
    Eigen::Vector3d Wc_cmd = Wd;
    const double Tank_energy = 5.0;

    // 바닥부 근처면 접촉으로 보고 K=0
    bool contact_on = (Xd(2) <= 10.0);

    for (int ax = 0; ax < 3; ++ax) {
        if (std::fabs(Fd_cmd(ax)) > 0.01 || FAAC_flag[ax])
            FAAC_flag[ax] = true;

        if (FAAC_flag[ax] && FAAC3step[ax]) {
            auto faac_mdk = FAAC3step[ax]->FAAC_MDKob_RUN(
                Tank_energy,
                F_ext(ax),
                Fd_cmd(ax),
                AC_pose_pos[ax],
                X_act(ax)
            );

            double used_K = contact_on ? 0.0 : faac_mdk.Stiffness;

            AControl[ax].adm_1D_MDK(
                faac_mdk.Mass,
                faac_mdk.Damping,
                used_K
            );
        }

        double next_pos = AControl[ax].adm_1D_control(
            Xd(ax),
            Fd_cmd(ax),
            F_ext(ax)
        );

        Xc_cmd(ax) = next_pos;
    }

    // 위치 안정화
    {
        static Eigen::Vector3d Xc_prev = Xd;
        const double max_offset = 0.010;
        const double max_step_each[3] = {0.001, 0.001, 0.0003};

        for (int ax=0; ax<3; ++ax) {
            double lo = Xd(ax) - max_offset;
            double hi = Xd(ax) + max_offset;
            if (Xc_cmd(ax) < lo) Xc_cmd(ax) = lo;
            if (Xc_cmd(ax) > hi) Xc_cmd(ax) = hi;

            double d = Xc_cmd(ax) - Xc_prev(ax);
            double max_step = max_step_each[ax];
            d = std::clamp(d, -max_step, max_step);
            Xc_cmd(ax) = Xc_prev(ax) + d;
        }
        Xc_prev = Xc_cmd;

        AC_pose_pos[0] = Xc_cmd(0);
        AC_pose_pos[1] = Xc_cmd(1);
        AC_pose_pos[2] = Xc_cmd(2);
    }

    AC_pose_ori[0] = Wc_cmd(0);
    AC_pose_ori[1] = Wc_cmd(1);
    AC_pose_ori[2] = Wc_cmd(2);

    // debug step4
    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(13);
        dbg.data[0]  = F_ext(0);
        dbg.data[1]  = F_ext(1);
        dbg.data[2]  = F_ext(2);
        dbg.data[3]  = Fd_cmd(0);
        dbg.data[4]  = Fd_cmd(1);
        dbg.data[5]  = Fd_cmd(2);
        dbg.data[6]  = X_act(0);
        dbg.data[7]  = X_act(1);
        dbg.data[8]  = X_act(2);
        dbg.data[9]  = Xc_cmd(0);
        dbg.data[10] = Xc_cmd(1);
        dbg.data[11] = Xc_cmd(2);
        dbg.data[12] = static_cast<double>(contact_on);
        debug_step4_pub_->publish(dbg);
    }

    // =========================================================================
    // STEP 5) IK용 pose 만들기
    //   Kinematics.h 의 EulerAngle2Rotation 이 비-const 참조를 요구하므로
    //   RPYd 복사본을 만든다.
    // =========================================================================
    // TCP 명령 Xc_cmd → 플랜지 위치로 변환
    Eigen::Vector3d flange_xyz = tcpToFlange(Xc_cmd);

    Eigen::Vector3d rpy_copy = RPYd;   // <- 비-const 참조 요구 때문에 복사
    Eigen::Matrix3d Rd_R_again;
    AKin.EulerAngle2Rotation(Rd_R_again, rpy_copy);

    RArm.Td <<
        Rd_R_again(0,0), Rd_R_again(0,1), Rd_R_again(0,2), flange_xyz(0),
        Rd_R_again(1,0), Rd_R_again(1,1), Rd_R_again(1,2), flange_xyz(1),
        Rd_R_again(2,0), Rd_R_again(2,1), Rd_R_again(2,2), flange_xyz(2),
        0,               0,               0,               1;

#if TCP_standard == 0
    AKin.InverseK_min(&RArm);
#else
    AKin.Ycontact_InverseK_min(&RArm);
#endif

    // =========================================================================
    // STEP 6) 조인트 publish
    // =========================================================================
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < 6; ++i) {
        joint_state_.position[i] = RArm.qd(i);
    }
    joint_commands_pub_->publish(joint_state_);

    // =========================================================================
    // STEP 7) 외력 publish
    // =========================================================================
    {
        std_msgs::msg::Float64MultiArray force_msg;
        force_msg.data.resize(3);
        force_msg.data[0] = F_ext(0);
        force_msg.data[1] = F_ext(1);
        force_msg.data[2] = F_ext(2);
        force_ext_base_pub_->publish(force_msg);
    }
}


// ============================================================================
// PathFollow – TXT → (Xd,RPYd,Fd)만 만들고 공통 체인 호출
// ============================================================================
bool JointControl::PathFollow(double dt_s)
{
    static bool  active      = true;
    static FILE* last_handle = nullptr;
    if (Hand_G_playback != last_handle) {
        active      = true;
        last_handle = Hand_G_playback;
    }
    if (!active) {
        return false;
    }

    if (!Hand_G_playback) {
        RCLCPP_ERROR(node_->get_logger(), "[PB] playback file closed unexpectedly.");
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, "Playback file closed");
        return false;
    }

    float des_x, des_y, des_z;
    float des_r, des_p, des_yaw;
    float des_fx, des_fy, des_fz;

    int reti = std::fscanf(
        Hand_G_playback,
        "%f %f %f %f %f %f %f %f %f",
        &des_x, &des_y, &des_z,
        &des_r, &des_p, &des_yaw,
        &des_fx, &des_fy, &des_fz
    );

    if (reti != 9) {
        std::fclose(Hand_G_playback);
        Hand_G_playback = nullptr;
        active = false;

        // Return-to-home 초기화
        return_active_   = true;
        return_elapsed_  = 0.0;
        return_duration_ = 4.0;
        for (int i = 0; i < DOF; ++i) {
            return_start_q_(i) = RArm.qc(i);
        }

        printf("[PB] End of file. Start return-to-home.\n");
        return false;
    }

    Eigen::Vector3d Xd(des_x, des_y, des_z);
    Eigen::Vector3d RPYd(des_r, des_p, des_yaw);
    Eigen::Vector3d Fd(des_fx, des_fy, des_fz);

    Desired_XYZ = Xd;
    Desired_RPY = RPYd;

    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(9);
        dbg.data[0] = Xd(0);
        dbg.data[1] = Xd(1);
        dbg.data[2] = Xd(2);
        dbg.data[3] = RPYd(0);
        dbg.data[4] = RPYd(1);
        dbg.data[5] = RPYd(2);
        dbg.data[6] = Fd(0);
        dbg.data[7] = Fd(1);
        dbg.data[8] = Fd(2);
        debug_step1_pub_->publish(dbg);
    }

    runCartesianForceChain(Xd, RPYd, Fd, dt_s);
    return true;
}

bool JointControl::ReturnHomePose(double dt_s)
{
    static bool     active    = false;
    static double   elapsed   = 0.0;
    static double   duration  = 10.0;
    static Vector6d start_q;

    static const Vector6d HOME_Q = (Vector6d() <<
        0.0,
        -M_PI / 2.0,
        -M_PI / 2.0,
        -M_PI / 2.0,
        +M_PI / 2.0,
        0.0).finished();

    if (return_active_ && !active) {
        active    = true;
        elapsed   = 0.0;
        duration  = 10.0;
        start_q   = return_start_q_;
    }

    if (active) {
        double dt_step = dt_s;
        if (dt_step <= 0.0 || dt_step > 0.05) {
            dt_step = 0.001;
        }

        elapsed += dt_step;

        double s_raw = elapsed / std::max(1e-6, duration);
        if (s_raw < 0.0) s_raw = 0.0;
        if (s_raw > 1.0) s_raw = 1.0;

        double s = (3.0 * s_raw * s_raw) - (2.0 * s_raw * s_raw * s_raw);

        Vector6d q_cmd;
        for (int i = 0; i < DOF; ++i) {
            q_cmd(i) = (1.0 - s) * start_q(i) + s * HOME_Q(i);
        }

        for (int i = 0; i < DOF; ++i) {
            RArm.qd(i) = q_cmd(i);
        }

        joint_state_.header.stamp = node_->now();
        for (int i = 0; i < DOF; ++i) {
            joint_state_.position[i] = RArm.qd(i);
        }
        joint_commands_pub_->publish(joint_state_);

        if (s_raw >= 1.0 - 1e-6) {
            active          = false;
            return_active_  = false;

            printf("[PB] Return-to-home done (10s smooth ramp).\n");

            ctrl.store(0, std::memory_order_release);
            set_status(message_status, "Playback finished");
        }

        return true;
    }

    return false;
}


// ===================== Main Control Loop =====================
// 역할: 주기(dt) 산출 → 최신 FK/상태 갱신 → 모드별 제어/퍼블리시
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

  // 최신 FK & RPY (플랜지 기준)
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
      const Eigen::Matrix3d R_TCP_base = RArm.Tc.block<3,3>(0,0);  // Base<-TCP 회전 (플랜지와 동일 방향)
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
      // 플랜지 → TCP 변환 후 출력
      Eigen::Vector3d tcp_act = flangeToTcp(RArm.xc);

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

  // Pose는 TCP 기준으로 퍼블리시
  Eigen::Vector3d tcp_act = flangeToTcp(RArm.xc);
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

          ik_target_xyz << buf[0], buf[1], buf[2];  // TCP 기준 목표
          ik_target_rpy << buf[3], buf[4], buf[5];

          ik_target_set = true;
      }

      // 디버그용 Desired 값 업데이트 (TCP 기준)
      Desired_XYZ = ik_target_xyz;
      Desired_RPY = ik_target_rpy;

      // TCP 기준 목표 → 플랜지 기준 목표 (z축으로 TOOL_Z 보정)
      Eigen::Vector3d flange_xyz = tcpToFlange(ik_target_xyz);

      Eigen::Vector3d rpy_copy = ik_target_rpy; // EulerAngle2Rotation이 non-const ref 요구
      Eigen::Matrix3d Rd;
      AKin.EulerAngle2Rotation(Rd, rpy_copy);

      RArm.Td <<
          Rd(0,0), Rd(0,1), Rd(0,2), flange_xyz(0),
          Rd(1,0), Rd(1,1), Rd(1,2), flange_xyz(1),
          Rd(2,0), Rd(2,1), Rd(2,2), flange_xyz(2),
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

      // ★ 수정: 모드가 변경될 때만 init/follow 상태 리셋
      if (pre_control_mode != control_mode) {
          init_done   = false;
          follow_done = false;
      }

      if (!init_done) {
          bool just_finished = InitMove(dt_s);
          if (!just_finished) {
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          }
          init_done = true;
      }

      if (!follow_done) {
          if (PathFollow(dt_s)) {
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          } else {
              follow_done = true;
          }
      }

      // PathFollow 끝난 이후에는 ReturnHomePose를 매 주기 호출
      ReturnHomePose(dt_s);
      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
  }

  // 4) Teleop mode: /calibrated_pose + /ftsensor → 공통 force chain
  if (control_mode == 4) {
      if (teleop_pose_valid_) {
          Eigen::Vector3d Xd  = teleop_xyz_;   // TCP 기준
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

          runCartesianForceChain(Xd, RPY, Fd, dt_s);
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
