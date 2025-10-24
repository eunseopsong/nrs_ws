// JointControl.cpp (v2025-rt-safe)
// - 기존 기능을 유지하면서 실시간성/일관성/안정성 개선
// - 주요 수정점: dt 측정, TOOL_Z 일관화, 조인트 소스 단일화, 힘 변환 간소화, 퍼블리시 최적화

#include "JointControl.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <filesystem>

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
//// ================================================================================

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
static constexpr double TOOL_Z = 0.248;  // [m]

// ===================== JointControl =====================
JointControl::JointControl(const rclcpp::Node::SharedPtr& node)
: node_(node), milisec(0.0)
{
  // 모니터링 객체
  AdaptiveK_msg_ = std::make_unique<nrs_msgmonitoring2::MsgMonitoring>(node_, "AdaptiveK_msg");
  FAAC3step_msg_ = std::make_unique<nrs_msgmonitoring2::MsgMonitoring>(node_, "FAAC3step_msg");

  // Publishers
  YSurfN_Fext_pub_    = node_->create_publisher<std_msgs::msg::Float64>("YSurfN_Fext", 20);
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

  // Timer (10ms 권장). 메인루프에서 실제 dt는 steady_clock으로 산출.
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10),
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
  LD_X=LD_Y=LD_Z=LD_Roll=LD_Pitch=LD_Yaw=LD_CFx=LD_CFy=LD_CFz=0.0f;
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

    if (mode_cmd == Hand_guiding_mode_cmd) {
      ctrl.store(2, std::memory_order_release);
      set_status(message_status, Hand_guiding_mode);
    }
    else if (mode_cmd == Continuous_reording_start) {
      path_recording_flag = true;

      if (hand_g_recording) { std::fclose(hand_g_recording); hand_g_recording = nullptr; }

      auto hand_path = yaml_get_path(NRS_recording, "hand_g_recording", node_->get_logger());
      if (hand_path.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "hand_g_recording path invalid; recording aborted.");
        path_recording_flag = false;
        set_status(message_status, "Recording path invalid");
        return;
      }
      ensure_parent_dir(hand_path, node_->get_logger());

      hand_g_recording = std::fopen(hand_path.c_str(), "wt");
      if (!hand_g_recording) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (%s)", hand_path.c_str(), std::strerror(errno));
        path_recording_flag = false;
        return;
      }
      set_status(message_status, Data_recording_on);
    }
    else if (mode_cmd == Continusous_recording_end) {
      path_recording_flag = false;
      if (hand_g_recording) { std::fclose(hand_g_recording); hand_g_recording = nullptr; }
      set_status(message_status, Data_recording_off);
    }
    else if (mode_cmd == Discrete_reording_start) {
      if (Num_RD_points != 0) {
        Inst_RD_points = Decr_RD_points;
        Decr_RD_points.resize(Num_RD_points+1, 6);
        Decr_RD_points.topRows(Num_RD_points) = Inst_RD_points;
      } else {
        Decr_RD_points.topRows(Num_RD_points+1) = Inst_RD_points;
      }
      Decr_RD_points.bottomRows(1) << RArm.xc(0), RArm.xc(1), RArm.xc(2), RArm.thc(0), RArm.thc(1), RArm.thc(2);
      Num_RD_points++;
      std::snprintf(Saved_way_point, sizeof(Saved_way_point), "Saved way point: %d", Num_RD_points);
      set_status(message_status, Saved_way_point);
      std::cout << "\n" << Decr_RD_points << std::endl;
    }
    else if (mode_cmd == Discrete_recording_save) {
      auto path = yaml_get_path(NRS_recording, "Discre_P_recording", node_->get_logger());
      if (path.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Discre_P_recording path invalid; save aborted.");
        return;
      }
      ensure_parent_dir(path, node_->get_logger());

      if (Discre_P_recording) { std::fclose(Discre_P_recording); Discre_P_recording = nullptr; }
      Discre_P_recording = std::fopen(path.c_str(), "wt");
      if (!Discre_P_recording) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (%s)", path.c_str(), std::strerror(errno));
        return;
      }
      for (int i = 0; i < Num_RD_points; i++) {
        std::fprintf(Discre_P_recording, "%10f %10f %10f %10f %10f %10f %10f\n",
          Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2),
          Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), 0.0);
      }
      std::fclose(Discre_P_recording); Discre_P_recording = nullptr;
      Num_RD_points = 0;
      Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
      Inst_RD_points = Eigen::MatrixXd::Zero(1,6);
      printf("\n Discrete points saved to txt file \n");
    }
    else if (mode_cmd == VRTeac_reording_start) {
      if (Num_RD_points != 0) {
        Inst_RD_points = Decr_RD_points;
        Decr_RD_points.resize(Num_RD_points+1,6);
        Decr_RD_points.topRows(Num_RD_points) = Inst_RD_points;
      } else {
        Decr_RD_points.topRows(Num_RD_points+1) = Inst_RD_points;
      }
      Decr_RD_points.bottomRows(1) <<
        VR_CalPoseRPY(0), VR_CalPoseRPY(1), VR_CalPoseRPY(2),
        VR_CalPoseRPY(3), VR_CalPoseRPY(4), VR_CalPoseRPY(5);
      Num_RD_points++;
      std::snprintf(Saved_way_point, sizeof(Saved_way_point), "Saved way point: %d", Num_RD_points);
      set_status(message_status, Saved_way_point);
      std::cout << "\n" << Decr_RD_points << std::endl;
    }
    else if (mode_cmd == VRTeac_recording_save) {
      auto path = yaml_get_path(NRS_recording, "Discre_P_recording", node_->get_logger());
      if (path.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Discre_P_recording path invalid; save aborted.");
        return;
      }
      ensure_parent_dir(path, node_->get_logger());

      if (Discre_P_recording) { std::fclose(Discre_P_recording); Discre_P_recording = nullptr; }
      Discre_P_recording = std::fopen(path.c_str(), "wt");
      if (!Discre_P_recording) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (%s)", path.c_str(), std::strerror(errno));
        return;
      }
      for (int i = 0; i < Num_RD_points; i++) {
        std::fprintf(Discre_P_recording, "%10f %10f %10f %10f %10f %10f %10f\n",
          Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2),
          Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), 0.0);
      }
      std::fclose(Discre_P_recording); Discre_P_recording = nullptr;
      Num_RD_points = 0;
      Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
      Inst_RD_points = Eigen::MatrixXd::Zero(1,6);
      printf("\n VR teach points saved \n");
    }
    else if (mode_cmd == VRCali_reording_start) {
      // EE
      if (Num_EE_points != 0) {
        Inst_EE_points = Decr_EE_points;
        Decr_EE_points.resize(Num_EE_points+1,12);
        Decr_EE_points.topRows(Num_EE_points) = Inst_EE_points;
      } else {
        Decr_EE_points.topRows(Num_EE_points+1) = Inst_EE_points;
      }
      Decr_EE_points.bottomRows(1) <<
        RArm.Tc(0,0),RArm.Tc(1,0),RArm.Tc(2,0),
        RArm.Tc(0,1),RArm.Tc(1,1),RArm.Tc(2,1),
        RArm.Tc(0,2),RArm.Tc(1,2),RArm.Tc(2,2),
        RArm.Tc(0,3),RArm.Tc(1,3),RArm.Tc(2,3);
      Num_EE_points++;
      std::snprintf(Saved_way_point, sizeof(Saved_way_point), "Saved cali. points: %d", Num_EE_points);
      set_status(message_status, Saved_way_point);
      std::cout << "\n" << Decr_EE_points << std::endl;

      // VR
      if (Num_VR_points != 0) {
        Inst_VR_points = Decr_VR_points;
        Decr_VR_points.resize(Num_VR_points+1,7);
        Decr_VR_points.topRows(Num_VR_points) = Inst_VR_points;
      } else {
        Decr_VR_points.topRows(Num_VR_points+1) = Inst_VR_points;
      }
      Decr_VR_points.bottomRows(1) <<
        VR_pose[0],VR_pose[1],VR_pose[2],VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6];
      Num_VR_points++;
      std::snprintf(Saved_way_point, sizeof(Saved_way_point), "Saved VR points: %d", Num_VR_points);
      set_status(message_status, Saved_way_point);
      std::cout << "\n" << Decr_VR_points << std::endl;
    }
    else if (mode_cmd == VRCali_recording_save) {
      auto ee_path = yaml_get_path(NRS_recording, "VRCali_UR10CB_EE", node_->get_logger());
      auto vr_path = yaml_get_path(NRS_recording, "VRCali_UR10CB_VR", node_->get_logger());
      if (ee_path.empty() || vr_path.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "VRCali save path invalid; save aborted.");
        return;
      }
      ensure_parent_dir(ee_path, node_->get_logger());
      ensure_parent_dir(vr_path, node_->get_logger());

      if (VRCali_UR10CB_EE) { std::fclose(VRCali_UR10CB_EE); VRCali_UR10CB_EE = nullptr; }
      if (VRCali_UR10CB_VR) { std::fclose(VRCali_UR10CB_VR); VRCali_UR10CB_VR = nullptr; }

      VRCali_UR10CB_EE = std::fopen(ee_path.c_str(), "wt");
      if (!VRCali_UR10CB_EE) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (%s)", ee_path.c_str(), std::strerror(errno));
        return;
      }
      for (int i = 0; i < Num_EE_points; i++) {
        std::fprintf(VRCali_UR10CB_EE,
          "%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
          Decr_EE_points(i,0), Decr_EE_points(i,1), Decr_EE_points(i,2),
          Decr_EE_points(i,3), Decr_EE_points(i,4), Decr_EE_points(i,5),
          Decr_EE_points(i,6), Decr_EE_points(i,7), Decr_EE_points(i,8),
          Decr_EE_points(i,9), Decr_EE_points(i,10), Decr_EE_points(i,11));
      }
      std::fclose(VRCali_UR10CB_EE); VRCali_UR10CB_EE = nullptr;
      Num_EE_points = 0;
      Decr_EE_points = Eigen::MatrixXd::Zero(1,12);
      Inst_EE_points = Eigen::MatrixXd::Zero(1,12);
      printf("\n Discrete EE points saved \n");

      VRCali_UR10CB_VR = std::fopen(vr_path.c_str(), "wt");
      if (!VRCali_UR10CB_VR) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (%s)", vr_path.c_str(), std::strerror(errno));
        return;
      }
      for (int i = 0; i < Num_VR_points; i++) {
        std::fprintf(VRCali_UR10CB_VR, "%10f %10f %10f %10f %10f %10f %10f\n",
          Decr_VR_points(i,0), Decr_VR_points(i,1), Decr_VR_points(i,2),
          Decr_VR_points(i,3), Decr_VR_points(i,4), Decr_VR_points(i,5), Decr_VR_points(i,6));
      }
      std::fclose(VRCali_UR10CB_VR); VRCali_UR10CB_VR = nullptr;
      Num_VR_points = 0;
      Decr_VR_points = Eigen::MatrixXd::Zero(1,7);
      Inst_VR_points = Eigen::MatrixXd::Zero(1,7);
      printf("\n Cali points saved \n");
    }
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
  PB_iter_cmd = msg->data; PB_iter_cur = 1; // 1 is right
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

    // 필요 시 퍼블리시/로그 확장 가능
    (void)F_base;
}

// ===================== InitMove() =====================
// 역할: 재생 시작 시 현재 위치→TXT 첫 포즈로 시간기반 보간 이동(SLERP+선형)
bool JointControl::InitMove(double dt_s)
{
    // ---- 파라미터(필요시 rosparam으로 승격 가능) ----
    static constexpr double LIN_VEL   = 0.20;  // m/s  (기존 0.05 -> 빠르게)
    static constexpr double ANG_VEL   = 1.0;   // rad/s
    static constexpr double MIN_DUR   = 0.8;   // s    (너무 짧지 않게)
    static constexpr double MAX_DUR   = 4.0;   // s    (너무 길지 않게)

    // ---- 내부 상태 ----
    static bool active = false, finished = false;
    static double elapsed = 0.0, duration = 0.0;
    static Eigen::Vector3d start_xyz, goal_xyz;
    static Eigen::Matrix3d start_rot, goal_rot;
    static FILE* last_handle = nullptr;

    // 파일 핸들이 바뀌면 상태 리셋(재생 재시작 안정화)
    if (Hand_G_playback != last_handle) {
        active = false; finished = false; elapsed = 0.0; duration = 0.0;
        last_handle = Hand_G_playback;
    }

    // --- 최초 진입 ---
    if (!active && !finished) {
        if (!Hand_G_playback) return false;

        // TXT 첫 유효 라인 읽기(#만 스킵)
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

        start_xyz = RArm.xc;
        goal_xyz  = Eigen::Vector3d(x, y, z);

        start_rot = RArm.Tc.block<3,3>(0,0);
        Eigen::Vector3d goal_rpy(r,p,yw);
        AKin.EulerAngle2Rotation(goal_rot, goal_rpy);

        // 시간 산정: 선형/회전 요구시간 중 큰 값 사용 + 캡핑
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

    // --- 진행 ---
    elapsed += dt_s;
    const double alpha = std::clamp(elapsed / std::max(1e-6, duration), 0.0, 1.0);

    const Eigen::Vector3d xyz_interp = (1.0 - alpha) * start_xyz + alpha * goal_xyz;
    Desired_XYZ = xyz_interp;

    Eigen::Quaterniond q0(start_rot), q1(goal_rot);
    if (q0.dot(q1) < 0.0) q1.coeffs() *= -1.0;
    const Eigen::Quaterniond q_interp = q0.slerp(alpha, q1).normalized();
    const Eigen::Matrix3d R_interp = q_interp.toRotationMatrix();
    Desired_RPY = R_interp.eulerAngles(0,1,2);

    // IK 변환 (TCP -> 플랜지, z에 TOOL_Z 더함)
    Eigen::Vector3d Desired_XYZ_cmd = Desired_XYZ;
    Desired_XYZ_cmd(2) += TOOL_Z;

    RArm.Td << R_interp(0,0),R_interp(0,1),R_interp(0,2),Desired_XYZ_cmd(0),
                R_interp(1,0),R_interp(1,1),R_interp(1,2),Desired_XYZ_cmd(1),
                R_interp(2,0),R_interp(2,1),R_interp(2,2),Desired_XYZ_cmd(2),
                0,0,0,1;
#if TCP_standard == 0
    AKin.InverseK_min(&RArm);
#else
    AKin.Ycontact_InverseK_min(&RArm);
#endif

    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    // --- 종료 ---
    if (alpha >= 1.0 - 1e-6) {
        active = false;
        finished = true;
        std::rewind(Hand_G_playback); // PathFollow가 처음부터 읽도록
        printf("[InitMove] completed.\n");
    }
    return finished;
}



// ===================== PathFollow() =====================
// 역할: TXT를 한 줄씩 읽어 목표 포즈로 IK → 조인트 명령 퍼블리시
bool JointControl::PathFollow(double /*dt_s*/)
{
    // 재생 파일 핸들이 바뀌면 active 재활성화
    static bool active = true;
    static FILE* last_handle = nullptr;
    if (Hand_G_playback != last_handle) {
        active = true;
        last_handle = Hand_G_playback;
    }

    if (!active) return false;

    if (!Hand_G_playback) {
        RCLCPP_ERROR(node_->get_logger(), "[PB] playback file closed unexpectedly.");
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, "Playback file closed");
        return false;
    }

    // TXT: x y z r p y fx fy fz
    float LD_X, LD_Y, LD_Z;
    float LD_Roll, LD_Pitch, LD_Yaw;
    float LD_CFx, LD_CFy, LD_CFz;

    int reti = std::fscanf(Hand_G_playback, "%f %f %f %f %f %f %f %f %f",
                           &LD_X, &LD_Y, &LD_Z,
                           &LD_Roll, &LD_Pitch, &LD_Yaw,
                           &LD_CFx, &LD_CFy, &LD_CFz);

    if (reti != 9) {
        std::fclose(Hand_G_playback);
        Hand_G_playback = nullptr;
        active = false;

        // Return 준비
        return_active_  = true;
        return_elapsed_ = 0.0;
        return_duration_= 4.0;
        for (int i = 0; i < DOF; ++i) return_start_q_(i) = RArm.qc(i);

        printf("[PB] End of file. Start return-to-home.\n");
        return false;
    }

    Desired_XYZ << (double)LD_X, (double)LD_Y, (double)LD_Z;
    Desired_RPY << (double)LD_Roll, (double)LD_Pitch, (double)LD_Yaw;

    Contact_Rot_force(0) = (double)LD_CFx;
    Contact_Rot_force(1) = (double)LD_CFy;
    Contact_Rot_force(2) = (double)LD_CFz;

    Eigen::Matrix3d Desired_rot;
    AKin.EulerAngle2Rotation(Desired_rot, Desired_RPY);

    // ⬅️ FIX: IK용 플랜지 z = TCP z + TOOL_Z
    Eigen::Vector3d Desired_XYZ_cmd = Desired_XYZ;
    Desired_XYZ_cmd(2) += TOOL_Z;

    RArm.Td << Desired_rot(0,0), Desired_rot(0,1), Desired_rot(0,2), Desired_XYZ_cmd(0),
                Desired_rot(1,0), Desired_rot(1,1), Desired_rot(1,2), Desired_XYZ_cmd(1),
                Desired_rot(2,0), Desired_rot(2,1), Desired_rot(2,2), Desired_XYZ_cmd(2),
                0,0,0,1;

#if TCP_standard == 0
    AKin.InverseK_min(&RArm);
#else
    AKin.Ycontact_InverseK_min(&RArm);
#endif

    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i)
        joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    return true;
}


// ===================== ReturnHomePose() =====================
// 역할: 현재 q에서 HOME_Q까지 시간기반 선형 보간으로 복귀
bool JointControl::ReturnHomePose(double dt_s)
{
    static bool   active   = false;
    static double elapsed  = 0.0;
    static double duration = 0.0;
    static Vector6d start_q;

    static const Vector6d HOME_Q = (Vector6d() <<
        0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, +M_PI / 2.0, 0.0).finished();

    // --- 활성화 ---
    if (return_active_ && !active) {
        active   = true;
        elapsed  = 0.0;
        duration = return_duration_;
        start_q  = return_start_q_;
    }

    // --- 진행 ---
    if (active) {
        elapsed += dt_s;
        const double alpha = std::clamp(elapsed / std::max(1e-6, duration), 0.0, 1.0);

        const Vector6d q_cmd = (1.0 - alpha) * start_q + alpha * HOME_Q;
        for (int i = 0; i < DOF; ++i) RArm.qd(i) = q_cmd(i);

        joint_state_.header.stamp = node_->now();
        for (int i = 0; i < DOF; ++i) joint_state_.position[i] = RArm.qd(i);
        joint_commands_pub_->publish(joint_state_);

        if (alpha >= 1.0 - 1e-6) {
            active = false;
            return_active_ = false;
            printf("[PB] Return-to-home done.\n");
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

  milisec += dt_s * 1000.0;

  // 가속/속도/증분 초기화
  for (int i = 0; i < DOF; i++) {
    RArm.ddqd(i) = 0;
    RArm.dqd(i)  = 0;
    RArm.dqc(i)  = 0;
  }
  RArm.qd = RArm.qc;
  RArm.qt = RArm.qc;

  // 최신 FK & RPY
#if TCP_standard == 0
  AKin.ForwardK_T(&RArm);
#else
  AKin.Ycontact_ForwardK_T(&RArm);
#endif
  AKin.Rotation2EulerAngle(&RArm); // Tc -> thc
  RArm.Td = RArm.Tc;

  // 클래스 보조 상태 동기화(가시화용)
  for (int i = 0; i < DOF; ++i) joint_pos[i] = RArm.qc(i);

  // TCP→Base 힘 변환(참고용)
  Eigen::Vector3d F_base = Eigen::Vector3d::Zero();
  {
      const Eigen::Matrix3d R_TCP_base = RArm.Tc.block<3,3>(0,0);
      const Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);
      F_base = R_TCP_base * F_TCP;
  }

  // 상태 로드
  const int control_mode     = ctrl.load(std::memory_order_relaxed);
  const int pre_control_mode = pre_ctrl.load(std::memory_order_relaxed);

  // ====== 디버그 출력(주기 제한) ======
  if (printer_counter >= print_period) {
  #if RT_printing
      printf("======================================== \n");
      printf("RUN MODE %d (prev %d)\n", control_mode, pre_control_mode);

      printf("q  : %.3f %.3f %.3f %.3f %.3f %.3f\n",
            RArm.qc(0), RArm.qc(1), RArm.qc(2), RArm.qc(3), RArm.qc(4), RArm.qc(5));
      printf("qd : %.3f %.3f %.3f %.3f %.3f %.3f\n",
            RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5));

      printf("Act_XYZ: %.3f %.3f %.3f | Act_RPY: %.3f %.3f %.3f\n",
            RArm.xc(0), RArm.xc(1), RArm.xc(2),
            RArm.thc(0), RArm.thc(1), RArm.thc(2));

      printf("Des_XYZ: %.3f %.3f %.3f | Des_RPY: %.3f %.3f %.3f\n",
            Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2),
            Desired_RPY(0), Desired_RPY(1), Desired_RPY(2));

      printf("Contact Fz: %.2f -> Base: %.3f %.3f %.3f\n",
            contact_force, F_base(0), F_base(1), F_base(2));
  #endif
      printer_counter = 0;
  } else {
      printer_counter++;
  }


  // ====== 토픽 퍼블리시(비차단/재사용 버퍼) ======
  UR10_pose_msg_.data.resize(6);
  UR10_wrench_msg_.data.resize(6);
  for(int i=0; i<6; i++){
    UR10_pose_msg_.data[i]   = (i<3) ? RArm.xc(i) : RArm.thc(i-3);
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
          joint_pos[i] = HOME_Q[i];  // 가시화용 동기화
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

  // 1) Cartesian / Joint 경로 실행 (기존 로직 유지)
  if (control_mode == 1) {
    if(path_done_flag == true) {
      if(path_exe_counter<Path_point_num) {
        if(mode_cmd == Joint_control_mode_cmd) {
          RArm.qd(0) = Joint_path_start(0) + ((double)(mjoint_cmd[0]==1))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(1) = Joint_path_start(1) + ((double)(mjoint_cmd[0]==2))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(2) = Joint_path_start(2) + ((double)(mjoint_cmd[0]==3))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(3) = Joint_path_start(3) + ((double)(mjoint_cmd[0]==4))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(4) = Joint_path_start(4) + ((double)(mjoint_cmd[0]==5))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(5) = Joint_path_start(5) + ((double)(mjoint_cmd[0]==6))*J_single.Final_pos(path_exe_counter,1);

          if (path_recording_joint) {
            std::fprintf(path_recording_joint,"%10f %10f %10f %10f %10f %10f \n",
              RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5));
          }
        }
        else if (mode_cmd == EE_Posture_control_mode_cmd) {
          Desired_XYZ << TCP_path_start(0), TCP_path_start(1), TCP_path_start(2);
          Desired_RPY << TCP_path_start(3)+path_planning.Final_pos(path_exe_counter,1),
                          TCP_path_start(4), TCP_path_start(5);
          AKin.EulerAngle2Rotation(Desired_rot,Desired_RPY);
          RArm.Td << Desired_rot(0,0),Desired_rot(0,1),Desired_rot(0,2),Desired_XYZ(0),
                      Desired_rot(1,0),Desired_rot(1,1),Desired_rot(1,2),Desired_XYZ(1),
                      Desired_rot(2,0),Desired_rot(2,1),Desired_rot(2,2),Desired_XYZ(2),
                      0,0,0,1;
#if TCP_standard == 0
          AKin.InverseK_min(&RArm);
#else
          AKin.Ycontact_InverseK_min(&RArm);
#endif
          if (path_recording_pos) {
            std::fprintf(path_recording_pos,"%10f %10f %10f %10f %10f %10f\n",
              PPB_RTinput.PFd, Contact_Rot_force(2),
              Power_PB.PTankE, Desired_rot(0,2), Desired_rot(1,2), Desired_rot(2,2));
          }
        }
        path_exe_counter++;
      } else {
        path_done_flag = false;
        path_exe_counter = 0;
      }
    }

    // 명령 전송 (시뮬)
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 2) Hand-guiding control mode (현 버전은 별 동작 없음, 유지)
  if (control_mode == 2) {
    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 3) Playback: InitMove → PathFollow → ReturnHomePose
  if (control_mode == 3) {
      static bool init_done = false;

      // 모드 전환 시 InitMove부터 다시 시작
      if (pre_control_mode != 3) {
          init_done = false;
      }

      if (!init_done) {
          bool just_finished = InitMove(dt_s);
          if (!just_finished) {
              // 아직 InitMove 진행 중이면 여기서 종료
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          }
          // ⬇️ 바로 PathFollow로 이어감 (다음 틱 기다리지 않음)
          init_done = true;
      }

      if (PathFollow(dt_s)) {
          pre_ctrl.store(control_mode, std::memory_order_relaxed);
          return;
      }

      // PathFollow가 false를 반환하면 EOF → Return 단계
      ReturnHomePose(dt_s);
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
