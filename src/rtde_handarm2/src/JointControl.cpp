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

  // Timer (10ms 권장). 메인루프에서 실제 dt는 steady_clock으로 산출.
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(1),
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




bool JointControl::PathFollow(double dt_s)
{
    // =========================================================================
    // [재생 상태 관리 - playback 활성 플래그]
    //
    // Hand_G_playback : 현재 열려있는 재생 txt 파일 핸들 (멤버/전역)
    // active          : 이 함수 안에서 유지되는 static 상태. true일 때만 재생.
    //
    // - 파일 핸들이 바뀌면 active=true로 리셋해서 새 재생 시작
    // - 파일이 끝나면 active=false로 만들고 false 리턴 → 상위 루프는 홈 복귀 단계로
    // =========================================================================
    static bool  active      = true;
    static FILE* last_handle = nullptr;
    if (Hand_G_playback != last_handle) {
        active      = true;
        last_handle = Hand_G_playback;
    }
    if (!active) {
        return false; // 이미 재생 종료 상태
    }

    if (!Hand_G_playback) {
        // 파일 핸들이 없는데 active라면 예외 상황 → 안전 정지
        RCLCPP_ERROR(node_->get_logger(), "[PB] playback file closed unexpectedly.");
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, "Playback file closed");
        return false;
    }


    // =========================================================================
    // [STEP 1] TXT에서 목표 pose/force 한 줄 읽기
    //
    // txt 라인 포맷 (공백 구분):
    //   x y z r p yaw fx fy fz
    //
    // 의미:
    //   x y z     : 목표 TCP 위치 [m], base frame 기준
    //   r p yaw   : 목표 오리엔테이션 [rad], roll-pitch-yaw (yaw=Z, pitch=Y, roll=X 순)
    //   fx fy fz  : 원하는 힘 [N], base frame 기준 "누르고 싶은 방향/크기"
    //
    // fscanf() 결과(reti)가 9가 아니면:
    //   → EOF 또는 파싱 실패로 간주
    //   → 파일 닫기
    //   → 홈 복귀(ReturnHomePose)를 위한 초기 상태 세팅
    //   → false 리턴 (상위 루프가 ReturnHomePose() 호출)
    // =========================================================================
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
        // --- 재생 종료 처리 ---------------------------------
        std::fclose(Hand_G_playback);
        Hand_G_playback = nullptr;
        active = false;

        // Return-to-home 초기화:
        //  return_active_   : ReturnHomePose()가 "이제 홈으로 가자" 인지 알 수 있는 플래그
        //  return_elapsed_  : 경과 시간 적분용 카운터 (0으로 초기화)
        //  return_duration_ : 홈까지 선형보간에 쓸 전체 시간(초)
        //  return_start_q_  : 복귀 시작 시점의 관절각(qc)을 저장해둔다
        return_active_   = true;
        return_elapsed_  = 0.0;
        return_duration_ = 4.0; // 기본 4초짜리 복귀 (필요시 조정 가능)
        for (int i = 0; i < DOF; ++i) {
            return_start_q_(i) = RArm.qc(i);
        }

        printf("[PB] End of file. Start return-to-home.\n");
        return false; // 상위 CalculateAndPublishJoint()에서 ReturnHomePose()로 넘어감
    }

    // txt에서 읽은 목표를 Eigen 벡터로 옮긴다.
    Eigen::Vector3d Xd;    // 원하는 위치 [m]
    Eigen::Vector3d RPYd;  // 원하는 (roll, pitch, yaw) [rad]
    Eigen::Vector3d Fd;    // 원하는 힘 [N] (base frame 기준)

    Xd   << (double)des_x,  (double)des_y,   (double)des_z;
    RPYd << (double)des_r,  (double)des_p,   (double)des_yaw;
    Fd   << (double)des_fx, (double)des_fy,  (double)des_fz;

    // 디버깅용으로 상위 printf에서 쓰는 상태 변수도 갱신
    Desired_XYZ = Xd;
    Desired_RPY = RPYd;


    // =========================================================================
    // [STEP 2] 현재 외력(측정 contact force) 추정 + LPF + saturation
    //
    // 현재 가져오는 접촉힘은 contact_force 하나뿐:
    //   contact_force ≈ TCP 프레임 z축 방향 힘 F_TCP.z
    //
    // 이것을 base frame 힘 벡터로 변환:
    //   1) R_base_TCP = RArm.Tc(0:2,0:2)  (Base→TCP 회전)
    //   2) F_TCP = [0, 0, contact_force]
    //   3) F_base = R_base_TCP * F_TCP
    //   4) F_ext  = -F_base (부호 보정: "양수면 로봇을 누르는 방향"으로 정규화)
    //
    // 이후:
    //   - 1차 저역통과(LPF)로 노이즈/스파이크 줄임
    //   - saturation으로 말도 안 되는 큰 튀김 값 잘라냄
    //
    // 추가로 현재 EE 실제 위치 X_act도 가져온다.
    //
    // force_control.cpp(KUKA)와의 차이점:
    //   * 거기는 이미 base/TCP에서 풀 6축 FT 데이터를 받는다(ft1data[0..5]).
    //   * 여긴 Isaac Sim에서 얻은 단일 접촉 스칼라(contact_force)만 직접 회전시켜서
    //     base-frame 외력 벡터로 재구성한다.  ★중요 차이★
    // =========================================================================
    Eigen::Matrix3d R_base_TCP = RArm.Tc.block<3,3>(0,0); // Base→TCP 회전행렬
    Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);       // TCP 프레임 힘(우리는 z만)
    Eigen::Vector3d F_base = R_base_TCP * F_TCP;          // Base 프레임 힘
    Eigen::Vector3d F_ext  = -F_base;                     // 부호 보정

    // --- LPF: 저역통과필터로 매끄럽게 ---
    {
        static Eigen::Vector3d F_lp = Eigen::Vector3d::Zero();
        static bool first_f = true;

        const double fc = 15.0; // [Hz] 커트오프
        const double Ts = (dt_s > 0.0 ? dt_s : 0.001); // 샘플링주기 추정
        const double alpha = (2.0 * M_PI * fc * Ts) / (1.0 + 2.0 * M_PI * fc * Ts);

        if (first_f) {
            F_lp   = F_ext;
            first_f = false;
        } else {
            F_lp = F_lp + alpha * (F_ext - F_lp);
        }
        F_ext = F_lp;
    }

    // --- Saturation: LPF 후에도 말도 안 되는 큰 spike는 잘라낸다 ---
    {
        const double FEXT_SAT = 30.0; // [N] 축별 최대 허용
        for (int k = 0; k < 3; ++k) {
            if (F_ext(k) >  FEXT_SAT) F_ext(k) =  FEXT_SAT;
            if (F_ext(k) < -FEXT_SAT) F_ext(k) = -FEXT_SAT;
        }
    }

    // 현재 EE 실제 위치 (base frame, [m])
    Eigen::Vector3d X_act = RArm.xc;


    // =========================================================================
    // [STEP 3] 목표 오리엔테이션(RPY) → 목표 spatial angle (회전벡터) 변환
    //
    // FAAC/Admittance 쪽은 orientation을 roll,pitch,yaw 대신 "spatial angle"
    // (= 회전행렬의 로그맵: 회전축 * 회전각) 형태 [wx, wy, wz] 로 쓰는 경우가 많음.
    //
    // 여기서는:
    //   1) RPYd (roll,pitch,yaw) → 회전행렬 Rd_R
    //   2) Rd_R → 회전벡터(축*각) = Wd
    //
    // force_control.cpp(KUKA)와의 차이점:
    //   * 거기는 YMatrix / fromSpatialAngle 같은 자체 유틸 사용
    //   * 우리는 Eigen으로 직접 계산한다.  ★차이★
    // =========================================================================
    auto rotFromRPY = [](const Eigen::Vector3d &rpy)->Eigen::Matrix3d {
        const double cr = std::cos(rpy(0));
        const double sr = std::sin(rpy(0));
        const double cp = std::cos(rpy(1));
        const double sp = std::sin(rpy(1));
        const double cy = std::cos(rpy(2));
        const double sy = std::sin(rpy(2));

        // Z(yaw) * Y(pitch) * X(roll) 순으로 가정
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
        // 회전행렬 → 회전벡터(axis * angle)
        double cos_theta = (R.trace() - 1.0) * 0.5;
        if (cos_theta >  1.0) cos_theta =  1.0;
        if (cos_theta < -1.0) cos_theta = -1.0;

        double theta = std::acos(cos_theta);
        if (theta < 1e-9) {
            // 아주 작은 회전은 거의 0벡터로 간주
            return Eigen::Vector3d::Zero();
        }

        Eigen::Vector3d omega;
        omega << R(2,1) - R(1,2),
                 R(0,2) - R(2,0),
                 R(1,0) - R(0,1);
        omega *= 0.5 / std::sin(theta);

        return theta * omega; // (축 * 각) 형태
    };

    Eigen::Matrix3d Rd_R = rotFromRPY(RPYd); // 목표 회전행렬
    Eigen::Vector3d Wd   = rotLog(Rd_R);     // 목표 spatial angle [wx, wy, wz]


    // =========================================================================
    // [STEP 4] FAAC + 어드미턴스 제어
    //
    // 목표:
    //   - 위치축(x,y,z)에 대해:
    //       FAAC(3-step adaptive admittance) + 1D 어드미턴스 컨트롤을 통해
    //       "순응적으로 보정된" 명령 위치 Xc_cmd 를 만든다.
    //
    //   - 자세축(wx,wy,wz)에 대해:
    //       아직은 힘/토크 기반 적응 없이 그냥 목표 Wd 그대로 사용.
    //
    // 내부 static 상태:
    //   fc_init        : 이 블록의 1회성 초기화 여부
    //   AControl[6]    : 축별 어드미턴스 컨트롤러 (Yadmittance_control)
    //   FAAC3step[3]   : x,y,z 축용 3-step FAAC 객체 (Nrs3StepFAAC)
    //   FAAC_flag[3]   : 축별로 "이미 힘제어 모드 들어갔는지" 여부
    //   AC_pose_pos[3] : 지난 루프에서 출력한 순응 위치 명령 (x,y,z)
    //   AC_pose_ori[3] : 지난 루프에서 출력한 순응 orientation 명령 (wx,wy,wz)
    //
    // FC_MASS / FC_DAMPER / FC_STIFFNESS :
    //   어드미턴스 M,D,K 초기값.
    //   force_control.cpp 의 kuka_motion::control_force()에서 쓰던 값과 유사 스케일.
    //
    // Tank_energy :
    //   FAAC 에너지탱크 로직의 파라미터. 여기선 임시로 상수 5.0.
    //
    // 안정화 포인트(중요):
    //   1) Fd → Fd_cmd : 목표 힘을 그대로 쓰지 않고 up/down 서로 다른 타임상수로 램프.
    //      - 올라갈 땐 천천히(충격 완화)
    //      - 내려올 땐 빨리(접촉 끝났을 때 잔류 힘을 빨리 0으로)
    //
    //   2) Contact gating:
    //      - 일정 힘 이상으로 바닥 등을 누르기 시작하면 그 순간의 z를 lock.
    //        그보다 더 내려가려 할 땐 막아서 바닥 뚫고 진동하는 걸 방지.
    //      - 하지만 힘 명령이 다시 거의 0이 되면 lock 풀어줘서
    //        로봇이 공중에서 얼어붙는 걸 막는다.
    //
    //   3) Step clamp:
    //      - 한 주기당 이동량(특히 z)을 강하게 제한.
    //
    // force_control.cpp(KUKA)와 다른 점:
    //   * 거긴 kuka_motion 클래스 멤버로 AControl/FAAC 들고 있고,
    //     orientation도 그 프레임워크에 맞춰 처리.
    //   * 우린 UR10 제어라 PathFollow() 내부 static으로 직접 유지.   ★차이★
    // =========================================================================
    static bool fc_init = false;

    // 6축 어드미턴스 컨트롤러 (x,y,z, wx,wy,wz)
    static Yadmittance_control AControl[6] = {
        Yadmittance_control(0.001),
        Yadmittance_control(0.001),
        Yadmittance_control(0.001),
        Yadmittance_control(0.001),
        Yadmittance_control(0.001),
        Yadmittance_control(0.001)
    };

    // x,y,z 축용 FAAC 객체
    static std::unique_ptr<Nrs3StepFAAC> FAAC3step[3];

    // 각 위치축별로 FAAC 활성화 여부
    static bool FAAC_flag[3] = {false,false,false};

    // 지난 루프의 어드미턴스 출력 (누적 상태)
    static double AC_pose_pos[3] = {0.0,0.0,0.0}; // x,y,z
    static double AC_pose_ori[3] = {0.0,0.0,0.0}; // wx,wy,wz (지금은 Wd 그대로만 저장)

    // 어드미턴스 초기 파라미터들
    static double FC_MASS[6]      = {1.0,   1.0,   1.0,   0.05, 0.05, 0.05};
    // static double FC_DAMPER[6]    = {3000., 3000., 3000., 10.0, 10.0, 10.0};
    static double FC_DAMPER[6]    = {6000., 6000., 6000., 10.0, 10.0, 10.0};
    static double FC_STIFFNESS[6] = {2000., 2000., 2000., 20.0, 20.0, 20.0};

    
    // --- 목표 힘 램프 (Fd -> Fd_cmd, up/down 서로 다른 타임상수) -------------
    //
    // 문제점(이전 버전):
    //   Fd가 10→0으로 뚝 떨어져도 저역통과가 느리면 Fd_cmd가 한동안 양수로 남아
    //   "아직 눌러야 해?" 라고 오해 → 로봇이 표면 위로 들려버리거나 꿀렁.
    //
    // 해결:
    //   올라갈 땐 alpha_up(작음)으로 천천히,
    //   내려갈 땐 alpha_down(큼)으로 빠르게.
    static Eigen::Vector3d Fd_cmd = Eigen::Vector3d::Zero();
    {
        const double alpha_up   = 0.02; // 목표 힘이 커질 때 부드럽게 상승
        const double alpha_down = 0.20; // 목표 힘이 줄 때 빠르게 감쇠

        for (int k = 0; k < 3; ++k) {
            double alpha = (std::fabs(Fd(k)) > std::fabs(Fd_cmd(k)))
                           ? alpha_up      // 더 세게 누르려는 중 → 천천히
                           : alpha_down;  // 힘을 빼는 중 → 빨리 내려가라
            Fd_cmd(k) += alpha * (Fd(k) - Fd_cmd(k));
        }

        // 램프된 힘도 안전하게 saturation
        const double FDES_SAT = 30.0; // [N]
        for (int k=0; k<3; ++k) {
            if (Fd_cmd(k) >  FDES_SAT) Fd_cmd(k) =  FDES_SAT;
            if (Fd_cmd(k) < -FDES_SAT) Fd_cmd(k) = -FDES_SAT;
        }
    }

    // --- 1회성 초기화: 어드미턴스 MDK 세팅 + FAAC 객체 생성 ------------------
    if (!fc_init) {
        // (a) 어드미턴스 기본 M,D,K
        for (int i = 0; i < 6; ++i) {
            AControl[i].adm_1D_MDK(
                FC_MASS[i],
                FC_DAMPER[i],
                FC_STIFFNESS[i]
            );
        }

        // (b) FAAC 인스턴스 준비
        //     (init_md, init_dd, init_kd, dt, process_noise, measurement_noise)
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
            FAAC_flag[ax] = false; // 처음엔 힘제어 모드 꺼져 있음
        }

        // (c) 첫 AC 출력을 현재 목표에서 시작
        AC_pose_pos[0] = Xd(0);
        AC_pose_pos[1] = Xd(1);
        AC_pose_pos[2] = Xd(2);

        AC_pose_ori[0] = Wd(0);
        AC_pose_ori[1] = Wd(1);
        AC_pose_ori[2] = Wd(2);

        fc_init = true;
    }

    // 이 스텝에서 만들 최종 명령:
    //   Xc_cmd : 이 루프에서 IK에 넣을 "명령 위치"
    //   Wc_cmd : 이 루프에서 IK에 넣을 "명령 spatial angle"
    Eigen::Vector3d Xc_cmd = Xd;
    Eigen::Vector3d Wc_cmd = Wd;

    const double Tank_energy = 5.0; // FAAC 내부 계산용 (고정값으로 사용 중)

    // ----- 접촉 게이팅(contact gating with release) -------------------------
    //
    // 목표:
    //   - 처음 표면을 누르기 시작하면 그 순간의 z를 lock 해서,
    //     그보다 더 아래(더 파고드는 방향)로는 못 가게 막는다.
    //   - 하지만 힘 명령/실제 힘이 거의 0으로 돌아오면 lock을 풀어줘서
    //     로봇이 공중에서 "고정된 자세"로 떠버리지 않게 한다.
    //
    static bool   contact_on     = false;
    static double contact_z_lock = 0.0;

    {
        const double CONTACT_ON_TH   = 2.0;  // [N] 접촉 시작으로 볼 최소 힘
        const double CONTACT_HOLD_TH = 0.5;  // [N] 이 이하이면 접촉 해제 가능하다고 판단

        // --- 접촉 시작 감지 ---
        // 조건:
        //   1) 아직 contact_on == false (안 누르고 있었음)
        //   2) 실제 외력 |F_ext(2)| 가 임계 이상 → 뭔가에 닿았다
        //   3) 의도된 힘명령 |Fd_cmd(2)| 도 충분히 큼 → 진짜 누를 의도
        if (!contact_on
            && std::fabs(F_ext(2))   > CONTACT_ON_TH
            && std::fabs(Fd_cmd(2))  > CONTACT_ON_TH)
        {
            contact_on     = true;
            contact_z_lock = AC_pose_pos[2]; // 그 순간의 z를 기억 (여기보다 더 낮게 못가게)
        }

        // --- 접촉 유지 여부 확인 (해제 조건) ---
        // 힘 의도(Fd_cmd)와 실제 힘(F_ext)이 둘 다 거의 0 근처면,
        // 이제는 더 안 누른다고 판단 → contact_on 해제
        if (contact_on
            && std::fabs(Fd_cmd(2)) < CONTACT_HOLD_TH
            && std::fabs(F_ext(2))  < CONTACT_HOLD_TH)
        {
            contact_on = false;
        }

        // --- z 하향(바닥쪽) 클램프 적용 ---
        // 아직 contact_on=true고, 그리고 "진짜 누를 의도"가 계속 있다면
        //   (즉 Fd_cmd(2)가 아직 어느 정도 남아있다면),
        // 기준 경로 Xd(2)가 lock보다 더 낮은 쪽(더 파고드는 쪽)으로 가려 할 때
        // 그걸 막는다.
        if (contact_on && std::fabs(Fd_cmd(2)) > CONTACT_HOLD_TH)
        {
            if (Xd(2) < contact_z_lock) {
                Xd(2) = contact_z_lock;
            }
        }
    }

    // ---- 위치축(x=0,y=1,z=2)에 대해 FAAC 및 어드미턴스 적용 -----------------
    for (int ax = 0; ax < 3; ++ax)
    {
        // (1) 목표 힘이 의미 있게 존재(|Fd_cmd|>0.01)하면 그 축의 FAAC를 활성화.
        //     한 번 활성화되면 계속 true.
        if (std::fabs(Fd_cmd(ax)) > 0.01 || FAAC_flag[ax]) {
            FAAC_flag[ax] = true;
        }

        // (2) 활성 축이면 FAAC로 새로운 M,D,K를 추정해서 그 축 어드미턴스에 반영
        if (FAAC_flag[ax] && FAAC3step[ax]) {
            auto faac_mdk = FAAC3step[ax]->FAAC_MDKob_RUN(
                Tank_energy,     // 임시 고정된 탱크 에너지 파라미터
                F_ext(ax),       // 현재 측정 외력 (LPF+sat 된 값)
                Fd_cmd(ax),      // ramp된 목표 힘
                AC_pose_pos[ax], // 지난 루프에서 어드미턴스가 낸 명령 위치
                X_act(ax)        // 실제 현재 위치
            );

            AControl[ax].adm_1D_MDK(
                faac_mdk.Mass,
                faac_mdk.Damping,
                faac_mdk.Stiffness
            );
        }

        // (3) 어드미턴스 1D 제어 실행
        // adm_1D_control(x_des, F_des, F_ext) -> compliant한 새 위치 명령
        double next_pos = AControl[ax].adm_1D_control(
            Xd(ax),      // (contact gating 후의) 기준 위치
            Fd_cmd(ax),  // ramp된 목표 힘
            F_ext(ax)    // 필터+클리핑된 외력
        );

        Xc_cmd(ax) = next_pos;
    }

    // ---- 자세축(wx,wy,wz)은 현재 힘제어/FAAC 미적용 ----
    // orientation은 그냥 목표 Wd를 그대로 쓴다.
    Wc_cmd = Wd;

    // ---- 위치 명령 안정화 (step clamp / offset clamp) ----------------------
    {
        static Eigen::Vector3d Xc_prev = Xd; // 이전 루프에서의 명령 위치 저장

        // 기준 경로(Xd)에서 너무 멀리 벗어나지 않도록 제한 (±1 cm)
        const double max_offset = 0.010; // [m]

        // 한 주기당 최대 이동량 (축별):
        //   x,y : 1 mm/틱
        //   z   : 0.3 mm/틱  (z는 바닥 방향이라 더 보수적으로 제한해 꿀렁 줄임)
        const double max_step_each[3] = {
            0.001,   // x
            0.001,   // y
            0.0003   // z
        };

        for (int ax=0; ax<3; ++ax) {
            // (a) 기준 경로 주변 offset 제한
            double lo = Xd(ax) - max_offset;
            double hi = Xd(ax) + max_offset;
            if (Xc_cmd(ax) < lo) Xc_cmd(ax) = lo;
            if (Xc_cmd(ax) > hi) Xc_cmd(ax) = hi;

            // (b) 한 틱에서의 step 크기 제한
            double d        = Xc_cmd(ax) - Xc_prev(ax);
            double max_step = max_step_each[ax];
            if (d >  max_step) d =  max_step;
            if (d < -max_step) d = -max_step;
            Xc_cmd(ax) = Xc_prev(ax) + d;
        }

        // 다음 루프 대비 저장
        Xc_prev = Xc_cmd;

        // 어드미턴스/FAAC 내부 상태 갱신 (다음 틱에 "직전 출력"으로 사용)
        AC_pose_pos[0] = Xc_cmd(0);
        AC_pose_pos[1] = Xc_cmd(1);
        AC_pose_pos[2] = Xc_cmd(2);
    }

    // orientation 누적 상태도 갱신(현재는 그냥 복사)
    AC_pose_ori[0] = Wc_cmd(0);
    AC_pose_ori[1] = Wc_cmd(1);
    AC_pose_ori[2] = Wc_cmd(2);


    // =========================================================================
    // [STEP 5] IK 입력 pose(Td) 구성
    //
    //    - 위치:   Xc_cmd (FAAC/어드미턴스 보정 결과)
    //    - 자세:   txt에서 준 RPYd 기반으로 회전행렬 다시 생성
    //              (아직 IK에 spatial angle Wc_cmd를 직접 넣지는 않음)
    //
    //    - TOOL_Z: TCP offset 보정.
    //              UR10 IK는 플랜지 기준 Pose를 기대하므로,
    //              실제 원하는 TCP z에 TOOL_Z를 더해 플랜지 위치를 만든다.
    //
    //    RArm.Td : IK 타겟 변환행렬(4x4 homogeneous transform).
    //              이걸 AKin.InverseK_min() 또는 AKin.Ycontact_InverseK_min()
    //              에 넣으면 RArm.qd 에 조인트 명령각이 채워진다.
    //
    // force_control.cpp(KUKA)와의 차이:
    //   * 거기는 kuka_motion::solve_IK() (QP solver 기반) 호출.
    //   * 우린 UR10 전용 AKin.*InverseK_min()을 호출.        ★차이★
    // =========================================================================
    Eigen::Vector3d flange_xyz = Xc_cmd;
    flange_xyz(2) += TOOL_Z;  // TCP(z) → 플랜지(z) 보정

    // 다시 RPY → 회전행렬 (UR10 kinematics util 사용)
    Eigen::Matrix3d Rd_R_again;
    AKin.EulerAngle2Rotation(Rd_R_again, RPYd);

    RArm.Td <<
        Rd_R_again(0,0), Rd_R_again(0,1), Rd_R_again(0,2), flange_xyz(0),
        Rd_R_again(1,0), Rd_R_again(1,1), Rd_R_again(1,2), flange_xyz(1),
        Rd_R_again(2,0), Rd_R_again(2,1), Rd_R_again(2,2), flange_xyz(2),
        0,               0,               0,               1;

#if TCP_standard == 0
    AKin.InverseK_min(&RArm);          // 일반 IK
#else
    AKin.Ycontact_InverseK_min(&RArm); // 접촉 상황 고려한 IK
#endif


    // =========================================================================
    // [STEP 6] 조인트 명령 publish
    //
    // joint_state_ :
    //   - 생성자에서 name[]과 position.resize()를 준비해둔 sensor_msgs::msg::JointState
    //   - 여기서 position[i] = RArm.qd(i) 로 업데이트하고
    //     joint_commands_pub_ 토픽(/isaac_joint_commands 등)에 publish한다.
    //
    // 상위 CalculateAndPublishJoint()에서 qc/qd/pose 등을 printf로 모니터링 중.
    // =========================================================================
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i) {
        joint_state_.position[i] = RArm.qd(i);
    }
    joint_commands_pub_->publish(joint_state_);


    // =========================================================================
    // [STEP 7] base frame 기준 외력(F_ext) publish
    //
    // STEP 2에서 구해놓은 F_ext (base frame):
    //   - contact_force (TCP z) -> base 회전변환 -> 부호정규화(-)
    //   - LPF
    //   - saturation
    //
    // 최종적으로 현재 EE에 작용 중이라고 추정되는 외력 [N] 이다.
    //
    // 여기서 이 F_ext(x,y,z)를 퍼블리시해서 rqt_plot 등으로 관찰할 수 있게 한다.
    //
    // force_ext_base_pub_ :
    //   - JointControl 생성자에서
    //     node_->create_publisher<std_msgs::msg::Float64MultiArray>("force_ext_base", 20)
    //     로 만들어 둔 퍼블리셔.
    //
    // message.data:
    //   [0] = Fx_base
    //   [1] = Fy_base
    //   [2] = Fz_base
    // =========================================================================
    {
        std_msgs::msg::Float64MultiArray force_msg;
        force_msg.data.resize(3);
        force_msg.data[0] = F_ext(0);
        force_msg.data[1] = F_ext(1);
        force_msg.data[2] = F_ext(2);
        force_ext_base_pub_->publish(force_msg);
    }

    // 아직 txt 재생 중이므로 true 반환
    return true;
}




bool JointControl::ReturnHomePose(double dt_s)
{
    // 내부 상태 유지용 static 변수들
    static bool     active    = false;   // 복귀 시퀀스 진행 중인지
    static double   elapsed   = 0.0;     // 누적 경과 시간 [s]
    static double   duration  = 10.0;    // 홈 복귀에 쓸 총 시간 [s] (느리게: 10초)
    static Vector6d start_q;             // 복귀 시작 관절각

    // 홈 자세 (라디안)
    static const Vector6d HOME_Q = (Vector6d() <<
        0.0,
        -M_PI / 2.0,
        -M_PI / 2.0,
        -M_PI / 2.0,
        +M_PI / 2.0,
        0.0).finished();

    // --- 복귀 시퀀스 시작 트리거 ---
    //
    // PathFollow()에서 EOF를 만나면
    //   return_active_  = true;
    //   return_start_q_ = 현재 로봇 관절각(RArm.qc)
    //
    // 여기서 그 신호를 보고 첫 1회만 latch
    if (return_active_ && !active) {
        active    = true;
        elapsed   = 0.0;
        duration  = 10.0;          // 무조건 10초 (느리게 이동)
        start_q   = return_start_q_;
    }

    // --- 복귀 중이라면 보간 수행 ---
    if (active) {
        // dt_s가 비정상적으로 큰 경우(타이머 hiccup 등) 너무 빨리 시간을 밀지 않도록 클램프
        double dt_step = dt_s;
        if (dt_step <= 0.0 || dt_step > 0.05) { // 50ms 이상 튀면 그냥 1ms로 본다
            dt_step = 0.001;
        }

        // 경과 시간 누적
        elapsed += dt_step;

        // 선형 진행비율 s_raw = [0..1]
        double s_raw = elapsed / std::max(1e-6, duration);
        if (s_raw < 0.0) s_raw = 0.0;
        if (s_raw > 1.0) s_raw = 1.0;

        // 부드러운 스케일 (smoothstep-ish: 3s^2 - 2s^3)
        //   - 초반/후반 속도 낮춰서 갑자기 확 잡아당기는 느낌 줄이기
        double s = (3.0 * s_raw * s_raw) - (2.0 * s_raw * s_raw * s_raw);

        // 보간된 목표 관절각 q_cmd
        Vector6d q_cmd;
        for (int i = 0; i < DOF; ++i) {
            q_cmd(i) = (1.0 - s) * start_q(i) + s * HOME_Q(i);
        }

        // qd에 반영하고 퍼블리시
        for (int i = 0; i < DOF; ++i) {
            RArm.qd(i) = q_cmd(i);
        }

        joint_state_.header.stamp = node_->now();
        for (int i = 0; i < DOF; ++i) {
            joint_state_.position[i] = RArm.qd(i);
        }
        joint_commands_pub_->publish(joint_state_);

        // 마무리: 다 도착했으면 모드 해제
        if (s_raw >= 1.0 - 1e-6) {
            active          = false;
            return_active_  = false;

            printf("[PB] Return-to-home done (10s smooth ramp).\n");

            // playback 모드 종료 → 홈 유지 모드(0)
            ctrl.store(0, std::memory_order_release);
            set_status(message_status, "Playback finished");
        }

        return true; // 복귀 시퀀스 진행 중
    }

    // 복귀 안 하고 있으면 false
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

  // TCP→Base 힘 변환 (for debug print)
  Eigen::Vector3d F_base_dbg = Eigen::Vector3d::Zero();
  {
      const Eigen::Matrix3d R_TCP_base = RArm.Tc.block<3,3>(0,0);  // Base<-TCP 회전
      const Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);        // TCP z축 힘
      Eigen::Vector3d F_base_raw = R_TCP_base * F_TCP;             // 변환된 힘 (Base 기준)

      // 제어 쪽에서 외력은 F_ext = -F_base_raw 를 쓰고 있으므로
      // 디버그도 그 부호에 맞춰서 출력한다 (contact_force와 같은 방향으로 보이도록 뒤집음)
      F_base_dbg = -F_base_raw;
  }

  // 상태 로드
  const int control_mode     = ctrl.load(std::memory_order_relaxed);
  const int pre_control_mode = pre_ctrl.load(std::memory_order_relaxed);

  // ====== 디버그 출력(주기 제한) ======
  if (printer_counter >= print_period) {
  #if RT_printing
      printf("======================================== \n");
      printf("Simulation time : %d ms\n", (int)milisec);
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

      // contact_force (TCP z) 와 같은 부호계로 맞춘 base-frame force 출력
      printf("Contact Fz: %.2f -> Base: %.3f %.3f %.3f\n",
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
      static bool follow_done = false;  // TXT 추종 완료 여부

      // 모드 전환 시 InitMove부터 다시 시작
      if (pre_control_mode != 3) {
          init_done   = false;
          follow_done = false;
      }

      // 1) InitMove 단계
      if (!init_done) {
          bool just_finished = InitMove(dt_s);
          if (!just_finished) {
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          }
          init_done = true;
      }

      // 2) PathFollow 단계 (아직 안 끝났으면 계속)
      if (!follow_done) {
          if (PathFollow(dt_s)) {
              pre_ctrl.store(control_mode, std::memory_order_relaxed);
              return;
          } else {
              // 여기 오면 EOF 도달해서 return_active_ 세팅 끝난 상태
              follow_done = true;
          }
      }

      // 3) Return 단계
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
