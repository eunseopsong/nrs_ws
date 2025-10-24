// JointControl.cpp
#include "JointControl.h"


constexpr int DOF = 6;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// ===================== 외부에서 선언/정의된 타입/인스턴스 가정 =====================
// - AKin                              : kinematics helper (ForwardK_T, InverseK_min, Rotation2EulerAngle, …)
// - RArm                              : robot state holder (qc, qd, xc, thc, Tc, Td, …)
// - Posture_PB, Power_PB              : path/Playback 관련 객체 (이번 버전에서는 PTP_* 미사용)
// - J_single, path_planning           : path generator들
// - NRS_recording, NRS_VR_setting     : YAML::Node
// - Contact_Fcon_mode, Playback_mode  : 설정값
// - 다양한 상수/문자열: Hand_guiding_mode, Motion_stop_mode, …
// ================================================================================

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

// ========== 안전한 YAML 경로 취득 & 디렉토리 준비 유틸 ==========

// YAML에서 key에 해당하는 값을 안전하게 파일 경로 문자열로 반환한다.
// - 스칼라면 그대로 사용
// - 시퀀스면 path join
// - 미정의/스칼라 아님/변환 실패면 ""을 반환하고 에러 로그
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

// 주어진 파일 경로의 부모 폴더를 생성(존재하지 않으면)한다.
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

// ===================== JointControl =====================
JointControl::JointControl(const rclcpp::Node::SharedPtr& node)
: node_(node), milisec(0)
{
  AdaptiveK_msg_ = std::make_unique<nrs_msgmonitoring2::MsgMonitoring>(node_, "AdaptiveK_msg");
  FAAC3step_msg_ = std::make_unique<nrs_msgmonitoring2::MsgMonitoring>(node_, "FAAC3step_msg");

  // Publishers
  YSurfN_Fext_pub_   = node_->create_publisher<std_msgs::msg::Float64>("YSurfN_Fext", 20);
  UR10e_mode_pub_    = node_->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 20);
  UR10_pose_pub_     = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_pose", 20);
  UR10_wrench_pub_   = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_wrench", 20);
  // Isaac joint command
  joint_commands_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands" , 20);

  joint_state_.name = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
  joint_state_.position.resize(6, 0.0);

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

  // Timer (100 ms로 동작 가정)
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&JointControl::CalculateAndPublishJoint, this));

  // 파일 핸들/상태 초기화
  if (hand_g_recording) { std::fclose(hand_g_recording); hand_g_recording = nullptr; }
  if (Discre_P_recording){ std::fclose(Discre_P_recording);Discre_P_recording= nullptr; }
  if (VRCali_UR10CB_EE) { std::fclose(VRCali_UR10CB_EE); VRCali_UR10CB_EE = nullptr; }
  if (VRCali_UR10CB_VR) { std::fclose(VRCali_UR10CB_VR); VRCali_UR10CB_VR = nullptr; }
  if (Hand_G_playback)  { std::fclose(Hand_G_playback);  Hand_G_playback  = nullptr; }
  if (path_recording_pos){ std::fclose(path_recording_pos);path_recording_pos= nullptr; }
  if (path_recording_joint){ std::fclose(path_recording_joint); path_recording_joint = nullptr; }
  if (EXPdata1) { std::fclose(EXPdata1); EXPdata1=nullptr; }

  // 디버그/상태
  set_status(message_status, "Motion stop");
  LD_X=LD_Y=LD_Z=LD_Roll=LD_Pitch=LD_Yaw=LD_CFx=LD_CFy=LD_CFz=0.0f;
}

JointControl::~JointControl() {
  if (hand_g_recording) std::fclose(hand_g_recording);
  if (Discre_P_recording) std::fclose(Discre_P_recording);
  if (VRCali_UR10CB_EE) std::fclose(VRCali_UR10CB_EE);
  if (VRCali_UR10CB_VR) std::fclose(VRCali_UR10CB_VR);
  if (Hand_G_playback) std::fclose(Hand_G_playback);
  if (path_recording_pos) std::fclose(path_recording_pos);
  if (path_recording_joint) std::fclose(path_recording_joint);
  if (EXPdata1) std::fclose(EXPdata1);
}

// ===================== Mode Callback =====================
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

      // 안전한 경로 취득 + 디렉토리 준비
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
      Decr_RD_points.bottomRows(1) << VR_CalPoseRPY(0), VR_CalPoseRPY(1), VR_CalPoseRPY(2),
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
      // 경로 파일 검증 + 오픈(재사용 위해 핸들 유지)
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
      if (Hand_G_playback) { std::fclose(Hand_G_playback); Hand_G_playback = nullptr; }
      if (hand_g_recording) { std::fclose(hand_g_recording); hand_g_recording = nullptr; }
      if (path_recording_pos){ std::fclose(path_recording_pos);path_recording_pos= nullptr; }
    }

  } catch (const std::exception& e) {
    RCLCPP_FATAL(node_->get_logger(), "cmdModeCallback exception: %s", e.what());
  }
}

// ===================== Other Callback =====================
void JointControl::PbIterCallback(std_msgs::msg::UInt16::SharedPtr msg) {
  PB_iter_cmd = msg->data; PB_iter_cur = 1; // 1 is right
}

void JointControl::JointCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  mjoint_cmd = msg->data;
  printf("\nSelected joint: %1.0f, Target relative joint angle: %4f \n", mjoint_cmd[0],mjoint_cmd[1]);
  double Tar_pos[] = {0.0};
  double Tar_vel[] = {0.0};
  double Waiting_time[] = {0,0}; // s
  double Vel_set = 0.1; // rad/s

  Tar_pos[0] = std::fabs(mjoint_cmd[1]);
  Tar_vel[0] = (mjoint_cmd[1]>=0) ? Vel_set : -Vel_set;

  Joint_path_start <<RArm.qc(0),RArm.qc(1),RArm.qc(2),RArm.qc(3),RArm.qc(4),RArm.qc(5);
  Path_point_num = J_single.Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));

  if(Path_point_num != -1) {
    ctrl.store(1, std::memory_order_release);
    set_status(message_status, path_gen_done);
    path_done_flag = true;
  }
}

void JointControl::getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (int i = 0; i < 6 && i < (int)msg->position.size(); ++i){
    RArm.qc[i] = msg->position[i];
  }
}

// ===================== UpdateState() =====================
void JointControl::UpdateState()
{
    // 1️⃣ joint_pos → Eigen::VectorXd 변환
    Eigen::VectorXd q(6);
    for (int i = 0; i < 6; ++i)
        q(i) = joint_pos[i];

    // 2️⃣ TCP offset 포함 Forward 계산
    constexpr double TOOL_Z = 0.248;  // [m]
    T_current = ur10e_forward(q, TOOL_Z);

    // 3️⃣ 위치, 자세 계산
    pos_current = T_current.block<3, 1>(0, 3);
    Eigen::Matrix3d R_TCP = T_current.block<3, 3>(0, 0);
    Eigen::Vector3d rpy;
    rpy(0) = std::atan2(R_TCP(2,1), R_TCP(2,2));   // Roll
    rpy(1) = std::asin(-R_TCP(2,0));               // Pitch
    rpy(2) = std::atan2(R_TCP(1,0), R_TCP(0,0));   // Yaw
    rpy_current = rpy;

    // 4️⃣ 조인트 각도 문자열로 변환
    // std::ostringstream q_str;
    // q_str << std::fixed << std::setprecision(3);
    // q_str << "[";
    // for (int i = 0; i < 6; ++i) {
    //     q_str << q(i);
    //     if (i < 5) q_str << ", ";
    // }
    // q_str << "]";

    // 5️⃣ 디버그 출력
    // RCLCPP_INFO(node_->get_logger(),
    //             "[UpdateState] q=%s → TCP Pose: x=%.6f y=%.6f z=%.6f | r=%.6f p=%.6f y=%.6f",
    //             q_str.str().c_str(),
    //             pos_current(0), pos_current(1), pos_current(2),
    //             rpy_current(0), rpy_current(1), rpy_current(2));
}


void JointControl::FtCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // 1️⃣ TCP 기준의 접촉 힘 (F_TCP = [0, 0, Fz])
    contact_force = msg->data;
    Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);

    // 2️⃣ 현재 조인트 각도로 Inverse 행렬 계산 (TCP → Base)
    Eigen::VectorXd q(6);
    for (int i = 0; i < 6; ++i)
        q(i) = joint_pos[i];

    constexpr double TOOL_Z = 0.248; // EE +Z → TCP offset [m]
    Eigen::Matrix4d T_TCP_base = ur10e_inverse(q, TOOL_Z);
    Eigen::Matrix3d R_TCP_base = T_TCP_base.block<3, 3>(0, 0);

    // 3️⃣ Base 좌표계 기준 힘 계산
    Eigen::Vector3d F_base = R_TCP_base * F_TCP;

    // 4️⃣ 디버깅 출력
    // RCLCPP_INFO(node_->get_logger(),
    //             "[FtCallback] F_TCP = [0, 0, %.3f] → F_base = [%.3f, %.3f, %.3f]",
    //             contact_force, F_base(0), F_base(1), F_base(2));

    // (선택) F_base 퍼블리시 가능
    // std_msgs::msg::Float64MultiArray F_base_msg;
    // F_base_msg.data = {F_base(0), F_base(1), F_base(2)};
    // UR10_wrench_pub_->publish(F_base_msg);
}


// ===================== Main Control Loop =====================
void JointControl::CalculateAndPublishJoint() {
  // timer는 100ms 가정
  const double dt_s = 0.1;
  milisec += 10;

  for(int i=0;i<6;i++){
    RArm.ddqd(i) = 0;
    RArm.dqd(i) = 0;
    RArm.dqc(i) = 0;
  }
  RArm.qd = RArm.qc;
  RArm.qt = RArm.qc;

#if TCP_standard == 0
  AKin.ForwardK_T(&RArm); // qc -> Tc, xc
#elif TCP_standard == 1
  AKin.Ycontact_ForwardK_T(&RArm);
#endif

  // 중요: RPY 업데이트
  AKin.Rotation2EulerAngle(&RArm); // Tc -> thc (R,P,Y)
  RArm.Td=RArm.Tc;

  VectorXd Init_qc = RArm.qc;
  int path_exe_counter = 0;

  // 상태 로드
  int control_mode = ctrl.load(std::memory_order_relaxed);
  int pre_control_mode = pre_ctrl.load(std::memory_order_relaxed);

  // ====== Printing ======
  if(printer_counter >= print_period) {
  #if RT_printing
      printf("======================================== \n");
      printf("Now RUNNING MODE(%d), EXTERNAL MODE CMD: %d(%d) (%d/%d) \n",
        Actual_mode,control_mode,pre_control_mode,path_exe_counter,Path_point_num);
      printf("Current status: %s \n",message_status);
      printf("Selected force controller: %d \n",Contact_Fcon_mode);
      printf("milisec: %.2f \n", milisec);
      printf("A_q1: %.3f(%.1f), A_q2: %.3f(%.1f), A_q3: %.3f(%.1f), A_q4: %.3f(%.1f), A_q5: %.3f(%.1f), A_q6: %.3f(%.1f)\n",
        RArm.qc(0),RArm.qc(0)*(180/PI), RArm.qc(1),RArm.qc(1)*(180/PI), RArm.qc(2),RArm.qc(2)*(180/PI),
        RArm.qc(3),RArm.qc(3)*(180/PI), RArm.qc(4),RArm.qc(4)*(180/PI), RArm.qc(5),RArm.qc(5)*(180/PI));
      printf("D_q1: %.3f(%.1f), D_q2: %.3f(%.1f), D_q3: %.3f(%.1f), D_q4: %.3f(%.1f), D_q5: %.3f(%.1f), D_q6: %.3f(%.1f)\n",
        RArm.qd(0),RArm.qd(0)*(180/PI), RArm.qd(1),RArm.qd(1)*(180/PI), RArm.qd(2),RArm.qd(2)*(180/PI),
        RArm.qd(3),RArm.qd(3)*(180/PI), RArm.qd(4),RArm.qd(4)*(180/PI), RArm.qd(5),RArm.qd(5)*(180/PI));

      // FtCallback
      printf("Contact force value: %.2f \n", contact_force);

      // ---- Base Frame 변환 (TCP → Base) ----
      {
          Eigen::VectorXd q(6);
          for (int i = 0; i < 6; ++i)
              q(i) = RArm.qc(i);

          constexpr double TOOL_Z = 0.248;  // EE +Z offset [m]
          Eigen::Matrix4d T_TCP_base = ur10e_inverse(q, TOOL_Z);
          Eigen::Matrix3d R_TCP_base = T_TCP_base.block<3,3>(0,0);

          Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);
          Eigen::Vector3d F_base = R_TCP_base * F_TCP;

          printf("→ Base Frame Force: CFx=%.3f  CFy=%.3f  CFz=%.3f\n",
                 F_base(0), F_base(1), F_base(2));
      }

      //// printf("HFx: %.2f, HFy: %.2f, HFz: %.2f | CFx: %.2f, CFy: %.2f, CFz: %.2f \n",
      //// ftS1(0),ftS1(1),ftS1(2), ftS2(0),ftS2(1),ftS2(2));
      printf("Act_XYZ: %.3f %.3f %.3f | Act_RPY: %.3f %.3f %.3f\n",
        RArm.xc(0),RArm.xc(1),RArm.xc(2), RArm.thc(0),RArm.thc(1),RArm.thc(2));
      printf("Des_XYZ: %.3f %.3f %.3f | Des_RPY: %.3f %.3f %.3f\n",
        Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2), Desired_RPY(0), Desired_RPY(1), Desired_RPY(2));
  #endif
    printer_counter = 0;
  } else {
    printer_counter++;
  }

  // ====== 토픽 퍼블리시 ======
  UR10_pose_msg_.data.clear();
  UR10_wrench_msg_.data.clear();
  for(int i=0; i<6; i++){
    if (i<3) UR10_pose_msg_.data.push_back(RArm.xc(i));
    else     UR10_pose_msg_.data.push_back(RArm.thc(i-3));
    UR10_wrench_msg_.data.push_back(ftS2(i));
  }
  UR10_pose_pub_->publish(UR10_pose_msg_);
  UR10_wrench_pub_->publish(UR10_wrench_msg_);

  // * ====== Control modes ====== *//

  // 0) Initial state (hold the fixed home pose: 0 -90 -90 -90 90 0 deg)
  if (control_mode == 0) {
      speedmode = 0;
      // Fixed home pose in radians
      static const double HOME_Q[6] = { 0.0, -M_PI/2.0, -M_PI/2.0, -M_PI/2.0, +M_PI/2.0, 0.0 };
      // static const double HOME_Q[6] = {2.584298, -1.113670, -1.044218, -0.983704, 1.604492, -3.141593 };

      // Command the home pose every cycle
      for (int i = 0; i < 6; ++i) { 
          RArm.qd(i) = HOME_Q[i]; 
          joint_pos[i] = HOME_Q[i];      // ✅ UpdateState에서 사용할 현재 조인트도 갱신
      }

      RArm.qt = RArm.qd;
      RArm.dqc << 0, 0, 0, 0, 0, 0;
      pause_cnt = 0;

      // ✅ UpdateState() 호출 (현재 홈자세로 FK 계산)
      // UpdateState();

      // Publish to Isaac
      joint_state_.header.stamp = node_->now();
      joint_state_.position.resize(6);
      for (int i = 0; i < 6; ++i) { 
          joint_state_.position[i] = RArm.qd(i); 
      }
      joint_commands_pub_->publish(joint_state_);

      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
  }


  // 1) Cartesian position control mode
  if (control_mode == 1) {
    if(path_done_flag == true) {
      if(path_exe_counter<Path_point_num) {
        /* The case of joint position control */
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
        /***** The case of EE posture control *****/
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
          AKin.InverseK_min(&RArm); // input: Td , output : qd
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
    for (int i = 0; i < 6; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 2) Hand-guiding control mode
  if (control_mode == 2) {
    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

// 3) Posture/Power playback control mode
if (control_mode == 3) {
  // ====== 이 블록 안에서만 유지되는 상태들 (콜백 간 유지) ======
  static bool pb_inited = false;            // ctrl==3 최초 진입 처리용
  static bool initmove_active = false;      // InitMove 단계 활성화
  static double initmove_elapsed = 0.0;     // [s]
  static double initmove_duration = 0.0;    // [s]
  static Eigen::Vector3d init_start_xyz, init_goal_xyz_raw; // 🔸 raw(txt) 기준 목표
  static Eigen::Matrix3d init_start_rot, init_goal_rot;

  // 🔧 스핀들 보정값 (TCP가 실제로 Z축으로 이만큼 내려가 있음)
  constexpr double TOOL_Z = -0.248; // [m] contact_force = 19N (success in flat surface)


  // ====== (추가) 재생 종료 후 홈 복귀 상태 ======
  static bool return_active = false;            // TXT 끝난 뒤 홈 복귀 중?
  static double return_elapsed = 0.0;           // [s]
  static double return_duration = 0.0;          // [s]
  static Vector6d return_start_q;               // 시작 관절각
  static const Vector6d HOME_Q = (Vector6d() <<
      0.0, -M_PI/2.0, -M_PI/2.0, -M_PI/2.0, +M_PI/2.0, 0.0).finished(); // 0 -90 -90 -90 90 0 [rad]

  // ====== 최초 진입시 설정 (pre_control_mode != control_mode) ======
  if (pre_control_mode != control_mode) {
    pb_inited = true;
    initmove_active = false;
    initmove_elapsed = 0.0;
    initmove_duration = 0.0;

    return_active = false;       // 복귀 상태 초기화
    return_elapsed = 0.0;
    return_duration = 0.0;

    // 파일 핸들 확인
    if (!Hand_G_playback) {
      auto hand_path = yaml_get_path(NRS_recording, "hand_g_recording", node_->get_logger());
      if (!hand_path.empty())
        Hand_G_playback = std::fopen(hand_path.c_str(), "rt");
    }
    if (!Hand_G_playback) {
      RCLCPP_ERROR(node_->get_logger(), "[PB] playback file not opened.");
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, "Playback file not opened");
      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
    }

    // 파일에서 **첫 유효 라인(9 float)** 읽기
    auto read_first_valid_9f = [&](FILE* fp, float& x,float& y,float& z,
                                   float& r,float& p,float& yaw,
                                   float& fx,float& fy,float& fz)->bool {
      std::rewind(fp);
      char buf[2048];
      while (std::fgets(buf, sizeof(buf), fp)) {
        bool only_space=true;
        for (char* t=buf; *t; ++t){
          if(!std::isspace((unsigned char)*t)){ only_space=false; break; }
        }
        if (only_space || buf[0]=='#') continue;

        float tx,ty,tz,tr,tp,tw,tfx,tfy,tfz;
        int n = std::sscanf(buf, " %f %f %f %f %f %f %f %f %f ",
                            &tx,&ty,&tz,&tr,&tp,&tw,&tfx,&tfy,&tfz);
        if (n==9){
          x=tx; y=ty; z=tz; r=tr; p=tp; yaw=tw; fx=tfx; fy=tfy; fz=tfz;
          return true;
        }
      }
      return false;
    };

    float fx,fy,fz, x,y,z,r,p,yw;
    if (!read_first_valid_9f(Hand_G_playback, x,y,z, r,p,yw, fx,fy,fz)) {
      RCLCPP_ERROR(node_->get_logger(), "[PB] no valid first line in txt.");
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, "Playback txt invalid");
      pre_ctrl.store(control_mode, std::memory_order_relaxed);
      return;
    }

    // InitMove 목표/시작 설정
    init_start_xyz = Eigen::Vector3d(RArm.xc(0), RArm.xc(1), RArm.xc(2));
    init_goal_xyz_raw  = Eigen::Vector3d((double)x,(double)y,(double)z); // 🔸 raw(txt)

    // 회전행렬 준비 (start는 현재 Tc에서, goal은 RPY로부터)
    init_start_rot = RArm.Tc.block<3,3>(0,0);
    Eigen::Vector3d goal_rpy; goal_rpy << (double)r, (double)p, (double)yw;
    AKin.EulerAngle2Rotation(init_goal_rot, goal_rpy);

    // 이동시간 계산 (선형거리/속도, 최소 3초)
    double Linear_travel_vel = 0.05; // m/s
    double dist = (init_goal_xyz_raw - init_start_xyz).norm(); // 🔸 raw 기준 거리
    initmove_duration = std::max(3.0, dist / std::max(1e-6, Linear_travel_vel));
    initmove_active = true;
    initmove_elapsed = 0.0;

#if Adm_mode == 1
    for(int i=0;i<6;i++) {
      // Isaac에서는 접촉모멘트 원자료가 없으므로 0 사용
      if(i<3) Cadmit_playback[i].adm_1D_init((double)0, 0.0, dt_s);
      else    Cadmit_playback[i].adm_1D_init((double)0, 0.0, dt_s);
    }
#endif

    // Contact region 파라미터 초기화
    KdToZero_flag = false;
    ZeroToKd_flag = false;
    KTZ_Fd_flag = false;
    Kd_change_dist = 0.005;       // Kd changed distance, Unit: m
    KTZ_Kd_init = Power_PB.PRamK[2]; // Save the initial contact stiffness
    KTZ_update_par = Power_PB.Ts/2;  // Kd to Z, Z to Kd upadte parameter (half-Ts is proper)
    KTZ_Kd_threshold = 3;
    KTZ_Kd_h = KTZ_Kd_init;

    // Contact region 계산 (discrete 기록 파일 사용)
    {
      auto Discre_P_recording_path = yaml_get_path(NRS_recording, "Discre_P_recording", node_->get_logger());
      if (!Discre_P_recording_path.empty()) {
        FILE* fp_cr = std::fopen(Discre_P_recording_path.c_str(),"rt");
        if (fp_cr) {
          int CR_reti = 0;
          int CR_reti_counter = 0;
          double CR_LD_histoty[100] = {0,};
          while(true) {
            CR_reti = std::fscanf(fp_cr, "%f %f %f %f %f %f %f\n",
                      &LD_X, &LD_Y, &LD_Z, &LD_Roll, &LD_Pitch, &LD_Yaw, &LD_resi);
            if (CR_reti == EOF || CR_reti == -1) break;
            CR_LD_histoty[CR_reti_counter] = LD_Z;
            if(CR_reti_counter == 1) {CR_start<< LD_X,LD_Y,LD_Z;}
            CR_reti_counter++;
            if (CR_reti_counter >= 100) break;
          }
          std::fclose(fp_cr);
          if (CR_reti_counter >= 3) {
            CR_startZP = CR_LD_histoty[1];
            CR_endZP = CR_LD_histoty[CR_reti_counter-3]; // Do not change the num 3!!
          } else {
            CR_startZP = init_goal_xyz_raw(2);
            CR_endZP = init_goal_xyz_raw(2);
          }
          printf("CR_startZP : %f \n",CR_startZP);
          printf("CR_endZP   : %f \n",CR_endZP);
        }
      }
    }

    // DS / Fuzzy / 3-step FAAC 보조 변수 초기화
    if(Contact_Fcon_mode == 1) {
      DB_AVA_phi = 0;
      DB_AVA_Dd_init = Power_PB.PRamD[2]; // Save the initial Z-damping
      DB_AVA_Xc = Power_PB.PXc_0(2);
      DB_AVA_Xc_pre = Power_PB.PXc_0(2);
      DB_AVA_Xc_dot = 0;
      // Step 0 : Update rate calculation of damping variation
      DB_AVA_sigma = DB_AVA_Rd*(dt_s*Power_PB.PRamD[2])/((double)1.0 + dt_s*Power_PB.PRamD[2]);
    }
    else if(Contact_Fcon_mode == 2) {
      DB_AVA_phi = 0;
      DB_AVA_Kd_init = Power_PB.PRamK[2]; // Save the initial Z-stiffness
      DB_AVA_Xc = Power_PB.PXc_0(2);
      DB_AVA_Xr = Power_PB.PXc_0(2);
      DB_AVA_sigma = 0.5;
    }
    else if(Contact_Fcon_mode == 3) {
      DB_AVA_phi = 0;
      DB_AVA_Kd_init = Power_PB.PRamK[2];
      DB_AVA_Xc = Power_PB.PXc_0(2);
      DB_AVA_Xr = Power_PB.PXc_0(2);
      Fuzzy_F_error = 0;
      Fuzzy_F_Perror = 0;
      Fuzzy_F_error_dot = 0;
      DB_AVA_sigma = 0.5; // Sigma initialization
    }
    else if(Contact_Fcon_mode == 4) {
      DB_AVA_Xc = Power_PB.PXc_0(2);
      DB_AVA_Xr = Power_PB.PXc_0(2);
      DB_AVA_X  = Power_PB.PXc_0(2);
      FAAC3step.FAAC_Init();
    }

    // 디버그 파일 (선택)
    auto test_path = yaml_get_path(NRS_recording, "test_path", node_->get_logger());
    if (path_recording_pos) { std::fclose(path_recording_pos); path_recording_pos = nullptr; }
    if (!test_path.empty()) {
      ensure_parent_dir(test_path, node_->get_logger());
      path_recording_pos = std::fopen(test_path.c_str(), "wt");
      if (!path_recording_pos) {
        RCLCPP_WARN(node_->get_logger(), "open for write failed: '%s' (%s)",
                    test_path.c_str(), std::strerror(errno));
      }
    }

    set_status(message_status, ST_path_gen_done);
  }

  // ====== 실행 ======
  if (!pb_inited) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "[PB] not initialized.");
    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // ---------- (추가) TXT 재생 종료 후 '홈 복귀' 처리 ----------
  if (return_active) {
    return_elapsed += dt_s;
    double alpha = std::clamp(return_elapsed / std::max(1e-6, return_duration), 0.0, 1.0);

    // 관절 선형보간 (슬로우 무빙)
    Vector6d q_cmd = (1.0 - alpha) * return_start_q + alpha * HOME_Q;
    for (int i = 0; i < 6; ++i) RArm.qd(i) = q_cmd(i);

    // 퍼블리시
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < 6; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    if (alpha >= 1.0 - 1e-6) {
      return_active = false;
      printf("[PB] Return-to-home done.\n");
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, "Playback finished");
    }

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }
  // -------------------------------------------------------

  // 3-1) InitMove: 현재 → txt 첫 행, (XYZ: 선형보간 / R: SLERP)
  if (initmove_active) {
    initmove_elapsed += dt_s;
    double alpha = std::clamp(initmove_elapsed / std::max(1e-6, initmove_duration), 0.0, 1.0);

    // 위치: 선형 보간 (🔸 raw로 보간하고 로그에도 raw를 사용)
    Eigen::Vector3d xyz_raw = (1.0 - alpha) * init_start_xyz + alpha * init_goal_xyz_raw;
    Desired_XYZ << xyz_raw(0), xyz_raw(1), xyz_raw(2);

    // 자세: 쿼터니언 SLERP (최단 경로)
    Eigen::Quaterniond q0(init_start_rot);
    Eigen::Quaterniond q1(init_goal_rot);
    if (q0.dot(q1) < 0.0) q1.coeffs() *= -1.0; // 최단 경로 강제
    Eigen::Quaterniond q = q0.slerp(alpha, q1).normalized();
    Eigen::Matrix3d Desired_rot = q.toRotationMatrix();

    // (로그/프린트용) 보간된 RPY → Desired_RPY도 동기화 (raw)
    Eigen::Vector3d rpy_print = Desired_rot.eulerAngles(0,1,2);
    Desired_RPY << rpy_print[0], rpy_print[1], rpy_print[2];

    // ✅ 디버깅: TXT 기준 목표값 출력
    printf("[PB][INITMOVE] alpha=%.3f | Des_XYZ(raw):(%.4f %.4f %.4f) Des_RPY(raw):(%.4f %.4f %.4f)\n",
      alpha, Desired_XYZ(0),Desired_XYZ(1),Desired_XYZ(2),
      Desired_RPY(0),Desired_RPY(1),Desired_RPY(2));

    // IK용 명령 좌표 (Z 오프셋 적용)
    Eigen::Vector3d Desired_XYZ_cmd = Desired_XYZ;
    Desired_XYZ_cmd(2) -= TOOL_Z;

    // IK & 커맨드
    RArm.Td << Desired_rot(0,0),Desired_rot(0,1),Desired_rot(0,2),Desired_XYZ_cmd(0),
                Desired_rot(1,0),Desired_rot(1,1),Desired_rot(1,2),Desired_XYZ_cmd(1),
                Desired_rot(2,0),Desired_rot(2,1),Desired_rot(2,2),Desired_XYZ_cmd(2),
                0,0,0,1;
#if TCP_standard == 0
    AKin.InverseK_min(&RArm);
#else
    AKin.Ycontact_InverseK_min(&RArm);
#endif
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < 6; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    // 종료 판정
    if (alpha >= 1.0 - 1e-6) {
      initmove_active = false;
      initmove_elapsed = 0.0;
      // 본격 추종을 위해 파일을 처음으로 되감기
      std::rewind(Hand_G_playback);
      printf("[PB][INITMOVE] done. start following TXT lines.\n");
    }

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // 3-2) TXT 라인 추종 (라인 단위로 그대로 명령)
  if (!Hand_G_playback) {
    RCLCPP_ERROR(node_->get_logger(), "[PB] playback file closed unexpectedly.");
    ctrl.store(0, std::memory_order_release);
    set_status(message_status, "Playback file closed");
    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  int reti = std::fscanf(Hand_G_playback, "%f %f %f %f %f %f %f %f %f",
                         &LD_X,&LD_Y,&LD_Z,&LD_Roll,&LD_Pitch,&LD_Yaw,
                         &LD_CFx,&LD_CFy,&LD_CFz);
  if (reti != 9) {
    // 파일 끝 or 에러 → 홈 복귀 시퀀스 시작
    if (Hand_G_playback) { std::fclose(Hand_G_playback); Hand_G_playback = nullptr; }
    printf("[PB] End of file (reti=%d). Start return-to-home.\n", reti);

    return_active   = true;
    return_elapsed  = 0.0;
    return_duration = 4.0;            // 천천히 4초에 걸쳐 복귀 (필요시 조정)
    for (int i = 0; i < 6; ++i) return_start_q(i) = RArm.qc(i);

    pre_ctrl.store(control_mode, std::memory_order_relaxed);
    return;
  }

  // TXT의 1~3열은 위치, 4~6열은 RPY → 그대로 반영 (🔸 raw 저장: 로그용/모니터링용)
  Desired_XYZ << (double)LD_X, (double)LD_Y, (double)LD_Z;
  Desired_RPY << (double)LD_Roll, (double)LD_Pitch, (double)LD_Yaw;

  // ✅ 디버깅: 현재 TXT 행의 목표값을 즉시 출력
  // printf("Des_XYZ(raw): %.3f %.3f %.3f | Des_RPY(raw): %.3f %.3f %.3f\n",
  //        Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2),
  //        Desired_RPY(0), Desired_RPY(1), Desired_RPY(2));

  // Isaac에서는 접촉 힘을 Fz 단일 값으로 사용: 나머지 성분은 0
  Contact_Rot_force(0) = 0.0;
  Contact_Rot_force(1) = 0.0;
  Contact_Rot_force(2) = contact_force; // <- /contact/force_magnitude

  Contact_Rot_moment(0) = 0.0;
  Contact_Rot_moment(1) = 0.0;
  Contact_Rot_moment(2) = 0.0;

  // (필요: RPY → 회전행렬)
  Eigen::Matrix3d Desired_rot;
  AKin.EulerAngle2Rotation(Desired_rot, Desired_RPY);

  // ===== Playback_mode / Contact_Fcon_mode 분기 (원본 스타일) =====
#if Playback_mode == 0
  // --- Position playback mode (+ 1D admittance) ---
  for(int i=0;i<6;i++) {
    if(i<3) Desired_XYZ(i)   = Cadmit_playback[i].adm_1D_control((double)Desired_XYZ(i),   (double)0, (double)Contact_Rot_force(i));
    else    Desired_RPY(i-3) = Cadmit_playback[i].adm_1D_control((double)Desired_RPY(i-3), (double)0, (double)Contact_Rot_moment(i-3));
  }
  // RPY가 변했을 수 있으니 다시 회전행렬 계산
  AKin.EulerAngle2Rotation(Desired_rot, Desired_RPY);

#elif Playback_mode == 1
  // --- Power playback mode ---
  if((Contact_Fcon_mode == 0) || (Contact_Fcon_mode == 1)) {
    // ===== (예시) 가변 강성 Kd->0 / 0->Kd 로직 (원본 방식)
#if 1
    // For 2024 NIST 스타일 유지
    if(KdToZero_flag == true) {
      CR_start_dist = RArm.xc-CR_start;
      if(CR_start_dist.norm() <= fabs(Kd_change_dist)) {
        Power_PB.PRamK[2] = Power_PB.PRamK[2]*exp(-KTZ_update_par* (CR_start_dist.norm()+fabs(Kd_change_dist))/(CR_start_dist.norm()));
      }
      if((Power_PB.PRamK[2] <= KTZ_Kd_threshold)) {
        Power_PB.PRamK[2] = 0;
        KTZ_Kd_h = KTZ_Kd_init;
        KdToZero_flag = false;
        ZeroToKd_flag = true;
        KTZ_Fd_flag = false;
      }
    }
    if((KTZ_Fd_flag == false) && (PPB_RTinput.PFd >= 0.01)) {KTZ_Fd_flag = true;}
    if((ZeroToKd_flag == true) && (PPB_RTinput.PFd < 0.01) && (KTZ_Fd_flag == true)) {
      CR_start_dist = RArm.xc-CR_start;
      if(CR_start_dist.norm() <= fabs(Kd_change_dist)) {
        KTZ_Kd_h = KTZ_Kd_h*exp(-KTZ_update_par* CR_start_dist.norm()/(CR_start_dist.norm()+fabs(Kd_change_dist)));
        Power_PB.PRamK[2] = KTZ_Kd_init - KTZ_Kd_h;
      }
      if(fabs(KTZ_Kd_init-Power_PB.PRamK[2]) <= KTZ_Kd_threshold) {
        Power_PB.PRamK[2] = KTZ_Kd_init;
        KdToZero_flag = false;
        ZeroToKd_flag = false;
        KTZ_Fd_flag = false;
      }
    }
#endif
    // ===== (Contact_Fcon_mode == 1) DS 기반 가변 감쇠 =====
    if(Contact_Fcon_mode == 1) {
      DB_AVA_Xc = Power_PB.PU3.transpose()*Power_PB.PXc_0;
      DB_AVA_Xc_dot = (DB_AVA_Xc - DB_AVA_Xc_pre)/dt_s;
      DB_AVA_Xc_pre = DB_AVA_Xc;
      if(PPB_RTinput.PFd >= 0.01) {
        DB_AVA_phi += DB_AVA_sigma*(PPB_RTinput.PFd - PPB_surfN_Fext)/DB_AVA_Dd_init;
        if (fabs(DB_AVA_Xc_dot) > 1e-9)
          Power_PB.PRamD[2] = DB_AVA_Dd_init + DB_AVA_phi*DB_AVA_Dd_init/DB_AVA_Xc_dot;

        if(Power_PB.PRamD[2] >= DB_AVA_Dsature[1]) Power_PB.PRamD[2] = DB_AVA_Dsature[1];
        else if(Power_PB.PRamD[2] <= DB_AVA_Dsature[0]) Power_PB.PRamD[2] = DB_AVA_Dsature[0];
      } else {
        Power_PB.PRamD[2] = DB_AVA_Dd_init;
      }
    }
  }
  else if(Contact_Fcon_mode == 2) {
    // DS 기반 가변 강성
    DB_AVA_Xc = Power_PB.PU3.transpose()*Power_PB.PXc_0;
    DB_AVA_Xr = Power_PB.PU3.transpose()*Desired_XYZ;    // 🔸 raw 사용 (원래 코드 유지)
    if((PPB_RTinput.PFd >= 0.01) || (Contact_Rot_force.norm() >= 2.0)) {
      DB_AVA_phi += DB_AVA_sigma*(PPB_RTinput.PFd - PPB_surfN_Fext)/DB_AVA_Kd_init;
      if (fabs(DB_AVA_Xc - DB_AVA_Xr) > 1e-9)
        Power_PB.PRamK[2] = DB_AVA_Kd_init + DB_AVA_phi*DB_AVA_Kd_init/(DB_AVA_Xc - DB_AVA_Xr);

      if(Power_PB.PRamK[2] >= DB_AVA_Ksature[1]) Power_PB.PRamK[2] = DB_AVA_Ksature[1];
      else if(Power_PB.PRamK[2] <= DB_AVA_Ksature[0]) Power_PB.PRamK[2] = DB_AVA_Ksature[0];
    }

    // 모니터링 메시지 (원본 스타일)
    std::vector<double> Mon1_input_data = {Power_PB.PRamM[2], Power_PB.PRamD[2], Power_PB.PRamK[2]};
    std::vector<double> Mon2_input_data = {PPB_RTinput.PFd, PPB_surfN_Fext};
    std::string Mon1_description = "Mass, Damping, Stiffness";
    std::string Mon2_description = "Contact force, Surface normal force";
    AdaptiveK_msg_->Mon1_publish(Mon1_input_data,Mon1_description,true);
    AdaptiveK_msg_->Mon2_publish(Mon2_input_data,Mon2_description,true);
  }
  else if(Contact_Fcon_mode == 3) {
    // 퍼지 기반 가변 강성
    DB_AVA_Xc = Power_PB.PU3.transpose()*Power_PB.PXc_0; // Surf. normal
    DB_AVA_Xr = Power_PB.PU3.transpose()*Desired_XYZ;    // Surf. normal (raw 유지)
    Fuzzy_F_error = PPB_RTinput.PFd - PPB_surfN_Fext;
    Fuzzy_F_error_dot = (Fuzzy_F_error - Fuzzy_F_Perror)/dt_s;
    Fuzzy_F_Perror = Fuzzy_F_error;

    if((PPB_RTinput.PFd >= 0.01) || (Contact_Rot_force.norm() >= 2.0)) {
      DB_AVA_sigma += FAK.FAAC_DelSigma_Cal(Fuzzy_F_error,Fuzzy_F_error_dot);
      DB_AVA_Kd_init += FAK.FAAC_DelK_Cal(Fuzzy_F_error,Fuzzy_F_error_dot);
      // saturation
      if(DB_AVA_Kd_init >= DB_AVA_Ksature[1]) DB_AVA_Kd_init = DB_AVA_Ksature[1];
      else if(DB_AVA_Kd_init <= 10.0)         DB_AVA_Kd_init = 10.0;
      if(DB_AVA_sigma >= 1.0) DB_AVA_sigma = 1.0;
      else if(DB_AVA_sigma <= 0.001) DB_AVA_sigma = 0.001;

      DB_AVA_phi += DB_AVA_sigma*(PPB_RTinput.PFd - PPB_surfN_Fext)/DB_AVA_Kd_init;
      if (fabs(DB_AVA_Xc - DB_AVA_Xr) > 1e-9)
        Power_PB.PRamK[2] = DB_AVA_Kd_init + DB_AVA_phi*DB_AVA_Kd_init/(DB_AVA_Xc - DB_AVA_Xr);

      if(Power_PB.PRamK[2] >= DB_AVA_Ksature[1]) Power_PB.PRamK[2] = DB_AVA_Ksature[1];
      else if(Power_PB.PRamK[2] <= DB_AVA_Ksature[0]) Power_PB.PRamK[2] = DB_AVA_Ksature[0];
    }
  }
  else if(Contact_Fcon_mode == 4) {
    // Three-Step FAAC (MDK)
    DB_AVA_Xc = Power_PB.PU3.transpose()*Power_PB.PXc_0; // Surf. normal
    DB_AVA_Xr = Power_PB.PU3.transpose()*Desired_XYZ;    // Surf. normal (raw 유지)
    DB_AVA_X  = Power_PB.PU3.transpose()*PPB_RTinput.PX; // Surf. normal

    auto TSFAAC_MDK = FAAC3step.FAAC_MDKob_RUN(Power_PB.PTankE, PPB_surfN_Fext, PPB_RTinput.PFd, DB_AVA_Xc, DB_AVA_X);
    Power_PB.PRamM[2] = TSFAAC_MDK.Mass;
    Power_PB.PRamD[2] = TSFAAC_MDK.Damping;
    Power_PB.PRamK[2] = TSFAAC_MDK.Stiffness;

    if(PPB_RTinput.PFd >= 0.01) {
      std::vector<double> Mon1_input_data = {TSFAAC_MDK.Mass, TSFAAC_MDK.Damping, TSFAAC_MDK.Stiffness};
      std::vector<double> Mon2_input_data = {FAAC3step.FAAC_Ferr, FAAC3step.FAAC_FDot_Err, FAAC3step.FAAC_Epsilon, FAAC3step.FAAC_XeSTD};
      std::vector<double> Mon3_input_data = {PPB_RTinput.PFd, PPB_surfN_Fext};
      std::string Mon1_description = "Mass, Damping, Stiffness";
      std::string Mon2_description = "Force error, Force error dot, Epsilon, XeSTD";
      std::string Mon3_description = "Contact force, Surface normal force";
      FAAC3step_msg_->Mon1_publish(Mon1_input_data,Mon1_description,true);
      FAAC3step_msg_->Mon2_publish(Mon2_input_data,Mon2_description,true);
      FAAC3step_msg_->Mon3_publish(Mon3_input_data,Mon3_description,true);
    }
  }
  else {
    printf("Wrong control mode was selected \n");
    printf("Check the 'NRS_Fcon_setting.yaml' \n");
    //// raiseFlag(0); // Stop the servo & program
  }
#endif // Playback_mode

  // ===== IK & 커맨드 (원본 흐름 유지 + Z 보정만 적용) =====
  // IK용 명령 좌표 (Z 오프셋 적용)
  Eigen::Vector3d Desired_XYZ_cmd = Desired_XYZ;
  Desired_XYZ_cmd(2) -= TOOL_Z;

  RArm.Td << Desired_rot(0,0),Desired_rot(0,1),Desired_rot(0,2),Desired_XYZ_cmd(0),
              Desired_rot(1,0),Desired_rot(1,1),Desired_rot(1,2),Desired_XYZ_cmd(1),
              Desired_rot(2,0),Desired_rot(2,1),Desired_rot(2,2),Desired_XYZ_cmd(2),
              0,0,0,1;
#if TCP_standard == 0
  AKin.InverseK_min(&RArm);
#else
  AKin.Ycontact_InverseK_min(&RArm);
#endif

  // (선택) 기록
  if (path_recording_pos) {
    // fprintf(path_recording_pos,"%10f %10f %10f %10f %10f %10f\n",
    // PPB_RTinput.PFd, (double)ftS2(2),
    // Power_PB.PTankE, Desired_rot(0,2), Desired_rot(1,2), Desired_rot(2,2));
  }

  // 퍼블리시
  joint_state_.header.stamp = node_->now();
  for (int i = 0; i < 6; ++i) joint_state_.position[i] = RArm.qd(i);
  joint_commands_pub_->publish(joint_state_);

  pre_ctrl.store(control_mode, std::memory_order_relaxed);
  return;
}





  // 그 외
  speedmode = 0;
  RArm.qd = RArm.qc;
  RArm.qt = RArm.qc;
  RArm.dqc << 0,0,0,0,0,0;
  pause_cnt=0;

  pre_ctrl.store(control_mode, std::memory_order_relaxed);
}
