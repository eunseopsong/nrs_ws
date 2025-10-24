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


// ===================== Force Callback =====================
void JointControl::FtCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // 1️⃣ TCP 기준의 접촉 힘 (F_TCP = [0, 0, Fz])
    contact_force = msg->data;
    Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);

    // 2️⃣ 현재 조인트 각도로 Inverse 행렬 계산 (TCP → Base)
    Eigen::VectorXd q(6);
    for (int i = 0; i < 6; ++i)
        q(i) = joint_pos[i];   // 최신 조인트 상태 사용

    constexpr double TOOL_Z = 0.248; // EE +Z → TCP offset [m]
    Eigen::Matrix4d T_TCP_base = ur10e_inverse(q, TOOL_Z);
    Eigen::Matrix3d R_TCP_base = T_TCP_base.block<3, 3>(0, 0);

    // 3️⃣ Base 좌표계 기준 힘 계산
    Eigen::Vector3d F_base = R_TCP_base * F_TCP;

    // (선택) F_base 퍼블리시 가능
    // std_msgs::msg::Float64MultiArray F_base_msg;
    // F_base_msg.data = {F_base(0), F_base(1), F_base(2)};
    // UR10_wrench_pub_->publish(F_base_msg);

    // (선택) 디버깅 출력
    // RCLCPP_INFO(node_->get_logger(),
    //             "[FtCallback] TCP Fz=%.3f → Base Fx=%.3f Fy=%.3f Fz=%.3f",
    //             contact_force, F_base(0), F_base(1), F_base(2));
}



// ========================
//  Force Control Function
// ========================
void JointControl::ControlForce()
{
    // --- 1. 기본 설정 ---
    static bool initialized = false;
    static Eigen::VectorXd FC_AC_desX(12);  // [0-5]: pose, [6-11]: desired force
    static Eigen::VectorXd FC_MASS(6), FC_DAMPER(6), FC_STIFFNESS(6);
    static Eigen::VectorXd AC_pose(6);      // Admittance output
    static Eigen::VectorXd prev_pose(6);    // 이전 포즈 저장
    const double dt = 0.001;                // 제어주기 [s]
    constexpr double TOOL_Z = 0.248;        // TCP offset [m]

    // --- 2. 초기화 (한 번만) ---
    if (!initialized) {
        FC_AC_desX.setZero();
        FC_MASS << 1, 1, 1, 0.05, 0.05, 0.05;       // pos(3), ori(3)
        FC_DAMPER << 3000, 3000, 3000, 10, 10, 10;
        FC_STIFFNESS << 2000, 2000, 2000, 20, 20, 20;

        // 현재 EE pose 초기화 (RArm.xc: [x y z r p y])
        for (int i = 0; i < 6; ++i)
            FC_AC_desX(i) = RArm.xc(i);

        prev_pose = RArm.xc;
        initialized = true;
    }

    // --- 3. 힘 센서 데이터 (Base frame 기준) ---
    Eigen::VectorXd q(6);
    for (int i = 0; i < 6; ++i) q(i) = RArm.qc(i);

    Eigen::Matrix4d T_TCP_base = ur10e_inverse(q, TOOL_Z);
    Eigen::Matrix3d R_TCP_base = T_TCP_base.block<3,3>(0,0);
    Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);
    Eigen::Vector3d F_base = R_TCP_base * F_TCP;

    // ft1data (측정된 힘)
    Eigen::VectorXd ft1data(6);
    ft1data << F_base(0), F_base(1), F_base(2), 0.0, 0.0, 0.0;

    // --- 4. Classical Admittance Control ---
    for (int i = 0; i < 6; ++i) {
        double M = FC_MASS(i);
        double B = FC_DAMPER(i);
        double K = FC_STIFFNESS(i);

        // 외력 오차 (Fd - Fext)
        double Fd = (i < 3) ? FC_AC_desX(i + 6) : 0.0;
        double Fext = ft1data(i);
        double F_error = Fd - Fext;

        // 1차 시스템 형태의 admittance 업데이트
        double x = FC_AC_desX(i);      // 이전 목표 (m or rad)
        double x_dot = (x - prev_pose(i)) / dt;
        double x_ddot = (F_error - B * x_dot - K * (x - RArm.xc(i))) / M;
        double new_x = x + x_dot * dt + 0.5 * x_ddot * dt * dt;

        AC_pose(i) = new_x;
    }

    prev_pose = RArm.xc;

    // --- 5. 목표 Pose 업데이트 ---
    // (m → mm 변환)
    Eigen::VectorXd target_pose(6);
    target_pose = AC_pose;
    target_pose(0) *= 1000.0;
    target_pose(1) *= 1000.0;
    target_pose(2) *= 1000.0;

    // --- 6. Inverse Kinematics 수행 ---
    Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_target;
    Eigen::AngleAxisd Rx(target_pose(3), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(target_pose(4), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(target_pose(5), Eigen::Vector3d::UnitZ());
    R_target = Rz * Ry * Rx;
    T_target.block<3,3>(0,0) = R_target;
    T_target(0,3) = target_pose(0);
    T_target(1,3) = target_pose(1);
    T_target(2,3) = target_pose(2);

    Eigen::VectorXd q_target = ur10e_inverse(q, TOOL_Z).col(0); // 단일 해 선택

    // --- 7. 결과 저장 ---
    Desired_XYZ << target_pose(0)/1000.0, target_pose(1)/1000.0, target_pose(2)/1000.0;
    Desired_RPY << target_pose(3), target_pose(4), target_pose(5);
    RArm.qt = q_target;

    // --- 8. 디버깅 출력 ---
#if RT_printing
    printf("[Force Control]\n");
    printf("Fx=%.3f Fy=%.3f Fz=%.3f | Xd: %.3f %.3f %.3f\n",
           F_base(0), F_base(1), F_base(2),
           target_pose(0), target_pose(1), target_pose(2));
#endif
}


bool JointControl::InitMove(double dt_s)
{
    // --- 내부 상태 ---
    static bool active = false;
    static bool initialized = false;
    static double elapsed = 0.0, duration = 0.0;
    static Eigen::Vector3d start_xyz, goal_xyz_raw;
    static Eigen::Matrix3d start_rot, goal_rot;

    constexpr double TOOL_Z = 0.248;

    // --- 최초 진입 시 ---
    if (!initialized) {
        // playback 파일 열기
        if (!Hand_G_playback) {
            auto hand_path = yaml_get_path(NRS_recording, "hand_g_recording", node_->get_logger());
            if (!hand_path.empty())
                Hand_G_playback = std::fopen(hand_path.c_str(), "rt");
        }
        if (!Hand_G_playback) {
            RCLCPP_ERROR(node_->get_logger(), "[PB] playback file not opened.");
            ctrl.store(0, std::memory_order_release);
            return false;
        }

        // 첫 행 읽기
        float x,y,z,r,p,yw,fx,fy,fz;
        char buf[2048];
        while (std::fgets(buf, sizeof(buf), Hand_G_playback)) {
            if (buf[0]=='#' || std::isspace(buf[0])) continue;
            int n = std::sscanf(buf,"%f %f %f %f %f %f %f %f %f",&x,&y,&z,&r,&p,&yw,&fx,&fy,&fz);
            if (n==9) break;
        }

        // init move 설정
        start_xyz << RArm.xc(0), RArm.xc(1), RArm.xc(2);
        goal_xyz_raw << x, y, z;
        start_rot = RArm.Tc.block<3,3>(0,0);
        Eigen::Vector3d goal_rpy(r,p,yw);
        AKin.EulerAngle2Rotation(goal_rot, goal_rpy);

        double v = 0.05;
        double dist = (goal_xyz_raw - start_xyz).norm();
        duration = std::max(3.0, dist / std::max(1e-6, v));
        elapsed = 0.0;

        initialized = true;
        active = true;

        std::rewind(Hand_G_playback);
        set_status(message_status, ST_path_gen_done);
        return true; // 첫 루프는 여기서 종료
    }

    // --- 활성 상태 ---
    if (active) {
        elapsed += dt_s;
        double alpha = std::clamp(elapsed / std::max(1e-6, duration), 0.0, 1.0);

        Eigen::Vector3d xyz_raw = (1.0 - alpha)*start_xyz + alpha*goal_xyz_raw;
        Desired_XYZ = xyz_raw;

        Eigen::Quaterniond q0(start_rot), q1(goal_rot);
        if (q0.dot(q1) < 0.0) q1.coeffs() *= -1.0;
        Eigen::Quaterniond q = q0.slerp(alpha, q1).normalized();
        Eigen::Matrix3d Desired_rot = q.toRotationMatrix();
        Desired_RPY = Desired_rot.eulerAngles(0,1,2);

        Eigen::Vector3d Desired_XYZ_cmd = Desired_XYZ;
        Desired_XYZ_cmd(2) -= TOOL_Z;

        RArm.Td << Desired_rot(0,0),Desired_rot(0,1),Desired_rot(0,2),Desired_XYZ_cmd(0),
                    Desired_rot(1,0),Desired_rot(1,1),Desired_rot(1,2),Desired_XYZ_cmd(1),
                    Desired_rot(2,0),Desired_rot(2,1),Desired_rot(2,2),Desired_XYZ_cmd(2),
                    0,0,0,1;
#if TCP_standard==0
        AKin.InverseK_min(&RArm);
#else
        AKin.Ycontact_InverseK_min(&RArm);
#endif
        joint_state_.header.stamp = node_->now();
        for (int i=0;i<6;i++) joint_state_.position[i] = RArm.qd(i);
        joint_commands_pub_->publish(joint_state_);

        if (alpha >= 1.0 - 1e-6) {
            active = false;
            initialized = true;
            printf("[PB][INITMOVE] done. start following TXT lines.\n");
        }
        return true;
    }
    return false;
}


bool JointControl::PathFollow(double dt_s)
{
    static bool active = true;
    if (!active)
        return false;

    // playback 파일 확인
    if (!Hand_G_playback) {
        RCLCPP_ERROR(node_->get_logger(), "[PB] playback file closed unexpectedly.");
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, "Playback file closed");
        return false;
    }

    // TXT 파일에서 9개 값 읽기 (x y z r p y fx fy fz)
    float X, Y, Z, Roll, Pitch, Yaw, Fx, Fy, Fz;
    int reti = std::fscanf(Hand_G_playback, "%f %f %f %f %f %f %f %f %f",
                           &X, &Y, &Z, &Roll, &Pitch, &Yaw, &Fx, &Fy, &Fz);

    // 파일 끝 (EOF) 처리
    if (reti != 9) {
        std::fclose(Hand_G_playback);
        Hand_G_playback = nullptr;
        active = false;

        // 다음 단계(ReturnHomePose) 준비
        return_active_ = true;
        return_elapsed_ = 0.0;
        return_duration_ = 4.0;
        for (int i = 0; i < 6; ++i)
            return_start_q_(i) = RArm.qc(i);

        printf("[PB] End of file. Start return-to-home.\n");
        return false;
    }

    // ---- Pose 및 Force 데이터 갱신 ----
    Desired_XYZ << (double)X, (double)Y, (double)Z;
    Desired_RPY << (double)Roll, (double)Pitch, (double)Yaw;
    Contact_Rot_force << (double)Fx, (double)Fy, (double)Fz;

    // ---- IK 변환 ----
    Eigen::Matrix3d Desired_rot;
    AKin.EulerAngle2Rotation(Desired_rot, Desired_RPY);

    Eigen::Vector3d Desired_XYZ_cmd = Desired_XYZ;
    Desired_XYZ_cmd(2) -= 0.248; // TOOL_Z 오프셋

    RArm.Td << Desired_rot(0, 0), Desired_rot(0, 1), Desired_rot(0, 2), Desired_XYZ_cmd(0),
                Desired_rot(1, 0), Desired_rot(1, 1), Desired_rot(1, 2), Desired_XYZ_cmd(1),
                Desired_rot(2, 0), Desired_rot(2, 1), Desired_rot(2, 2), Desired_XYZ_cmd(2),
                0, 0, 0, 1;
#if TCP_standard == 0
    AKin.InverseK_min(&RArm);
#else
    AKin.Ycontact_InverseK_min(&RArm);
#endif

    // ---- Joint 명령 퍼블리시 ----
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < 6; ++i)
        joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    return true;
}





bool JointControl::ReturnHomePose(double dt_s)
{
    static bool active = false;
    static double elapsed = 0.0;
    static double duration = 0.0;
    static Vector6d start_q;

    static const Vector6d HOME_Q = (Vector6d() <<
        0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, +M_PI / 2.0, 0.0).finished();

    // --- 홈 복귀 활성화 ---
    if (return_active_ && !active) {
        active = true;
        elapsed = 0.0;
        duration = return_duration_;
        start_q = return_start_q_;
    }

    // --- 홈 복귀 중 ---
    if (active) {
        elapsed += dt_s;
        double alpha = std::clamp(elapsed / std::max(1e-6, duration), 0.0, 1.0);

        Vector6d q_cmd = (1.0 - alpha) * start_q + alpha * HOME_Q;
        for (int i = 0; i < 6; ++i)
            RArm.qd(i) = q_cmd(i);

        joint_state_.header.stamp = node_->now();
        for (int i = 0; i < 6; ++i)
            joint_state_.position[i] = RArm.qd(i);
        joint_commands_pub_->publish(joint_state_);

        // --- 완료 처리 ---
        if (alpha >= 1.0 - 1e-6) {
            active = false;
            return_active_ = false;
            printf("[PB] Return-to-home done.\n");
            ctrl.store(0, std::memory_order_release);
            set_status(message_status, "Playback finished");
        }
        return true;
    }

    // --- 비활성 상태 ---
    return false;
}











// ===================== Main Control Loop =====================
void JointControl::CalculateAndPublishJoint() {
  const double dt_s = 0.1;
  milisec += 10;

  for (int i = 0; i < 6; i++) {
    RArm.ddqd(i) = 0;
    RArm.dqd(i)  = 0;
    RArm.dqc(i)  = 0;
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
  RArm.Td = RArm.Tc;

  VectorXd Init_qc = RArm.qc;
  int path_exe_counter = 0;

  // 상태 로드
  int control_mode = ctrl.load(std::memory_order_relaxed);
  int pre_control_mode = pre_ctrl.load(std::memory_order_relaxed);

  // ---- Base Frame 변환 (TCP → Base) ----
  Eigen::Vector3d F_base;
  {
      Eigen::VectorXd q(6);
      for (int i = 0; i < 6; ++i)
          q(i) = RArm.qc(i);   // 현재 로봇 관절 상태 사용

      constexpr double TOOL_Z = 0.248;  // EE +Z offset [m]
      Eigen::Matrix4d T_TCP_base = ur10e_inverse(q, TOOL_Z);
      Eigen::Matrix3d R_TCP_base = T_TCP_base.block<3,3>(0,0);

      Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);
      F_base = R_TCP_base * F_TCP;
  }

  // ====== Printing ======
  if (printer_counter >= print_period) {
  #if RT_printing
      printf("======================================== \n");
      printf("Now RUNNING MODE(%d), EXTERNAL MODE CMD: %d(%d) (%d/%d) \n",
             Actual_mode, control_mode, pre_control_mode, path_exe_counter, Path_point_num);
      printf("Current status: %s \n", message_status);
      printf("Selected force controller: %d \n", Contact_Fcon_mode);
      printf("milisec: %.2f \n", milisec);
      printf("A_q1: %.3f(%.1f), A_q2: %.3f(%.1f), A_q3: %.3f(%.1f), "
             "A_q4: %.3f(%.1f), A_q5: %.3f(%.1f), A_q6: %.3f(%.1f)\n",
             RArm.qc(0), RArm.qc(0)*(180/PI),
             RArm.qc(1), RArm.qc(1)*(180/PI),
             RArm.qc(2), RArm.qc(2)*(180/PI),
             RArm.qc(3), RArm.qc(3)*(180/PI),
             RArm.qc(4), RArm.qc(4)*(180/PI),
             RArm.qc(5), RArm.qc(5)*(180/PI));

      printf("D_q1: %.3f(%.1f), D_q2: %.3f(%.1f), D_q3: %.3f(%.1f), "
             "D_q4: %.3f(%.1f), D_q5: %.3f(%.1f), D_q6: %.3f(%.1f)\n",
             RArm.qd(0), RArm.qd(0)*(180/PI),
             RArm.qd(1), RArm.qd(1)*(180/PI),
             RArm.qd(2), RArm.qd(2)*(180/PI),
             RArm.qd(3), RArm.qd(3)*(180/PI),
             RArm.qd(4), RArm.qd(4)*(180/PI),
             RArm.qd(5), RArm.qd(5)*(180/PI));

      // Contact force 출력
      printf("Contact force value: %.2f \n", contact_force);
      printf("→ Base Frame Force: CFx=%.3f  CFy=%.3f  CFz=%.3f\n",
             F_base(0), F_base(1), F_base(2));

      printf("Act_XYZ: %.3f %.3f %.3f | Act_RPY: %.3f %.3f %.3f\n",
             RArm.xc(0), RArm.xc(1), RArm.xc(2),
             RArm.thc(0), RArm.thc(1), RArm.thc(2));
      printf("Des_XYZ: %.3f %.3f %.3f | Des_RPY: %.3f %.3f %.3f\n",
             Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2),
             Desired_RPY(0), Desired_RPY(1), Desired_RPY(2));
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



  // ===================================================
  // 3) Posture / Power Playback Control Mode
  // ===================================================
  if (control_mode == 3) {
      // ✅ 안전 초기화
      if (Desired_XYZ.size() != 3) Desired_XYZ.setZero(3);
      if (Desired_RPY.size() != 3) Desired_RPY.setZero(3);
      if (Contact_Rot_force.size() != 3) Contact_Rot_force.setZero(3);

      // ✅ 순서: InitMove → PathFollow → ReturnHomePose
      if (InitMove(dt_s)) {
          pre_ctrl.store(control_mode, std::memory_order_relaxed);
          return;
      }

      if (PathFollow(dt_s)) {
          pre_ctrl.store(control_mode, std::memory_order_relaxed);
          return;
      }

      ReturnHomePose(dt_s);
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
