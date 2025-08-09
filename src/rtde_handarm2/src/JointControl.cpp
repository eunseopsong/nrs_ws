#include "JointControl.h"
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

constexpr int DOF = 6;

//========================
// 전역 동시성 보호 오브젝트
//========================
static std::mutex g_cmdmode_mtx;  // cmdModeCallback 재진입 방지
static std::mutex g_io_mtx;       // 파일 IO 전용 락

// 안전한 status 설정
template <size_t N>
static inline void set_status(char (&dst)[N], const char* s) {
  std::snprintf(dst, N, "%s", s ? s : "");
}

// 공백/CRLF 트리밍
static std::string trim_path(std::string s) {
  auto notspace = [](unsigned char c){ return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
  while (!s.empty() && (s.back()=='\r' || s.back()=='\n' || std::isspace((unsigned char)s.back()))) {
    s.pop_back();
  }
  return s;
}

// 마지막 유효 9-플로트 라인을 로컬 핸들로 파싱 (공유 핸들 경쟁 회피)
static bool read_last_valid_9f(const std::string& p,
                               float& x,float& y,float& z,
                               float& r,float& pitch,float& yaw,
                               float& fx,float& fy,float& fz) {
  FILE* fp = std::fopen(p.c_str(), "rt");
  if (!fp) return false;
  char buf[2048]; bool got=false;
  while (std::fgets(buf, sizeof(buf), fp)) {
    bool only_space=true;
    for (char* t=buf; *t; ++t){ if(!std::isspace((unsigned char)*t)){ only_space=false; break; } }
    if (only_space || buf[0]=='#') continue;
    float tx,ty,tz,tr,tp,tw,tfx,tfy,tfz;
    int n = std::sscanf(buf, " %f %f %f %f %f %f %f %f %f ",
                        &tx,&ty,&tz,&tr,&tp,&tw,&tfx,&tfy,&tfz);
    if (n==9){ x=tx; y=ty; z=tz; r=tr; pitch=tp; yaw=tw; fx=tfx; fy=tfy; fz=tfz; got=true; }
  }
  std::fclose(fp);
  return got;
}

JointControl::JointControl(const rclcpp::Node::SharedPtr& node)
: node_(node), milisec(0)
{
  AdaptiveK_msg_ = std::make_unique<nrs_msgmonitoring2::MsgMonitoring>(node_, "AdaptiveK_msg");
  FAAC3step_msg_ = std::make_unique<nrs_msgmonitoring2::MsgMonitoring>(node_, "FAAC3step_msg");

  // Publishers
  YSurfN_Fext_pub_ = node_->create_publisher<std_msgs::msg::Float64>("YSurfN_Fext", 20);
  UR10e_mode_pub_  = node_->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 20);
  UR10_pose_pub_   = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_pose", 20);
  UR10_wrench_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_wrench", 20);

  // Isaac sim joint command
  joint_commands_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands" , 20);
  joint_state_.name = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };
  joint_state_.position.resize(6, 0.0);

  // Subscribers
  UR10e_mode_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "Yoon_UR10e_mode", 20,
      std::bind(&JointControl::cmdModeCallback, this, std::placeholders::_1));

  PB_iter_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/Yoon_PbNum_cmd", 100,
      std::bind(&JointControl::PbIterCallback, this, std::placeholders::_1));

  joint_cmd_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/yoon_UR10e_joint_cmd", 100,
      std::bind(&JointControl::JointCmdCallback, this, std::placeholders::_1));

  VR_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vive/pos0", 100,
      std::bind(&JointControl::VRdataCallback, this, std::placeholders::_1));

  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/isaac_joint_states", rclcpp::QoS(10),
      std::bind(&JointControl::getActualQ, this, std::placeholders::_1));

  // Timer
  timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&JointControl::CalculateAndPublishJoint, this));
}

JointControl::~JointControl() {}

//-----------------------------
// Mode command callback
//-----------------------------
void JointControl::cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(g_cmdmode_mtx);

  try {
    mode_cmd = msg->data;
    printf("[DEBUG] cmdModeCallback called. mode_cmd=%u\n", mode_cmd);

    if (!NRS_recording["hand_g_recording"]) {
      RCLCPP_WARN(node_->get_logger(), "YAML key 'hand_g_recording' not found.");
    }

    if (mode_cmd == Joint_control_mode_cmd) {
      // no-op
    }
    else if (mode_cmd == EE_Posture_control_mode_cmd) {
      // no-op
    }
    else if (mode_cmd == Hand_guiding_mode_cmd) {
      ctrl.store(2, std::memory_order_release);
      set_status(message_status, Hand_guiding_mode);
    }
    else if (mode_cmd == Continuous_reording_start) {
      path_recording_flag = true;

      std::string hand_g_recording_path = trim_path(NRS_recording["hand_g_recording"].as<std::string>());
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        if (hand_g_recording) { std::fclose(hand_g_recording); hand_g_recording = nullptr; }
        hand_g_recording = std::fopen(hand_g_recording_path.c_str(), "wt");
      }
      if (!hand_g_recording) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)",
                     hand_g_recording_path.c_str(), errno, std::strerror(errno));
        path_recording_flag = false;
        return;
      }
      set_status(message_status, Data_recording_on);
    }
    else if (mode_cmd == Continusous_recording_end) {
      path_recording_flag = false;
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        if (hand_g_recording) { std::fclose(hand_g_recording); hand_g_recording = nullptr; }
      }
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
      std::string path = trim_path(NRS_recording["Discre_P_recording"].as<std::string>());
      FILE* fp = nullptr;
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        if (Discre_P_recording) { std::fclose(Discre_P_recording); Discre_P_recording = nullptr; }
        fp = std::fopen(path.c_str(), "wt");
        Discre_P_recording = fp;
      }
      if (!fp) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)", path.c_str(), errno, std::strerror(errno));
        return;
      }
      for (int i = 0; i < Num_RD_points; i++) {
        std::fprintf(fp, "%10f %10f %10f %10f %10f %10f %10f\n",
          Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2),
          Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), 0.0);
      }
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        std::fclose(fp);
        Discre_P_recording = nullptr;
      }
      Num_RD_points = 0;
      Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
      Inst_RD_points = Eigen::MatrixXd::Zero(1,6);
      printf("\n Discrete points saved\n");
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
      std::string path = trim_path(NRS_recording["Discre_P_recording"].as<std::string>());
      FILE* fp = nullptr;
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        if (Discre_P_recording) { std::fclose(Discre_P_recording); Discre_P_recording = nullptr; }
        fp = std::fopen(path.c_str(), "wt");
        Discre_P_recording = fp;
      }
      if (!fp) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)", path.c_str(), errno, std::strerror(errno));
        return;
      }
      for (int i = 0; i < Num_RD_points; i++) {
        std::fprintf(fp, "%10f %10f %10f %10f %10f %10f %10f\n",
          Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2),
          Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), 0.0);
      }
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        std::fclose(fp);
        Discre_P_recording = nullptr;
      }
      Num_RD_points = 0;
      Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
      Inst_RD_points = Eigen::MatrixXd::Zero(1,6);
      printf("\n VR teach points saved\n");
    }
    else if (mode_cmd == VRCali_reording_start) {
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

      if (Num_VR_points != 0) {
        Inst_VR_points = Decr_VR_points;
        Decr_VR_points.resize(Num_VR_points+1,7);
        Decr_VR_points.topRows(Num_VR_points) = Inst_VR_points;
      } else {
        Decr_VR_points.topRows(Num_VR_points+1) = Inst_VR_points;
      }
      Decr_VR_points.bottomRows(1) <<
        VR_pose[0],VR_pose[1],VR_pose[2],
        VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6];
      Num_VR_points++;
      std::snprintf(Saved_way_point, sizeof(Saved_way_point), "Saved VR points: %d", Num_VR_points);
      set_status(message_status, Saved_way_point);
      std::cout << "\n" << Decr_VR_points << std::endl;
    }
    else if (mode_cmd == VRCali_recording_save) {
      std::string ee_path = trim_path(NRS_recording["VRCali_UR10CB_EE"].as<std::string>());
      std::string vr_path = trim_path(NRS_recording["VRCali_UR10CB_VR"].as<std::string>());

      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        if (VRCali_UR10CB_EE) { std::fclose(VRCali_UR10CB_EE); VRCali_UR10CB_EE = nullptr; }
        if (VRCali_UR10CB_VR) { std::fclose(VRCali_UR10CB_VR); VRCali_UR10CB_VR = nullptr; }
        VRCali_UR10CB_EE = std::fopen(ee_path.c_str(), "wt");
      }
      if (!VRCali_UR10CB_EE) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)", ee_path.c_str(), errno, std::strerror(errno));
        return;
      }
      for (int i = 0; i < Num_EE_points; i++) {
        std::fprintf(VRCali_UR10CB_EE,
          "%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
          Decr_EE_points(i,0), Decr_EE_points(i,1),  Decr_EE_points(i,2),
          Decr_EE_points(i,3), Decr_EE_points(i,4),  Decr_EE_points(i,5),
          Decr_EE_points(i,6), Decr_EE_points(i,7),  Decr_EE_points(i,8),
          Decr_EE_points(i,9), Decr_EE_points(i,10), Decr_EE_points(i,11));
      }
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        std::fclose(VRCali_UR10CB_EE); VRCali_UR10CB_EE = nullptr;
      }
      Num_EE_points = 0;
      Decr_EE_points = Eigen::MatrixXd::Zero(1,12);
      Inst_EE_points = Eigen::MatrixXd::Zero(1,12);
      printf("\n Discrete EE points saved\n");

      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        VRCali_UR10CB_VR = std::fopen(vr_path.c_str(), "wt");
      }
      if (!VRCali_UR10CB_VR) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)", vr_path.c_str(), errno, std::strerror(errno));
        return;
      }
      for (int i = 0; i < Num_VR_points; i++) {
        std::fprintf(VRCali_UR10CB_VR, "%10f %10f %10f %10f %10f %10f %10f\n",
          Decr_VR_points(i,0), Decr_VR_points(i,1), Decr_VR_points(i,2),
          Decr_VR_points(i,3), Decr_VR_points(i,4), Decr_VR_points(i,5), Decr_VR_points(i,6));
      }
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        std::fclose(VRCali_UR10CB_VR); VRCali_UR10CB_VR = nullptr;
      }
      Num_VR_points = 0;
      Decr_VR_points = Eigen::MatrixXd::Zero(1,7);
      Inst_VR_points = Eigen::MatrixXd::Zero(1,7);
      printf("\n Cali points saved\n");
    }
    else if (mode_cmd == Playback_mode_cmd) {
      std::string hand_path = trim_path(NRS_recording["hand_g_recording"].as<std::string>());

      if (hand_path.empty() || !std::filesystem::exists(hand_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory file not found: '%s'", hand_path.c_str());
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, Motion_stop_mode);
        return;
      }

      float LD_X=0, LD_Y=0, LD_Z=0, LD_Roll=0, LD_Pitch=0, LD_Yaw=0, LD_CFx=0, LD_CFy=0, LD_CFz=0;
      if (!read_last_valid_9f(hand_path, LD_X,LD_Y,LD_Z, LD_Roll,LD_Pitch,LD_Yaw, LD_CFx,LD_CFy,LD_CFz)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse a valid line from '%s'", hand_path.c_str());
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, Motion_stop_mode);
        return;
      }

      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        if (Hand_G_playback) { std::fclose(Hand_G_playback); Hand_G_playback = nullptr; }
        Hand_G_playback = std::fopen(hand_path.c_str(), "rt");
      }
      if (!Hand_G_playback) {
        RCLCPP_ERROR(node_->get_logger(), "open for read failed: '%s' (errno=%d: %s)",
                     hand_path.c_str(), errno, std::strerror(errno));
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, Motion_stop_mode);
        return;
      }

      double Linear_travel_vel  = 0.03;
      double Tar_pos[6]  = {LD_X,LD_Y,LD_Z,LD_Roll,LD_Pitch,LD_Yaw};
      double Init_pos[6] = {RArm.xc(0), RArm.xc(1), RArm.xc(2), RArm.thc(0), RArm.thc(1), RArm.thc(2)};

      double Linear_travel_time =
        std::sqrt(std::pow(Init_pos[0]-Tar_pos[0],2) +
                  std::pow(Init_pos[1]-Tar_pos[1],2) +
                  std::pow(Init_pos[2]-Tar_pos[2],2)) / Linear_travel_vel;
      if (Linear_travel_time < 3) Linear_travel_time = 3;

      PB_starting_path_done_flag = Posture_PB.PTP_6D_path_init(Init_pos, Tar_pos, Linear_travel_time);
      printf("Playback init path generation done\n");

      std::string test_path = trim_path(NRS_recording["test_path"].as<std::string>());
      {
        std::lock_guard<std::mutex> g(g_io_mtx);
        if (path_recording_pos) { std::fclose(path_recording_pos); path_recording_pos = nullptr; }
        path_recording_pos = std::fopen(test_path.c_str(), "wt");
      }
      if (!path_recording_pos) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)",
                     test_path.c_str(), errno, std::strerror(errno));
      }

      set_status(message_status, ST_path_gen_done);
      #if Playback_mode == 1
      Power_PB.playback_init(RArm.xc, RArm.thc);
      #endif

      ctrl.store(3, std::memory_order_release);
    }
    else if (mode_cmd == Motion_stop_cmd) {
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, Motion_stop_mode);
      std::lock_guard<std::mutex> g(g_io_mtx);
      if (Hand_G_playback)   { std::fclose(Hand_G_playback);   Hand_G_playback   = nullptr; }
      if (hand_g_recording)  { std::fclose(hand_g_recording);  hand_g_recording  = nullptr; }
      if (path_recording_pos){ std::fclose(path_recording_pos);path_recording_pos= nullptr; }
      if (path_recording_joint){ std::fclose(path_recording_joint);path_recording_joint=nullptr; }
      if (EXPdata1){ std::fclose(EXPdata1); EXPdata1=nullptr; }
      if (Discre_P_recording){ std::fclose(Discre_P_recording); Discre_P_recording=nullptr; }
    }

  } catch (const std::exception& e) {
    RCLCPP_FATAL(node_->get_logger(), "cmdModeCallback exception: %s", e.what());
  }
}

//-----------------------------
// Misc Callbacks
//-----------------------------
void JointControl::PbIterCallback(std_msgs::msg::UInt16::SharedPtr msg) {
  PB_iter_cmd = msg->data;
  PB_iter_cur = 1; // 1 is right
}

void JointControl::JointCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  mjoint_cmd = msg->data;
  printf("\nSelected joint: %1.0f, Target relative joint angle: %4f \n", mjoint_cmd[0], mjoint_cmd[1]);

  double Tar_pos[] = { std::fabs(mjoint_cmd[1]) };
  double Tar_vel[] = { (mjoint_cmd[1] >= 0) ? 0.1 : -0.1 };
  double Waiting_time[] = {0,0};

  Joint_path_start << RArm.qc(0),RArm.qc(1),RArm.qc(2),RArm.qc(3),RArm.qc(4),RArm.qc(5);

  Path_point_num = J_single.Single_blended_path(Tar_pos, Tar_vel, Waiting_time, (int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));
  if (Path_point_num != -1) {
    ctrl.store(1, std::memory_order_release);
    memcpy(message_status, path_gen_done, sizeof(path_gen_done));
    path_done_flag = true;
  }
}

void JointControl::VRdataCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!VR_yaml_loader) {
    YAML::Node T_AD = NRS_VR_setting["T_AD"];
    for (std::size_t i = 0; i < T_AD.size(); ++i)
      for(std::size_t j = 0; j < T_AD[i].size(); ++j) VR_Cali_TAD(i,j) = T_AD[i][j].as<double>();

    YAML::Node T_BC = NRS_VR_setting["T_BC"];
    for (std::size_t i = 0; i < T_BC.size(); ++i)
      for(std::size_t j = 0; j < T_BC[i].size(); ++j) VR_Cali_TBC(i,j) = T_BC[i][j].as<double>();

    YAML::Node T_BC_inv = NRS_VR_setting["T_BC_inv"];
    for (std::size_t i = 0; i < T_BC_inv.size(); ++i)
      for(std::size_t j = 0; j < T_BC_inv[i].size(); ++j) VR_Cali_TBC_inv(i,j) = T_BC_inv[i][j].as<double>();

    YAML::Node T_BC_PB = NRS_VR_setting["T_BC_PB"];
    for (std::size_t i = 0; i < T_BC_PB.size(); ++i)
      for(std::size_t j = 0; j < T_BC_PB[i].size(); ++j) VR_Cali_TBC_PB(i,j) = T_BC_PB[i][j].as<double>();

    YAML::Node T_CE = NRS_VR_setting["T_CE"];
    for (std::size_t i = 0; i < T_CE.size(); ++i)
      for(std::size_t j = 0; j < T_CE[i].size(); ++j) VR_Cali_TCE(i,j) = T_CE[i][j].as<double>();

    YAML::Node R_Adj = NRS_VR_setting["R_Adj"];
    for (std::size_t i = 0; i < R_Adj.size(); ++i)
      for(std::size_t j = 0; j < R_Adj[i].size(); ++j) VR_Cali_RAdj(i,j) = R_Adj[i][j].as<double>();

    VR_yaml_loader = true;
  }

  // raw pose
  VR_pose[0] = msg->pose.position.x;
  VR_pose[1] = msg->pose.position.y;
  VR_pose[2] = msg->pose.position.z;
  VR_pose[3] = msg->pose.orientation.w;
  VR_pose[4] = msg->pose.orientation.x;
  VR_pose[5] = msg->pose.orientation.y;
  VR_pose[6] = msg->pose.orientation.z;

  // HTM
  VR_Q2Rot = AKin.Qua2Rot(VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6]);
  VR_PoseM.block(0,0,3,3) = VR_Q2Rot;
  VR_PoseM.block(0,3,3,1) << VR_pose[0], VR_pose[1], VR_pose[2];
  VR_PoseM.block(3,0,1,4) << 0.0, 0.0, 0.0, 1.0;

  // Heuristic adj.
  VR_Cali_TAdj.block(0,0,3,3) = AKin.RotZ(0.0)*(AKin.RotX(0.0))*VR_Cali_RAdj.transpose();
  VR_Cali_TAdj.block(0,3,3,1) << 0.0, 0.0, 0.0;
  VR_Cali_TAdj.block(3,0,1,4) << 0.0, 0.0, 0.0, 1.0;
  VR_PoseM = VR_Cali_TAdj*VR_PoseM;

  Quaterniond VR_roted_qua = AKin.Rot2Qua(VR_PoseM.block(0,0,3,3));
  VR_pose[0] = VR_PoseM(0,3);
  VR_pose[1] = VR_PoseM(1,3);
  VR_pose[2] = VR_PoseM(2,3);
  VR_pose[3] = VR_roted_qua.w();
  VR_pose[4] = VR_roted_qua.x();
  VR_pose[5] = VR_roted_qua.y();
  VR_pose[6] = VR_roted_qua.z();

  // to robot base
  VR_CalPoseM = VR_Cali_TAD*VR_PoseM*VR_Cali_TBC_inv*VR_Cali_TBC_PB*VR_Cali_TCE;
  Quaterniond VR_cal_qua = AKin.Rot2Qua(VR_CalPoseM.block(0,0,3,3));
  VR_cal_pose[0] = VR_CalPoseM(0,3);
  VR_cal_pose[1] = VR_CalPoseM(1,3);
  VR_cal_pose[2] = VR_CalPoseM(2,3);
  VR_cal_pose[3] = VR_cal_qua.w();
  VR_cal_pose[4] = VR_cal_qua.x();
  VR_cal_pose[5] = VR_cal_qua.y();
  VR_cal_pose[6] = VR_cal_qua.z();

  VR_CalRPY = AKin.VR_Rot2RPY(VR_CalPoseM.block(0,0,3,3));
  VR_CalPoseRPY << VR_CalPoseM(0,3),VR_CalPoseM(1,3),VR_CalPoseM(2,3),
                    PI+VR_CalRPY(1), -PI-VR_CalRPY(2), -PI/2-VR_CalRPY(0);
}

void JointControl::getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (int i = 0; i < 6; ++i){
    RArm.qc[i] = msg->position[i];
  }
}

//-----------------------------
// Main control loop (timer)
//-----------------------------
void JointControl::CalculateAndPublishJoint()
{
  milisec += 20;

  // init joint states
  for(int i=0;i<6;i++){ RArm.ddqd(i)=0; RArm.dqd(i)=0; RArm.dqc(i)=0; }
  RArm.qd = RArm.qc; RArm.qt = RArm.qc;
  #if TCP_standard == 0
    AKin.ForwardK_T(&RArm);
  #else
    AKin.Ycontact_ForwardK_T(&RArm);
  #endif
  RArm.Td=RArm.Tc;
  VectorXd Init_qc = RArm.qc;
  int path_exe_counter = 0;

  // Hand-guiding 기본 파라미터 예시 (필요 시 조정)
  #if Adm_mode == 0
    Hadmit_M << 0.1,0.1,0.1, 0.005,0.005,0.005;
    Hadmit_D << 0.4,0.4,0.4, 0.02,0.02,0.02;
    Hadmit_K << 5,5,5,1,1,1;
  #else
    Hadmit_M << 6,6,6, 0.4,0.4,0.4;
    Hadmit_D << 50,50,50, 4,4,4;
    Hadmit_K << 0.0,0.0,0.0, 0.0,0.0,0.0;
    for(int i=0;i<6;i++) Hadmit_force[i].adm_1D_MDK((double)Hadmit_M(i),(double)Hadmit_D(i),(double)Hadmit_K(i));
  #endif

  int c  = ctrl.load(std::memory_order_relaxed);
  int pc = pre_ctrl.load(std::memory_order_relaxed);

  // 주기 출력 (요약)
  if(printer_counter >= print_period){
    printf("======================================== \n");
    printf("Now RUNNING MODE(%d), EXTERNAL MODE CMD: %d(%d) (%d/%d) \n",Actual_mode,c,pc,path_exe_counter,Path_point_num);
    printf("Current status: %s \n",message_status);
    printf("Selected force controller: %d \n",Contact_Fcon_mode);
    printf("milisec: %.2f \n", milisec);
    printf("A_q1: %.3f, A_q2: %.3f, A_q3: %.3f, A_q4: %.3f, A_q5: %.3f, A_q6: %.3f\n",
           RArm.qc(0),RArm.qc(1),RArm.qc(2),RArm.qc(3),RArm.qc(4),RArm.qc(5));
    printf("D_q1: %.3f, D_q2: %.3f, D_q3: %.3f, D_q4: %.3f, D_q5: %.3f, D_q6: %.3f\n",
           RArm.qd(0),RArm.qd(1),RArm.qd(2),RArm.qd(3),RArm.qd(4),RArm.qd(5));
    printf("HFx: %.2f, HFy: %.2f, HFz: %.2f | CFx: %.2f, CFy: %.2f, CFz: %.2f \n",
           ftS1(0),ftS1(1),ftS1(2), ftS2(0),ftS2(1),ftS2(2));
    printf("Act_XYZ: %.3f %.3f %.3f | Act_RPY: %.3f %.3f %.3f\n",
           RArm.xc(0),RArm.xc(1),RArm.xc(2), RArm.thc(0),RArm.thc(1),RArm.thc(2));
    printer_counter = 0;
  } else printer_counter++;

  // publish pose/wrench
  UR10_pose_msg_.data.clear(); UR10_wrench_msg_.data.clear();
  for(int i=0;i<6;i++){
    if (i<3) UR10_pose_msg_.data.push_back(RArm.xc(i));
    else     UR10_pose_msg_.data.push_back(RArm.thc(i-3));
    UR10_wrench_msg_.data.push_back(ftS2(i));
  }
  UR10_pose_pub_->publish(UR10_pose_msg_);
  UR10_wrench_pub_->publish(UR10_wrench_msg_);

  // ctrl 분기
  if (ctrl.load(std::memory_order_acquire) == 0) {
    // idle
    RArm.qt = RArm.qc; RArm.dqc.setZero();
    pre_ctrl.store(ctrl.load(std::memory_order_relaxed), std::memory_order_relaxed);
  }
  else if (ctrl.load(std::memory_order_acquire) == 1) {
    // joint path 실행 (일부만 유지)
    if(path_done_flag){
      if(path_exe_counter<Path_point_num){
        if(mode_cmd == Joint_control_mode_cmd){
          RArm.qd(0) = Joint_path_start(0) + ((double)(mjoint_cmd[0]==1))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(1) = Joint_path_start(1) + ((double)(mjoint_cmd[0]==2))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(2) = Joint_path_start(2) + ((double)(mjoint_cmd[0]==3))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(3) = Joint_path_start(3) + ((double)(mjoint_cmd[0]==4))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(4) = Joint_path_start(4) + ((double)(mjoint_cmd[0]==5))*J_single.Final_pos(path_exe_counter,1);
          RArm.qd(5) = Joint_path_start(5) + ((double)(mjoint_cmd[0]==6))*J_single.Final_pos(path_exe_counter,1);

          // joint 기록 (보호)
          std::lock_guard<std::mutex> g(g_io_mtx);
          if (path_recording_joint) {
            std::fprintf(path_recording_joint,"%10f %10f %10f %10f %10f %10f \n",
                         RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5));
          }
        }
        path_exe_counter++;
      } else {
        path_done_flag = false;
        path_exe_counter = 0;
      }
    }
    pre_ctrl.store(ctrl.load(std::memory_order_relaxed), std::memory_order_relaxed);
  }
  else if (ctrl.load(std::memory_order_acquire) == 2) {
    // Hand-guiding (요지 유지, 파일 기록부 보호)
    RArm.qt = RArm.qc; RArm.dqc.setZero();

    // 센서 좌표 변환
    Contact_Rot_force(0)=ftS2(0); Contact_Rot_force(1)=ftS2(1); Contact_Rot_force(2)=ftS2(2);
    Contact_Rot_moment(0)=-ftS2(4); Contact_Rot_moment(1)=ftS2(3); Contact_Rot_moment(2)=ftS2(5);

    // 간소화한 admittance (필요시 상세 로직 유지)
    Hadm_pos_act << RArm.xc(0),RArm.xc(1),RArm.xc(2),RArm.thc(0),RArm.thc(1),RArm.thc(2);
    if(pre_ctrl != ctrl){
      Hadm_pos_cmd = Hadm_pos_act;
      for(int i=0;i<6;i++) Hadmit_force[i].adm_1D_init(0-Hadm_pos_cmd(i),0.0,dt);
    }
    Hadm_FT_data << Contact_Rot_force(0),Contact_Rot_force(1),Contact_Rot_force(2),
                   -ftS2(4), ftS2(3), ftS2(5);

    for(int i=0;i<6;i++){
      Hadmit_force[i].adm_1D_MDK((double)Hadmit_M(i),(double)Hadmit_D(i),(double)Hadmit_K(i));
      Hadm_pos_cmd(i)=Hadmit_force[i].adm_1D_control((double)0,(double)0, Hadm_FT_data(i));
    }

    Desired_XYZ << Hadm_pos_cmd(0),Hadm_pos_cmd(1),Hadm_pos_cmd(2);
    Desired_RPY << Hadm_pos_cmd(3),Hadm_pos_cmd(4),Hadm_pos_cmd(5);
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

    // 기록 (보호)
    if(path_recording_flag == true){
      std::lock_guard<std::mutex> g(g_io_mtx);
      if (hand_g_recording) {
        std::fprintf(hand_g_recording,"%10f %10f %10f %10f %10f %10f %10f %10f %10f \n",
          Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2), Desired_RPY(0), Desired_RPY(1), Desired_RPY(2),
          Contact_Rot_force(0),Contact_Rot_force(1),Contact_Rot_force(2));
      }
    }

    // 명령 publish
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < 6; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);
    pre_ctrl.store(ctrl.load(std::memory_order_relaxed), std::memory_order_relaxed);
  }
  else if (ctrl.load(std::memory_order_acquire) == 3) {
    // Playback (핵심 I/O 보호 추가)
    RArm.qt = RArm.qc; RArm.dqc.setZero();
    Contact_Rot_force << ftS2(0),ftS2(1),ftS2(2);
    Contact_Rot_moment << -ftS2(4),ftS2(3),ftS2(5);

    if(pre_ctrl != ctrl){
      #if Adm_mode == 1
      for(int i=0;i<6;i++){
        if(i<3) Cadmit_playback[i].adm_1D_init((double)0,0-Contact_Rot_force(i),dt);
        else     Cadmit_playback[i].adm_1D_init((double)0,0-Contact_Rot_moment(i-3),dt);
      }
      #endif
      KdToZero_flag=false; ZeroToKd_flag=false; KTZ_Fd_flag=false;
      Kd_change_dist=0.005; KTZ_Kd_init=Power_PB.PRamK[2]; KTZ_update_par=Power_PB.Ts/2; KTZ_Kd_threshold=3; KTZ_Kd_h=KTZ_Kd_init;

      // Contact region 추정용 샘플 로드 (로컬 핸들 사용)
      std::string dpath = trim_path(NRS_recording["Discre_P_recording"].as<std::string>());
      FILE* fp = std::fopen(dpath.c_str(),"rt");
      if (fp){
        int CR_reti=0, CR_reti_counter=0;
        double CR_LD_histoty[100] = {0,};
        while(CR_reti != -1){
          CR_reti = std::fscanf(fp, "%f %f %f %f %f %f %f\n", &LD_X, &LD_Y, &LD_Z, &LD_Roll, &LD_Pitch, &LD_Yaw, &LD_resi);
          if (CR_reti == 7){
            CR_LD_histoty[CR_reti_counter] = LD_Z;
            if(CR_reti_counter == 1) {CR_start<< LD_X,LD_Y,LD_Z;}
            CR_reti_counter++;
          }
        }
        std::fclose(fp);
        if (CR_reti_counter>=3){
          CR_startZP = CR_LD_histoty[1];
          CR_endZP   = CR_LD_histoty[CR_reti_counter-3];
        }
      }

      // 가변 감쇠/강성 초기화 일부 생략 (원본 유지)
      if(Contact_Fcon_mode == 4){ FAAC3step.FAAC_Init(); }
    }

    // 시작점으로 이동 또는 파일에서 다음 레코드 읽기
    int reti = 0;
    if(PB_starting_path_done_flag){
      double path_out[6] = {0,};
      if(Posture_PB.PTP_6D_path_exe(path_out)){
        Desired_XYZ << path_out[0], path_out[1], path_out[2];
        Desired_RPY << path_out[3], path_out[4], path_out[5];
        LD_CFx = LD_CFy = LD_CFz = 0;
        KdToZero_flag=true; ZeroToKd_flag=false;
      } else {
        // 파일에서 한 줄 읽기 (보호)
        std::lock_guard<std::mutex> g(g_io_mtx);
        if (Hand_G_playback){
          reti = std::fscanf(Hand_G_playback, "%f %f %f %f %f %f %f %f %f\n",
                             &LD_X,&LD_Y,&LD_Z,&LD_Roll,&LD_Pitch,&LD_Yaw,&LD_CFx,&LD_CFy,&LD_CFz);
        } else {
          reti = -1;
        }
        if(reti != -1){
          Desired_XYZ << LD_X, LD_Y, LD_Z;
          Desired_RPY << LD_Roll, LD_Pitch, LD_Yaw;
        } else {
          if (Hand_G_playback){ std::fclose(Hand_G_playback); Hand_G_playback=nullptr; }
          PB_starting_path_done_flag = false;
          path_exe_counter = 0;
        }
      }

      if(reti != -1){
        // (Playback_mode == 1) 중심 로직 유지 (강성/감쇠 업데이트 등)
        AKin.EulerAngle2Rotation(Desired_rot,Desired_RPY);

        // 표면 법선 힘 publish
        PPB_RTinput.PRtz << -Desired_rot(0,2),-Desired_rot(1,2),-Desired_rot(2,2);
        PPB_RTinput.P_Tool_Rot = Desired_rot;
        PPB_RTinput.PXr = Desired_XYZ;
        PPB_RTinput.PX  = RArm.xc;
        PPB_RTinput.PFext = Contact_Rot_force;
        PPB_RTinput.PFd = LD_CFz;

        double surfN = (Power_PB.PU3.transpose()*Contact_Rot_force);
        PPB_surfN_Fext = surfN;
        YSurfN_Fext_msg_.data = surfN;
        YSurfN_Fext_pub_->publish(YSurfN_Fext_msg_);

        #if Playback_mode == 1
          if(Power_PB.playback_start(PPB_RTinput)) Desired_XYZ = Power_PB.PXc_0;
        #endif

        // 기록 (보호)
        {
          std::lock_guard<std::mutex> g(g_io_mtx);
          if (path_recording_pos) {
            int wrote = std::fprintf(
              path_recording_pos,
              "%10f %10f %10f %10f %10f %10f\n",
              PPB_RTinput.PFd,
              Contact_Rot_force(2),
              Power_PB.PTankE,
              Desired_rot(0,2), Desired_rot(1,2), Desired_rot(2,2)
            );
            if (wrote < 0) {
              RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                    "path_recording_pos write failed: %s", std::strerror(errno));
            }
          }
        }

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
      } else {
        KdToZero_flag = true; ZeroToKd_flag = false;
        if(PB_iter_cmd > PB_iter_cur){
          UR10e_mode_msg_.data = Playback_mode_cmd; PB_iter_cur++;
        }else{
          UR10e_mode_msg_.data = Motion_stop_cmd; PB_iter_cur = 1;
        }
        sprintf(Playback_iteration,"playback iter:(cur: %d/tot: %d)",PB_iter_cur,PB_iter_cmd);
        memcpy(message_status,Playback_iteration,sizeof(Playback_iteration));
        UR10e_mode_pub_->publish(UR10e_mode_msg_);
      }
    }

    // 로봇으로 명령 전송 (Isaac)
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < 6; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    pre_ctrl.store(ctrl.load(std::memory_order_relaxed), std::memory_order_relaxed);
  }
  else {
    // default safe
    RArm.qd = RArm.qc; RArm.qt = RArm.qc; RArm.dqc.setZero(); pause_cnt=0;
  }

  // 간단한 에러 메트릭 (사용시 확장)
  double maxerr=0;
  for(int i=0;i<6;i++){
    double err=RArm.qd(i)-RArm.qc(i);
    if(maxerr<std::fabs(err)) maxerr=std::fabs(err);
  }
}
