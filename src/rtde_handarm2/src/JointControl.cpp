// JointControl.cpp
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
// #include <std_msgs/msg/UInt16.hpp> 
#include <yaml-cpp/yaml.h>

constexpr int DOF = 6;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// ===================== 외부에서 선언/정의된 타입/인스턴스 가정 =====================
// - AKin                              : kinematics helper (ForwardK_T, InverseK_min, etc.)
// - RArm                              : robot state holder (qc, qd, xc, thc, Tc, Td, …)
// - Posture_PB, Power_PB              : path/Playback 관련 객체
// - J_single, path_planning           : path generator들
// - NRS_recording, NRS_VR_setting     : YAML::Node
// - Contact_Fcon_mode, Playback_mode  : 설정값
// - 다양한 상수/문자열: Hand_guiding_mode, Motion_stop_mode, …
// ================================================================================

// 재진입 방지
static std::mutex g_cmdmode_mtx;

// message_status 안전 설정 헬퍼
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

// ===================== JointControl =====================
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

    // Isaac joint command
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

    // 안정성 초기화
    if (hand_g_recording)  { std::fclose(hand_g_recording);  hand_g_recording  = nullptr; }
    if (Discre_P_recording){ std::fclose(Discre_P_recording);Discre_P_recording= nullptr; }
    if (VRCali_UR10CB_EE)  { std::fclose(VRCali_UR10CB_EE);  VRCali_UR10CB_EE  = nullptr; }
    if (VRCali_UR10CB_VR)  { std::fclose(VRCali_UR10CB_VR);  VRCali_UR10CB_VR  = nullptr; }
    if (Hand_G_playback)   { std::fclose(Hand_G_playback);   Hand_G_playback   = nullptr; }
    if (path_recording_pos){ std::fclose(path_recording_pos);path_recording_pos= nullptr; }
    if (path_recording_joint){ std::fclose(path_recording_joint); path_recording_joint = nullptr; }
    if (EXPdata1) { std::fclose(EXPdata1); EXPdata1=nullptr; }

    // 디버그/상태
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

// ===================== 모드 콜백 =====================
void JointControl::cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(g_cmdmode_mtx);

  try {
    mode_cmd = msg->data;
    printf("[DEBUG] cmdModeCallback called. mode_cmd=%u\n", mode_cmd);

    // 설정 확인
    if (!NRS_recording || !NRS_recording["hand_g_recording"]) {
      RCLCPP_WARN(node_->get_logger(), "YAML key 'hand_g_recording' not found.");
    }

    if (mode_cmd == Hand_guiding_mode_cmd) {
      ctrl.store(2, std::memory_order_release);
      set_status(message_status, Hand_guiding_mode);
    }
    else if (mode_cmd == Continuous_reording_start) {
      path_recording_flag = true;
      if (hand_g_recording) { std::fclose(hand_g_recording); hand_g_recording = nullptr; }
      auto hand_path = trim_path(NRS_recording["hand_g_recording"].as<std::string>());
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
      auto path = trim_path(NRS_recording["Discre_P_recording"].as<std::string>());
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
      auto path = trim_path(NRS_recording["Discre_P_recording"].as<std::string>());
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
      Decr_VR_points.bottomRows(1) << VR_pose[0],VR_pose[1],VR_pose[2],VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6];
      Num_VR_points++;
      std::snprintf(Saved_way_point, sizeof(Saved_way_point), "Saved VR points: %d", Num_VR_points);
      set_status(message_status, Saved_way_point);
      std::cout << "\n" << Decr_VR_points << std::endl;
    }
    else if (mode_cmd == VRCali_recording_save) {
      auto ee_path = trim_path(NRS_recording["VRCali_UR10CB_EE"].as<std::string>());
      auto vr_path = trim_path(NRS_recording["VRCali_UR10CB_VR"].as<std::string>());

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
          Decr_EE_points(i,0), Decr_EE_points(i,1),  Decr_EE_points(i,2),
          Decr_EE_points(i,3), Decr_EE_points(i,4),  Decr_EE_points(i,5),
          Decr_EE_points(i,6), Decr_EE_points(i,7),  Decr_EE_points(i,8),
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
      // 경로 파일 검증
      auto hand_path = trim_path(NRS_recording["hand_g_recording"].as<std::string>());
      if (hand_path.empty() || !std::filesystem::exists(hand_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory file not found: '%s'", hand_path.c_str());
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, Motion_stop_mode);
        return;
      }

      // 마지막 유효 9-플로트 라인 읽기 (타겟 정의용)
      auto read_last_valid_9f = [&](const std::string& p,
                                    float& x,float& y,float& z,
                                    float& r,float& pitch,float& yaw,
                                    float& fx,float& fy,float& fz)->bool {
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
      };

      float LDx=0, LDy=0, LDz=0, LDr=0, LDp=0, LDw=0, LFx=0, LFy=0, LFz=0;
      if (!read_last_valid_9f(hand_path, LDx,LDy,LDz, LDr,LDp,LDw, LFx,LFy,LFz)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse a valid line from '%s'", hand_path.c_str());
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

      // 시작점 -> 파일 첫번째 포즈까지 PTP 경로 생성
      double Linear_travel_vel  = 0.03;
      double Tar_pos[6]  = {LDx,LDy,LDz,LDr,LDp,LDw};
      double Init_pos[6] = {RArm.xc(0), RArm.xc(1), RArm.xc(2), RArm.thc(0), RArm.thc(1), RArm.thc(2)};

      double Linear_travel_time =
        std::sqrt(std::pow(Init_pos[0]-Tar_pos[0],2) +
                  std::pow(Init_pos[1]-Tar_pos[1],2) +
                  std::pow(Init_pos[2]-Tar_pos[2],2)) / Linear_travel_vel;
      if (Linear_travel_time < 3) Linear_travel_time = 3;

      PB_starting_path_done_flag = Posture_PB.PTP_6D_path_init(Init_pos, Tar_pos, Linear_travel_time);
      printf("PTP_6D_path_init was done \n");
      printf("Playback init path generation done\n");

      // 디버그 경로 기록 파일 열기(있으면)
      auto test_path = trim_path(NRS_recording["test_path"].as<std::string>());
      if (path_recording_pos) { std::fclose(path_recording_pos); path_recording_pos = nullptr; }
      path_recording_pos = std::fopen(test_path.c_str(), "wt");
      if (!path_recording_pos) {
        RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (%s)", test_path.c_str(), std::strerror(errno));
      }

      set_status(message_status, ST_path_gen_done);

      #if Playback_mode == 1
      Power_PB.playback_init(RArm.xc, RArm.thc);
      #endif

      ctrl.store(3, std::memory_order_release);
      pre_ctrl.store(0, std::memory_order_relaxed); // 다음 사이클에서 init 감지되도록
    }
    else if (mode_cmd == Motion_stop_cmd) {
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, Motion_stop_mode);
      if (Hand_G_playback)   { std::fclose(Hand_G_playback);   Hand_G_playback   = nullptr; }
      if (hand_g_recording)  { std::fclose(hand_g_recording);  hand_g_recording  = nullptr; }
      if (path_recording_pos){ std::fclose(path_recording_pos);path_recording_pos= nullptr; }
    }

  } catch (const std::exception& e) {
    RCLCPP_FATAL(node_->get_logger(), "cmdModeCallback exception: %s", e.what());
  }
}

// ===================== 기타 콜백 =====================
void JointControl::PbIterCallback(std_msgs::msg::UInt16::SharedPtr msg)
{
    PB_iter_cmd = msg->data;
    PB_iter_cur = 1; // 1 is right
}
void JointControl::JointCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    mjoint_cmd = msg->data;
    printf("\nSelected joint: %1.0f, Target relative joint angle: %4f \n", mjoint_cmd[0],mjoint_cmd[1]);

    double Tar_pos[] = {0.0};
    double Tar_vel[] = {0.0};
    double Waiting_time[] = {0,0}; // s

    double Vel_set = 0.1;  // rad/s
    Tar_pos[0] = std::fabs(mjoint_cmd[1]);
    Tar_vel[0] = (mjoint_cmd[1]>=0) ? Vel_set : -Vel_set;

    Joint_path_start <<RArm.qc(0),RArm.qc(1),RArm.qc(2),RArm.qc(3),RArm.qc(4),RArm.qc(5);

    Path_point_num = J_single.Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));
    if(Path_point_num != -1)
    {
        ctrl.store(1, std::memory_order_release);
        set_status(message_status, path_gen_done);
        path_done_flag = true;
    }
}
void JointControl::VRdataCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!VR_yaml_loader) {
        YAML::Node T_AD = NRS_VR_setting["T_AD"];
        for (std::size_t i = 0; i < T_AD.size(); ++i)
            for(std::size_t j = 0; j < T_AD[i].size(); ++j) VR_Cali_TAD(i,j) = T_AD[i][j].as<double>();

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

    VR_pose[0] = msg->pose.position.x;
    VR_pose[1] = msg->pose.position.y;
    VR_pose[2] = msg->pose.position.z;
    VR_pose[3] = msg->pose.orientation.w;
    VR_pose[4] = msg->pose.orientation.x;
    VR_pose[5] = msg->pose.orientation.y;
    VR_pose[6] = msg->pose.orientation.z;

    VR_Q2Rot = AKin.Qua2Rot(VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6]);
    VR_PoseM.block(0,0,3,3) = VR_Q2Rot;
    VR_PoseM.block(0,3,3,1) << VR_pose[0], VR_pose[1], VR_pose[2];
    VR_PoseM.block(3,0,1,4) << 0.0,0.0,0.0,1.0;

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

    VR_CalPoseM = VR_Cali_TAD*VR_PoseM*VR_Cali_TBC_inv*VR_Cali_TBC_PB*VR_Cali_TCE;

    Quaterniond VR_cal_qua = AKin.Rot2Qua(VR_CalPoseM.block(0,0,3,3));
    VR_cal_pose[0] = VR_CalPoseM(0,3);
    VR_cal_pose[1] = VR_CalPoseM(1,3);
    VR_cal_pose[2] = VR_CalPoseM(2,3);
    VR_cal_pose[3] = VR_cal_qua.w();
    VR_cal_pose[4] = VR_cal_qua.x();
    VR_cal_pose[5] = VR_cal_qua.y();
    VR_cal_pose[6] = VR_cal_qua.z();

    VR_CalRPY=AKin.VR_Rot2RPY(VR_CalPoseM.block(0,0,3,3));
    VR_CalPoseRPY << VR_CalPoseM(0,3),VR_CalPoseM(1,3),VR_CalPoseM(2,3), M_PI+VR_CalRPY(1), -M_PI-VR_CalRPY(2), -M_PI/2-VR_CalRPY(0);
}
void JointControl::getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (int i = 0; i < 6 && i < (int)msg->position.size(); ++i){
        RArm.qc[i] = msg->position[i];
    }
}

// ===================== 메인 제어 루프 =====================
void JointControl::CalculateAndPublishJoint()
{
    milisec += 20; // for printing timeline (ms)

    // 공통 전처리
	for(int i=0;i<6;i++){
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

	RArm.Td=RArm.Tc;
    VectorXd Init_qc = RArm.qc;
    int path_exe_counter = 0;

    // 상태 로드
    int c  = ctrl.load(std::memory_order_relaxed);
    int pc = pre_ctrl.load(std::memory_order_relaxed);

    // ====== Printing (요청했던 상세프린트 복원+보강) ======
    if(printer_counter >= print_period)
    {
        #if RT_printing
        printf("======================================== \n");
        printf("Now RUNNING MODE(%d), EXTERNAL MODE CMD: %d(%d) (%d/%d) \n",Actual_mode,c,pc,path_exe_counter,Path_point_num);
        printf("Current status: %s \n",message_status);
        printf("Selected force controller: %d \n",Contact_Fcon_mode);
        printf("milisec: %.2f \n", milisec);

        printf("A_q1: %.3f(%.1f), A_q2: %.3f(%.1f), A_q3: %.3f(%.1f), A_q4: %.3f(%.1f), A_q5: %.3f(%.1f), A_q6: %.3f(%.1f)\n",
        RArm.qc(0),RArm.qc(0)*(180/PI), RArm.qc(1),RArm.qc(1)*(180/PI), RArm.qc(2),RArm.qc(2)*(180/PI),
        RArm.qc(3),RArm.qc(3)*(180/PI), RArm.qc(4),RArm.qc(4)*(180/PI), RArm.qc(5),RArm.qc(5)*(180/PI));

        printf("D_q1: %.3f(%.1f), D_q2: %.3f(%.1f), D_q3: %.3f(%.1f), D_q4: %.3f(%.1f), D_q5: %.3f(%.1f), D_q6: %.3f(%.1f)\n",
        RArm.qd(0),RArm.qd(0)*(180/PI), RArm.qd(1),RArm.qd(1)*(180/PI), RArm.qd(2),RArm.qd(2)*(180/PI),
        RArm.qd(3),RArm.qd(3)*(180/PI), RArm.qd(4),RArm.qd(4)*(180/PI), RArm.qd(5),RArm.qd(5)*(180/PI));

        printf("HFx: %.2f, HFy: %.2f, HFz: %.2f | CFx: %.2f, CFy: %.2f, CFz: %.2f \n",
        ftS1(0),ftS1(1),ftS1(2), ftS2(0),ftS2(1),ftS2(2));

        printf("Act_XYZ: %.3f %.3f %.3f | Act_RPY: %.3f %.3f %.3f\n",
        RArm.xc(0),RArm.xc(1),RArm.xc(2), RArm.thc(0),RArm.thc(1),RArm.thc(2));

        printf("Des_X: %.3f, Des_Y: %.3f, Des_Z: %.3f, Des_R: %.3f(%.1f), Des_P: %.3f(%.1f), Des_Y: %.3f(%.1f) \n",
        Desired_XYZ(0),Desired_XYZ(1),Desired_XYZ(2),
        Desired_RPY(0),Desired_RPY(0)*(180/PI),Desired_RPY(1),Desired_RPY(1)*(180/PI),Desired_RPY(2),Desired_RPY(2)*(180/PI));

        printf("PB_PMx: %.3f, PB_PMy: %.3f, PB_PMz: %.4f\n",Power_PB.PRamM[0],Power_PB.PRamM[1],Power_PB.PRamM[2]);
        printf("PB_PDx: %.3f, PB_PDy: %.3f, PB_PDz: %.4f\n",Power_PB.PRamD[0],Power_PB.PRamD[1],Power_PB.PRamD[2]);
        printf("PB_PKx: %.3f, PB_PKy: %.3f, PB_PKz: %.4f\n",Power_PB.PRamK[0],Power_PB.PRamK[1],Power_PB.PRamK[2]);

        printf("DB_AVA_sigma: %0.3f, DB_AVA_phi: %0.3f\n",DB_AVA_sigma,DB_AVA_phi);
        printf("DB_PU3_x: %.3f, DB_PU3_y: %.3f, DB_PU3_z: %.3f\n", Power_PB.PU3(0), Power_PB.PU3(1), Power_PB.PU3(2));

        printf("Surf. normal Fd: %.3f, Fext: %.3f \n",PPB_RTinput.PFd, PPB_surfN_Fext);

        printf("VR_x: %.4f, VR_y: %.4f, VR_z: %.4f, VR_R: %.4f(%.2f), VR_P: %.4f(%.2f), VR_Y: %.4f(%.2f) \n",
        VR_CalPoseRPY(0), VR_CalPoseRPY(1), VR_CalPoseRPY(2),
        VR_CalPoseRPY(3),VR_CalPoseRPY(3)*(180/PI),VR_CalPoseRPY(4),VR_CalPoseRPY(4)*(180/PI),VR_CalPoseRPY(5),VR_CalPoseRPY(5)*(180/PI));
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

    // ====== 모드별 제어 ======

    // 0) 정지/유지
    if (c == 0) {
        speedmode = 0;
        RArm.qt = RArm.qc;
        RArm.dqc << 0,0,0,0,0,0;
        pause_cnt=0;

        std::array<double,6> joint_q = {RArm.qd(0),RArm.qd(1),RArm.qd(2),RArm.qd(3),RArm.qd(4),RArm.qd(5)};
        (void)joint_q;

        pre_ctrl.store(c, std::memory_order_relaxed);
        return;
    }

    // 1) 경로(조인트/EE) 실행
    if (c == 1) {
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
                } else if (mode_cmd == EE_Posture_control_mode_cmd) {
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
        for (int i = 0; i < 6; ++i) joint_state_.position[i] = RArm.qd(i);
        joint_commands_pub_->publish(joint_state_);

        pre_ctrl.store(c, std::memory_order_relaxed);
        return;
    }

    // 2) 핸드가이딩 (상세 로직 생략, 기존 코드 유지 가정)
    if (c == 2) {
        // … 기존 가이딩 로직 …
        pre_ctrl.store(c, std::memory_order_relaxed);
        return;
    }

    // 3) Playback (요청: 매 콜백마다 TXT 읽은 값과 Desired 출력)
    if (c == 3)
    {
        // 초기 진입시 1회 설정
        if (pc != c)
        {
            // contact/초기화 등 필요한 값들 설정 (생략 부분 기존 코드 그대로)
            // Discrete 파일 로드 시 가드
            auto disc_path = trim_path(NRS_recording["Discre_P_recording"].as<std::string>());
            FILE* fp = std::fopen(disc_path.c_str(), "rt");
            if (fp) {
                int CR_reti_counter = 0;
                double CR_LD_histoty[256] = {0,};
                while (true) {
                    float _x,_y,_z,_r,_p,_y2,_resi;
                    int CR_reti = std::fscanf(fp, "%f %f %f %f %f %f %f",
                                              &_x,&_y,&_z,&_r,&_p,&_y2,&_resi);
                    if (CR_reti == EOF) break;
                    if (CR_reti != 7) continue;
                    if (CR_reti_counter < 256) CR_LD_histoty[CR_reti_counter] = _z;
                    if (CR_reti_counter == 1) { CR_start<< _x,_y,_z; }
                    CR_reti_counter++;
                }
                std::fclose(fp);
                if (CR_reti_counter >= 3) {
                    CR_startZP = CR_LD_histoty[1];
                    CR_endZP   = CR_LD_histoty[CR_reti_counter-3];
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[PB] Discrete file lines < 3; Kd scheduling disabled.");
                    CR_startZP = CR_endZP = RArm.xc(2);
                }
            } else {
                RCLCPP_WARN(node_->get_logger(), "[PB] Cannot open discrete file: '%s'", disc_path.c_str());
            }

            // 파일 핸들 확인
            if (!Hand_G_playback) {
                RCLCPP_ERROR(node_->get_logger(), "[PB] Hand_G_playback is null at entry. Stop playback.");
                ctrl.store(0, std::memory_order_release);
                set_status(message_status, "Playback file null");
                pre_ctrl.store(c, std::memory_order_relaxed);
                return;
            }
        }

        // 실제 실행
        // 현재 자세
        Vector6d Hadm_pos_act; Hadm_pos_act << RArm.xc(0),RArm.xc(1),RArm.xc(2),RArm.thc(0),RArm.thc(1),RArm.thc(2);

        int reti = 0;
        if(PB_starting_path_done_flag == true)
        {
            double path_out[6] = {0,};
            if(Posture_PB.PTP_6D_path_exe(path_out))
            {
                Desired_XYZ << path_out[0], path_out[1], path_out[2];
                Desired_RPY << path_out[3], path_out[4], path_out[5];

                // 초기 이동 중에는 파일 Fz 영향 없음
                LD_CFx = LD_CFy = LD_CFz = 0.0f;

                // 디버그: 초기이동 단계
                printf("[PB] INITMOVE Desired XYZ: %.4f %.4f %.4f | RPY: %.4f %.4f %.4f | Fz_txt: %.4f\n",
                       Desired_XYZ(0),Desired_XYZ(1),Desired_XYZ(2),
                       Desired_RPY(0),Desired_RPY(1),Desired_RPY(2), (double)LD_CFz);
            }
            else
            {
                // 본격적으로 TXT를 매 프레임 읽어서 반영
                if (!Hand_G_playback) {
                    RCLCPP_ERROR(node_->get_logger(), "[PB] playback file closed unexpectedly.");
                    ctrl.store(0, std::memory_order_release);
                    set_status(message_status, "Playback file closed");
                    pre_ctrl.store(c, std::memory_order_relaxed);
                    return;
                }
                reti = std::fscanf(Hand_G_playback, "%f %f %f %f %f %f %f %f %f",
                                   &LD_X,&LD_Y,&LD_Z,&LD_Roll,&LD_Pitch,&LD_Yaw,
                                   &LD_CFx,&LD_CFy,&LD_CFz);
                if (reti != 9) {
                    // 파일 끝 or 에러
                    std::fclose(Hand_G_playback); Hand_G_playback = nullptr;
                    PB_starting_path_done_flag = false;
                    path_exe_counter = 0;
                    printf("[PB] End of file (reti=%d). Stop playback.\n", reti);
                    ctrl.store(0, std::memory_order_release);
                    set_status(message_status, "Playback finished");
                    pre_ctrl.store(c, std::memory_order_relaxed);
                    return;
                }

                Desired_XYZ << (double)LD_X, (double)LD_Y, (double)LD_Z;
                Desired_RPY << (double)LD_Roll, (double)LD_Pitch, (double)LD_Yaw;

                // ★ 매 콜백마다 TXT 값 및 Desired 출력
                printf("[PB] TXT-> X:%.4f Y:%.4f Z:%.4f | R:%.4f P:%.4f Y:%.4f | Fz:%.4f\n",
                       (double)LD_X,(double)LD_Y,(double)LD_Z,
                       (double)LD_Roll,(double)LD_Pitch,(double)LD_Yaw,(double)LD_CFz);
                printf("[PB] DES-> X:%.4f Y:%.4f Z:%.4f | R:%.4f P:%.4f Y:%.4f\n",
                       Desired_XYZ(0),Desired_XYZ(1),Desired_XYZ(2),
                       Desired_RPY(0),Desired_RPY(1),Desired_RPY(2));
            }

            // Playback_mode == 1 (Power playback)인 경우 필요한 보정/제어 수행 (기존 코드 유지 가정)
            AKin.EulerAngle2Rotation(Desired_rot,Desired_RPY);
            RArm.Td << Desired_rot(0,0),Desired_rot(0,1),Desired_rot(0,2),Desired_XYZ(0),
                       Desired_rot(1,0),Desired_rot(1,1),Desired_rot(1,2),Desired_XYZ(1),
                       Desired_rot(2,0),Desired_rot(2,1),Desired_rot(2,2),Desired_XYZ(2),
                       0,0,0,1;

            // IK
            #if TCP_standard == 0
            AKin.InverseK_min(&RArm);
            #else
            AKin.Ycontact_InverseK_min(&RArm);
            #endif

            // 로깅 (있으면)
            if (path_recording_pos) {
                int wrote = std::fprintf(path_recording_pos,
                    "%10f %10f %10f %10f %10f %10f\n",
                    PPB_RTinput.PFd, (double)ftS2(2),
                    Power_PB.PTankE, Desired_rot(0,2), Desired_rot(1,2), Desired_rot(2,2));
                if (wrote < 0) {
                    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                        "path_recording_pos write failed: %s", std::strerror(errno));
                }
            }

            // 로봇 명령 퍼블리시(시뮬)
            joint_state_.header.stamp = node_->now();
            for (int i = 0; i < 6; ++i) joint_state_.position[i] = RArm.qd(i);
            joint_commands_pub_->publish(joint_state_);
        }
        else
        {
            // 시작경로 미생성(이론상 여기 안 옴)
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                                 "[PB] Start path not initialized.");
        }

        pre_ctrl.store(c, std::memory_order_relaxed);
        return;
    }

    // 그 외
    speedmode = 0;
    RArm.qd = RArm.qc;
    RArm.qt = RArm.qc;
    RArm.dqc << 0,0,0,0,0,0;
    pause_cnt=0;
    pre_ctrl.store(c, std::memory_order_relaxed);
}
