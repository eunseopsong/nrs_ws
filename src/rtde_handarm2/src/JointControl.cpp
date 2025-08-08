#include "JointControl.h"
#include <iostream>
#include <memory>

#include <algorithm>
#include <cctype>
constexpr int DOF = 6;
#include <sys/stat.h>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <string>
#include <filesystem>
#include <cerrno>
#include <cstring>

JointControl::JointControl(const rclcpp::Node::SharedPtr& node)
: node_(node), milisec(0)
////   rclcpp::Node("nrs_control_node"),
{
    AdaptiveK_msg_ = std::make_unique<nrs_msgmonitoring2::MsgMonitoring>(node_, "AdaptiveK_msg");
    FAAC3step_msg_ = std::make_unique<nrs_msgmonitoring2::MsgMonitoring>(node_, "FAAC3step_msg");

    // Publishers
    YSurfN_Fext_pub_ = node_->create_publisher<std_msgs::msg::Float64>("YSurfN_Fext", 20);
    UR10e_mode_pub_  = node_->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 20);
    // UR10_Jangle_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_Jangle", 20);
    UR10_pose_pub_   = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_pose", 20);
    UR10_wrench_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_wrench", 20);
    joint_commands_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands" , 20); // Add on 2025.07.09


    //// Add on 2025.07.09
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
        std::chrono::milliseconds(1),
        std::bind(&JointControl::CalculateAndPublishJoint, this));
}
JointControl::~JointControl() {}



// ===== 헬퍼: 경로 트리밍 =====
static std::string trim_path(std::string s) {
  auto notspace = [](unsigned char c){ return !std::isspace(c); };
  // 앞쪽 공백 제거
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
  // 뒤쪽 공백/CR/LF 제거
  while (!s.empty() && (s.back()=='\r' || s.back()=='\n' || std::isspace((unsigned char)s.back()))) {
    s.pop_back();
  }
  return s;
}




// ===== LoadFirstTrajectory: 마지막 유효 라인 1줄만 읽기 (9개 float) =====
bool JointControl::LoadFirstTrajectory(
    const std::string& filepath_raw,
    float& LD_X, float& LD_Y, float& LD_Z,
    float& LD_Roll, float& LD_Pitch, float& LD_Yaw,
    float& LD_CFx, float& LD_CFy, float& LD_CFz)
{
    std::string filepath = trim_path(filepath_raw);

    if (filepath.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Empty filepath for trajectory (after trim).");
        return false;
    }

    // 선택: 존재 확인 (실패 원인 빠르게 파악용)
    if (!std::filesystem::exists(filepath)) {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory file not found: '%s'", filepath.c_str());
        return false;
    }

    FILE* fp = fopen(filepath.c_str(), "r"); // Linux에선 "r"이면 충분
    if (!fp) {
        RCLCPP_ERROR(node_->get_logger(), "fopen failed: '%s' (errno=%d: %s)",
                     filepath.c_str(), errno, strerror(errno));
        return false;
    }

    // 파일 끝까지 돌면서 "마지막으로 파싱 성공한" 1줄을 보관
    char buf[2048];
    float x=0, y=0, z=0, roll=0, pitch=0, yaw=0, fx=0, fy=0, fz=0;
    bool got = false;

    while (fgets(buf, sizeof(buf), fp)) {
        // 공백/주석 라인 스킵
        bool only_space = true;
        for (char* p = buf; *p; ++p) { if (!std::isspace((unsigned char)*p)) { only_space = false; break; } }
        if (only_space || buf[0]=='#') continue;

        float tx,ty,tz,tr,tp,tw,tfx,tfy,tfz;
        // 마지막 개행/CR 섞여도 괜찮게 처리
        int n = sscanf(buf, " %f %f %f %f %f %f %f %f %f ",
                       &tx,&ty,&tz,&tr,&tp,&tw,&tfx,&tfy,&tfz);
        if (n == 9) {
            x=tx; y=ty; z=tz; roll=tr; pitch=tp; yaw=tw; fx=tfx; fy=tfy; fz=tfz;
            got = true; // 계속 갱신 → 최종적으로 "마지막 유효 라인" 값이 남음
        }
    }

    fclose(fp);

    if (!got) {
        RCLCPP_ERROR(node_->get_logger(), "Parse failed: no valid line with 9 floats in '%s'", filepath.c_str());
        return false;
    }

    LD_X = x; LD_Y = y; LD_Z = z;
    LD_Roll = roll; LD_Pitch = pitch; LD_Yaw = yaw;
    LD_CFx = fx; LD_CFy = fy; LD_CFz = fz;
    return true;
}



// ===== cmdModeCallback: 파일 여는 부분만 방어코드 추가 + 실패 시 조기 리턴 =====
void JointControl::cmdModeCallback(std_msgs::msg::UInt16::SharedPtr msg)
{
    try {
        std::cout << "[DEBUG] cmdModeCallback called. ";
        mode_cmd = msg->data;
        printf("mode_cmd: %d \n", mode_cmd);
        std::cout << std::endl;

        RCLCPP_INFO(node_->get_logger(), "[YAML loaded] hand_g_recording exists: %d, IsScalar: %d",
            (bool)NRS_recording["hand_g_recording"], NRS_recording["hand_g_recording"].IsScalar());

        if (mode_cmd == Joint_control_mode_cmd) {
            // no-op
        }
        else if (mode_cmd == EE_Posture_control_mode_cmd) {
            // 기존 코드 유지 (비활성 #if 0)
        }
        else if (mode_cmd == Hand_guiding_mode_cmd) {
            ctrl = 2;
            memcpy(message_status, Hand_guiding_mode, sizeof(Hand_guiding_mode));
        }
        else if (mode_cmd == Continuous_reording_start) // data recording flag on
        {
            path_recording_flag = true;

            auto raw = NRS_recording["hand_g_recording"].as<std::string>();
            auto hand_g_recording_path = trim_path(raw);

            hand_g_recording = fopen(hand_g_recording_path.c_str(), "wt");
            if (!hand_g_recording) {
                RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)",
                             hand_g_recording_path.c_str(), errno, strerror(errno));
                path_recording_flag = false;
                return; // 조기 종료: 기록 못 할 때는 더 진행하지 않음
            }

            memcpy(message_status, Data_recording_on, sizeof(Data_recording_on));
        }
        else if (mode_cmd == Continusous_recording_end) // data recording flag off
        {
            path_recording_flag = false;
            if (hand_g_recording) {
                fclose(hand_g_recording);
                hand_g_recording = nullptr;
            }
            memcpy(message_status, Data_recording_off, sizeof(Data_recording_off));
        }
        else if (mode_cmd == Descrete_reording_start) {
            // (원래 로직 유지)
            if (Num_RD_points != 0) {
                Inst_RD_points = Decr_RD_points;
                Decr_RD_points.resize(Num_RD_points+1,6);
                Decr_RD_points.topRows(Num_RD_points) = Inst_RD_points;
            } else { Decr_RD_points.topRows(Num_RD_points+1) = Inst_RD_points; }

            Decr_RD_points.bottomRows(1) << RArm.xc(0), RArm.xc(1), RArm.xc(2), RArm.thc(0), RArm.thc(1), RArm.thc(2);
            Num_RD_points ++;
            sprintf(Saved_way_point,"Saved way point: %d",Num_RD_points);
            memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
            std::cout << "\n" <<Decr_RD_points << std::endl;
        }
        else if (mode_cmd == Descrete_recording_save)
        {
            auto raw = NRS_recording["Descre_P_recording"].as<std::string>();
            auto Descre_P_recording_path = trim_path(raw);

            Descre_P_recording = fopen(Descre_P_recording_path.c_str(), "wt");
            if (!Descre_P_recording) {
                RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)",
                             Descre_P_recording_path.c_str(), errno, strerror(errno));
                return;
            }

            for (int i = 0; i < Num_RD_points; i++) {
                fprintf(Descre_P_recording,"%10f %10f %10f %10f %10f %10f %10f\n",
                        Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2),
                        Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), (double)0.0);
            }
            fclose(Descre_P_recording);
            Num_RD_points = 0;
            Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
            Inst_RD_points = Eigen::MatrixXd::Zero(1,6);
            printf("\n Descrete points was save to txt file \n");
        }
        else if (mode_cmd == VRTeac_reording_start) {
            // (원래 로직 유지)
            if (Num_RD_points != 0) {
                Inst_RD_points = Decr_RD_points;
                Decr_RD_points.resize(Num_RD_points+1,6);
                Decr_RD_points.topRows(Num_RD_points) = Inst_RD_points;
            } else { Decr_RD_points.topRows(Num_RD_points+1) = Inst_RD_points; }

            Decr_RD_points.bottomRows(1) <<
                VR_CalPoseRPY(0), VR_CalPoseRPY(1), VR_CalPoseRPY(2),
                VR_CalPoseRPY(3), VR_CalPoseRPY(4), VR_CalPoseRPY(5);
            Num_RD_points ++;
            sprintf(Saved_way_point,"Saved way point: %d",Num_RD_points);
            memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
            std::cout << "\n" <<Decr_RD_points << std::endl;
        }
        else if (mode_cmd == VRTeac_recording_save)
        {
            auto raw = NRS_recording["Descre_P_recording"].as<std::string>();
            auto Descre_P_recording_path = trim_path(raw);

            Descre_P_recording = fopen(Descre_P_recording_path.c_str(), "wt");
            if (!Descre_P_recording) {
                RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)",
                             Descre_P_recording_path.c_str(), errno, strerror(errno));
                return;
            }

            for (int i = 0; i < Num_RD_points; i++) {
                fprintf(Descre_P_recording,"%10f %10f %10f %10f %10f %10f %10f\n",
                        Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2),
                        Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), (double)0.0);
            }
            fclose(Descre_P_recording);
            Num_RD_points = 0;
            Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
            Inst_RD_points = Eigen::MatrixXd::Zero(1,6);
            printf("\n Descrete points was save to txt file \n");
        }
        else if (mode_cmd == VRCali_reording_start) {
            // (원래 로직 유지)
            if (Num_EE_points != 0) {
                Inst_EE_points = Decr_EE_points;
                Decr_EE_points.resize(Num_EE_points+1,12);
                Decr_EE_points.topRows(Num_EE_points) = Inst_EE_points;
            } else { Decr_EE_points.topRows(Num_EE_points+1) = Inst_EE_points; }

            Decr_EE_points.bottomRows(1) <<
                RArm.Tc(0,0),RArm.Tc(1,0),RArm.Tc(2,0),
                RArm.Tc(0,1),RArm.Tc(1,1),RArm.Tc(2,1),
                RArm.Tc(0,2),RArm.Tc(1,2),RArm.Tc(2,2),
                RArm.Tc(0,3),RArm.Tc(1,3),RArm.Tc(2,3);
            Num_EE_points ++;
            sprintf(Saved_way_point,"Saved cali. points: %d",Num_EE_points);
            memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
            std::cout << "\n" <<Decr_EE_points << std::endl;

            if (Num_VR_points != 0) {
                Inst_VR_points = Decr_VR_points;
                Decr_VR_points.resize(Num_VR_points+1,7);
                Decr_VR_points.topRows(Num_VR_points) = Inst_VR_points;
            } else { Decr_VR_points.topRows(Num_VR_points+1) = Inst_VR_points; }

            Decr_VR_points.bottomRows(1) <<
                VR_pose[0],VR_pose[1],VR_pose[2],
                VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6];
            Num_VR_points ++;
            sprintf(Saved_way_point,"Saved VR points: %d",Num_VR_points);
            memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
            std::cout << "\n" <<Decr_VR_points << std::endl;
        }
        else if (mode_cmd == VRCali_recording_save)
        {
            auto raw1 = NRS_recording["VRCali_UR10CB_EE"].as<std::string>();
            auto raw2 = NRS_recording["VRCali_UR10CB_VR"].as<std::string>();
            auto VRCali_UR10CB_EE_path = trim_path(raw1);
            auto VRCali_UR10CB_VR_path = trim_path(raw2);

            VRCali_UR10CB_EE = fopen(VRCali_UR10CB_EE_path.c_str(), "wt");
            if (!VRCali_UR10CB_EE) {
                RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)",
                             VRCali_UR10CB_EE_path.c_str(), errno, strerror(errno));
                return;
            }
            for (int i = 0; i < Num_EE_points; i++) {
                fprintf(VRCali_UR10CB_EE,"%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
                        Decr_EE_points(i,0), Decr_EE_points(i,1),  Decr_EE_points(i,2),
                        Decr_EE_points(i,3), Decr_EE_points(i,4),  Decr_EE_points(i,5),
                        Decr_EE_points(i,6), Decr_EE_points(i,7),  Decr_EE_points(i,8),
                        Decr_EE_points(i,9), Decr_EE_points(i,10), Decr_EE_points(i,11));
            }
            fclose(VRCali_UR10CB_EE);
            Num_EE_points = 0;
            Decr_EE_points = Eigen::MatrixXd::Zero(1,12);
            Inst_EE_points = Eigen::MatrixXd::Zero(1,12);
            printf("\n Descrete EE points was save to txt file \n");

            VRCali_UR10CB_VR = fopen(VRCali_UR10CB_VR_path.c_str(), "wt");
            if (!VRCali_UR10CB_VR) {
                RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)",
                             VRCali_UR10CB_VR_path.c_str(), errno, strerror(errno));
                return;
            }
            for (int i = 0; i < Num_VR_points; i++) {
                fprintf(VRCali_UR10CB_VR,"%10f %10f %10f %10f %10f %10f %10f\n",
                        Decr_VR_points(i,0), Decr_VR_points(i,1), Decr_VR_points(i,2),
                        Decr_VR_points(i,3), Decr_VR_points(i,4), Decr_VR_points(i,5), Decr_VR_points(i,6));
            }
            fclose(VRCali_UR10CB_VR);
            Num_VR_points = 0;
            Decr_VR_points = Eigen::MatrixXd::Zero(1,7);
            Inst_VR_points = Eigen::MatrixXd::Zero(1,7);
            printf("\n Cali points was save to txt file \n");
        }
        else if (mode_cmd == Playback_mode_cmd)
        {
            // Trajectory path
            auto raw = NRS_recording["hand_g_recording"].as<std::string>();
            auto hand_g_recording_path = trim_path(raw);

            // (A) 재생 원본 파일 오픈 (전체 재생용)
            Hand_G_playback = fopen(hand_g_recording_path.c_str(), "rt");
            if (!Hand_G_playback) {
                RCLCPP_ERROR(node_->get_logger(), "open for read failed: '%s' (errno=%d: %s)",
                            hand_g_recording_path.c_str(), errno, strerror(errno));
                ctrl = 0;
                memcpy(message_status, Motion_stop_mode, sizeof(Motion_stop_mode));
                return;
            }

            // (B) 시작점만 미리 읽어오는 건 지금의 LoadFirstTrajectory로 OK
            float LD_X, LD_Y, LD_Z, LD_Roll, LD_Pitch, LD_Yaw;
            float LD_CFx, LD_CFy, LD_CFz;
            if (!LoadFirstTrajectory(hand_g_recording_path,
                LD_X, LD_Y, LD_Z, LD_Roll, LD_Pitch, LD_Yaw,
                LD_CFx, LD_CFy, LD_CFz)) {
                RCLCPP_ERROR(node_->get_logger(), "❌ Failed to load trajectory data from: %s",
                            hand_g_recording_path.c_str());
                fclose(Hand_G_playback); Hand_G_playback = nullptr;
                ctrl = 0;
                memcpy(message_status, Motion_stop_mode, sizeof(Motion_stop_mode));
                return;
            }

            printf("%f %f %f %f %f %f %f %f %f\n",
                   LD_X, LD_Y, LD_Z, LD_Roll, LD_Pitch, LD_Yaw, LD_CFx, LD_CFy, LD_CFz);

            // 기존 로직 그대로
            double Linear_travel_vel = 0.03; // m/s
            double Linear_travel_time;
            double Tar_pos[6] = {LD_X, LD_Y, LD_Z, LD_Roll, LD_Pitch, LD_Yaw};
            double Init_pos[6] = {RArm.xc(0), RArm.xc(1), RArm.xc(2),
                                  RArm.thc(0), RArm.thc(1), RArm.thc(2)};

            Linear_travel_time = sqrt(pow(Init_pos[0]-Tar_pos[0],2)+
                                      pow(Init_pos[1]-Tar_pos[1],2)+
                                      pow(Init_pos[2]-Tar_pos[2],2)) / Linear_travel_vel;
            if (Linear_travel_time < 3) Linear_travel_time = 3;

            PB_starting_path_done_flag = Posture_PB.PTP_6D_path_init(Init_pos, Tar_pos, Linear_travel_time);
            printf("Playback init path generation done \n");

            auto raw_tp = NRS_recording["test_path"].as<std::string>();
            auto test_path_path = trim_path(raw_tp);
            path_recording_pos = fopen(test_path_path.c_str(), "wt");
            if (!path_recording_pos) {
                RCLCPP_ERROR(node_->get_logger(), "open for write failed: '%s' (errno=%d: %s)",
                             test_path_path.c_str(), errno, strerror(errno));
                // 파일 기록만 실패 → 재생은 계속할지 말지는 선택. 여기선 계속.
            }
            memcpy(message_status, ST_path_gen_done, sizeof(ST_path_gen_done));

            #if Playback_mode == 1
            Power_PB.playback_init(RArm.xc, RArm.thc);
            #endif

            ctrl = 3;
        }
        else if (mode_cmd == Motion_stop_cmd) {
            ctrl = 0;
            memcpy(message_status, Motion_stop_mode, sizeof(Motion_stop_mode));
        }

    } catch (const std::exception& e) {
        RCLCPP_FATAL(node_->get_logger(), "cmdModeCallback exception: %s", e.what());
    }
}





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
    double Waiting_time[] = {0,0}; // Waiting time, unit : s, 0이 아닌 값을 넣으려면 동일 array 위치의 pos와 vel은 0

    double Vel_set = 0.1;          // rad/s

    // Set the joint relative target angle
    Tar_pos[0] = fabs(mjoint_cmd[1]);

    // Set the joint target velocity
    if(mjoint_cmd[1]>=0) Tar_vel[0] = Vel_set;
    else Tar_vel[0] = -Vel_set;

    Joint_path_start <<RArm.qc(0),RArm.qc(1),RArm.qc(2),RArm.qc(3),RArm.qc(4),RArm.qc(5);

    Path_point_num = J_single.Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));
    if(Path_point_num != -1)
    {
        ctrl = 1;
        memcpy(message_status,path_gen_done,sizeof(path_gen_done));
        path_done_flag = true;

    }
}

void JointControl::VRdataCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    /* Calibration matrix load from yaml file */
    if(VR_yaml_loader == false) // To load the CM once
    {
        YAML::Node T_AD = NRS_VR_setting["T_AD"];
        for (std::size_t i = 0; i < T_AD.size(); ++i)
        {
            for(std::size_t j = 0; j < T_AD[i].size(); ++j){VR_Cali_TAD(i,j) = T_AD[i][j].as<double>();}
        }

        // std::cout << VR_Cali_TAD << std::endl;

        YAML::Node T_BC = NRS_VR_setting["T_BC"];
        for (std::size_t i = 0; i < T_BC.size(); ++i)
        {
            for(std::size_t j = 0; j < T_BC[i].size(); ++j){VR_Cali_TBC(i,j) = T_BC[i][j].as<double>();}
        }

        // std::cout << VR_Cali_TBC << std::endl;

        YAML::Node T_BC_inv = NRS_VR_setting["T_BC_inv"];
        for (std::size_t i = 0; i < T_BC_inv.size(); ++i)
        {
            for(std::size_t j = 0; j < T_BC_inv[i].size(); ++j){VR_Cali_TBC_inv(i,j) = T_BC_inv[i][j].as<double>();}
        }

        // std::cout << VR_Cali_TBC_inv << std::endl;

        YAML::Node T_BC_PB = NRS_VR_setting["T_BC_PB"];
        for (std::size_t i = 0; i < T_BC_PB.size(); ++i)
        {
            for(std::size_t j = 0; j < T_BC_PB[i].size(); ++j){VR_Cali_TBC_PB(i,j) = T_BC_PB[i][j].as<double>();}
        }

        // std::cout << VR_Cali_TBC_PB << std::endl;

        YAML::Node T_CE = NRS_VR_setting["T_CE"];
        for (std::size_t i = 0; i < T_CE.size(); ++i)
        {
            for(std::size_t j = 0; j < T_CE[i].size(); ++j){VR_Cali_TCE(i,j) = T_CE[i][j].as<double>();}
        }

        // std::cout << VR_Cali_TCE << std::endl;

        YAML::Node R_Adj = NRS_VR_setting["R_Adj"];
        for (std::size_t i = 0; i < R_Adj.size(); ++i)
        {
            for(std::size_t j = 0; j < R_Adj[i].size(); ++j){VR_Cali_RAdj(i,j) = R_Adj[i][j].as<double>();}
        }

        VR_yaml_loader = true;
    }

    /*** VR position data upload ***/
    VR_pose[0] = msg->pose.position.x;
    VR_pose[1] = msg->pose.position.y;
    VR_pose[2] = msg->pose.position.z;

    /*** VR orientation data upload ***/
    VR_pose[3] = msg->pose.orientation.w;
    VR_pose[4] = msg->pose.orientation.x;
    VR_pose[5] = msg->pose.orientation.y;
    VR_pose[6] = msg->pose.orientation.z;

    /*** VR orientation quaternion to rotation matrix ***/
    /* Raw VR HTM*/
    VR_Q2Rot = AKin.Qua2Rot(VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6]);
    VR_PoseM.block(0,0,3,3) = VR_Q2Rot;
    VR_PoseM.block(0,3,3,1) << VR_pose[0], VR_pose[1], VR_pose[2];

    /* Transfrom from rotation matrix to Euler angles */
    VR_PoseM.block(3,0,1,4) << 0.0, 0.0, 0.0, 1.0;

    /* VR adjustment matrix - Heuristic method */

    // VR_Cali_TAdj.block(0,0,3,3) = AKin.RotZ((-90.0)*(pi/180.0))*AKin.RotX((-180.0)*(pi/180.0))*VR_Cali_RAdj.transpose();
    VR_Cali_TAdj.block(0,0,3,3) = AKin.RotZ((0.0)*(pi/180.0))*AKin.RotX((0.0)*(pi/180.0))*VR_Cali_RAdj.transpose();
    // VR_Cali_TAdj.block(0,0,3,3) = AKin.RotX((-180.0)*(pi/180.0))*AKin.RotZ((-90.0)*(pi/180.0))*VR_Cali_RAdj.transpose();
    VR_Cali_TAdj.block(0,3,3,1) << 0.0, 0.0, 0.0;
    VR_Cali_TAdj.block(3,0,1,4) << 0.0, 0.0, 0.0, 1.0;

    /* VR data adjustment - Heuristic method */
    VR_PoseM = VR_Cali_TAdj*VR_PoseM;

    /* VR matrix to quaternion */
    Quaterniond VR_roted_qua =AKin.Rot2Qua(VR_PoseM.block(0,0,3,3));
    VR_pose[0] = VR_PoseM(0,3); // X -> X
    VR_pose[1] = VR_PoseM(1,3); // Y -> Y
    VR_pose[2] = VR_PoseM(2,3); // Z -> Z
    VR_pose[3] = VR_roted_qua.w();
    VR_pose[4] = VR_roted_qua.x();
    VR_pose[5] = VR_roted_qua.y();
    VR_pose[6] = VR_roted_qua.z();


    /*** VR coordinate transform from base-station to robot-base ***/

    /* VR calibration by multiplying calibration matrix */
    // VR_CalPoseM = VR_Cali_TAD*VR_PoseM*VR_Cali_TBC.transpose();
    VR_CalPoseM = VR_Cali_TAD*VR_PoseM*VR_Cali_TBC_inv*VR_Cali_TBC_PB*VR_Cali_TCE;
    // VR_CalPoseM.block(0,0,3,3) = VR_Cali_RAdj*VR_CalPoseM.block(0,0,3,3);

    /* Transfrom from rotation matrix to Qua for RViz */
    Quaterniond VR_cal_qua =AKin.Rot2Qua(VR_CalPoseM.block(0,0,3,3));
    VR_cal_pose[0] = VR_CalPoseM(0,3); // X -> X
    VR_cal_pose[1] = VR_CalPoseM(1,3); // Y -> Y
    VR_cal_pose[2] = VR_CalPoseM(2,3); // Z -> Z
    VR_cal_pose[3] = VR_cal_qua.w();
    VR_cal_pose[4] = VR_cal_qua.x();
    VR_cal_pose[5] = VR_cal_qua.y();
    VR_cal_pose[6] = VR_cal_qua.z();


    /* Transfrom from rotation matrix to Euler angles */
    VR_CalRPY=AKin.VR_Rot2RPY(VR_CalPoseM.block(0,0,3,3));
    // VR_CalPoseRPY << VR_CalPoseM(0,3),VR_CalPoseM(1,3),VR_CalPoseM(2,3),VR_CalRPY(0),VR_CalRPY(1),VR_CalRPY(2); // Original
    VR_CalPoseRPY << VR_CalPoseM(0,3),VR_CalPoseM(1,3),VR_CalPoseM(2,3), pi+VR_CalRPY(1), -pi-VR_CalRPY(2), -pi/2-VR_CalRPY(0); // UR10CB custom

    /* Calibration confirmation */
    // To use this confirmation method, we must use hand-guiding mode
    #if 0
    printf("Ro_x: %.4f, Ro_y: %.4f, Ro_z: %.4f, Ro_R: %.4f, Ro_P: %.4f, Ro_Y: %.4f \n",
    Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2), Desired_RPY(0), Desired_RPY(1), Desired_RPY(2));
    printf("VR_x: %.4f, VR_y: %.4f, VR_z: %.4f, VR_R: %.4f, VR_P: %.4f, VR_Y: %.4f \n",
    VR_CalPoseRPY(0), VR_CalPoseRPY(1), VR_CalPoseRPY(2), VR_CalPoseRPY(3), VR_CalPoseRPY(4), VR_CalPoseRPY(5));
    #endif

    // vive_ros_tracker::VRposRtMsg_Qua pos_cal_stamped_Qua;

    // pos_cal_stamped_Qua.x = VR_cal_pose[0];
    // pos_cal_stamped_Qua.y = VR_cal_pose[1];
    // pos_cal_stamped_Qua.z = VR_cal_pose[2];
    // pos_cal_stamped_Qua.qw = VR_cal_pose[3];
    // pos_cal_stamped_Qua.qx = VR_cal_pose[4];
    // pos_cal_stamped_Qua.qy = VR_cal_pose[5];
    // pos_cal_stamped_Qua.qz = VR_cal_pose[6];



    // vive_ros_tracker::VRposRtMsg_RPY pos_cal_stamped_RPY;

    // pos_cal_stamped_RPY.x = VR_CalPoseRPY(0);
    // pos_cal_stamped_RPY.y = VR_CalPoseRPY(1);
    // pos_cal_stamped_RPY.z = VR_CalPoseRPY(2);
    // pos_cal_stamped_RPY.roll = VR_CalPoseRPY(3);
    // pos_cal_stamped_RPY.pitch = VR_CalPoseRPY(4);
    // pos_cal_stamped_RPY.yaw = VR_CalPoseRPY(5);
}

//// JointControl.cpp에서 정의 수정 2025.08.05
void JointControl::getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // std::cout << "[DEBUG] getActualQ called. Current joint states: ";
    for (int i = 0; i < 6; ++i){
        RArm.qc[i] = msg->position[i];
        std::cout << RArm.qc[i] << " ";
    }
    std::cout << std::endl;
}

void JointControl::CalculateAndPublishJoint()
{
    milisec += 1; // to get the published time


    // std::string hand_g_recording_path = NRS_recording["hand_g_recording"].as<std::string>();
    // std::cout << "[DEBUG] raw path = [" << hand_g_recording_path << "]" << std::endl;
    // std::cout << "[DEBUG] size of path = " << hand_g_recording_path.size() << std::endl;


    /* Set application realtime priority */
    int priority = 80;
    //// if (RTDEUtility::setRealtimePriority(priority)) {
    ////     std::cout << "Realtime priority was set to " << priority << std::endl;
    //// } else {
    ////     std::cerr << "실시간 우선순위 설정에 실패했습니다. 관리자 권한이 필요할 수 있습니다." << std::endl;
    //// }

    /* rtde 연결 확인 로그 */
    //// if (!rtde_control.isProgramRunning()) {
    ////     std::cerr << "RTDE 프로그램이 실행되지 않았습니다." << std::endl;
    ////     return -1;
    //// }
    //// if (!rtde_control.isConnected() || !rtde_receive.isConnected()) {
    //// std::cerr << "RTDE connection failed!" << std::endl;
    //// return -1;
    //// }

    //////signal(SIGINT, raiseFlag); // Active
	////// signal(SIGTERM, raiseFlag); // Termination (ctrl + c)

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    std::chrono::duration<double>pre_now = start-start;

    //joint
	for(int i=0;i<6;i++){
		RArm.ddqd(i) = 0;
		RArm.dqd(i) = 0;
		RArm.dqc(i) = 0;
	}

	RArm.qd = RArm.qc;
	RArm.qt = RArm.qc;
    #if TCP_standard == 0
    AKin.ForwardK_T(&RArm); // qc -> Tc, xc (current X,Y,Z)
    #elif TCP_standard == 1
    AKin.Ycontact_ForwardK_T(&RArm);
    #endif
	RArm.Td=RArm.Tc;
    VectorXd Init_qc = RArm.qc;
    VectorXd qd_pre(6), qc_pre(6), dqd_pre(6);

    int path_exe_counter = 0;

    // Hand-guding parameter setting
    #if Adm_mode == 0 // defualt mode
    Hadmit_M << 0.1,0.1,0.1, 0.005,0.005,0.005;
    Hadmit_D << 0.4,0.4,0.4, 0.02,0.02,0.02;
    Hadmit_K << 5,5,5,1,1,1;


    #elif Adm_mode == 1 // tustin method

    #if Hand_spring_mode == 0 // Hand-guding mode
    /* Light guiding parameters */
    // Hadmit_M << 2,2,2, 0.1,0.1,0.1;
    // Hadmit_D << 30,30,30, 1,1,1;
    // Hadmit_K << 0.0,0.0,0.0, 0.0,0.0,0.0;

    /* Middle guiding parameters */
    // Hadmit_M << 4,4,4, 0.2,0.2,0.2;
    // Hadmit_D << 40,40,40, 2,2,2;
    // Hadmit_K << 0.0,0.0,0.0, 0.0,0.0,0.0;

    /* Massive guiding parameters */
    Hadmit_M << 6,6,6, 0.4,0.4,0.4;
    Hadmit_D << 50,50,50, 4,4,4;
    Hadmit_K << 0.0,0.0,0.0, 0.0,0.0,0.0;

    /* Very Massive guiding parameters */
    // Hadmit_M << 10,10,10, 1,1,1;
    // Hadmit_D << 80,80,80, 8,8,8;
    // Hadmit_K << 0.0,0.0,0.0, 0.0,0.0,0.0;

    #elif Hand_spring_mode == 1 // Spring mode
    Hadmit_M << 1,1,1, 0.1,0.1,0.1;
    Hadmit_D << 500,500,500, 50,50,50;
    Hadmit_K << 0,0,0, 0,0,0;
    #endif


    for(int i=0;i<6;i++) // MDK update
    {
        Hadmit_force[i].adm_1D_MDK((double)Hadmit_M(i),(double)Hadmit_D(i),(double)Hadmit_K(i));
    }

    #endif

    // cmdModeCallback();
    ctrl = 3;
    //// ctrl debugger ////
    // printf("ctrl: %d, pre_ctrl: %d \n", ctrl, pre_ctrl);
    // printf("mode_cmd: %d \n", mode_cmd);
    ///////////////////////

    // spring mode control parameter
    VectorXd Hspring_mode_init_pos(6);


    // printf("While loop start\n");

    // printf("MODE SELECT: Select Your Task\n");
    // printf("1 : For Servoj,  q : Quit    \n");
    // printf("getchar: %s \n", key_MODE);

    // key_MODE=getchar();
    // key_MODE='1';
    key_MODE='q';
    

    /* Recording file open */
    auto test_path_joint_path = NRS_recording["test_path_joint"].as<std::string>();
    path_recording_joint = fopen(test_path_joint_path.c_str(),"wt");

    auto EXPdata1_path = NRS_recording["EXPdata1_path"].as<std::string>();
    EXPdata1 = fopen(EXPdata1_path.c_str(),"wt");

    /*==================== Main control loop ==================*/
    try
    {
        // switch (key_MODE)
        // {
        // case '1':
            ctrl = 0;
            pause_cnt = 0;
            //// while (running)
            // while (rclcpp::ok() && running) // Add on 2025.08.03 00:10
            // {
                // t_start = rtde_control.initPeriod();
                // getActualQ(); // Get the joint data of UR10e

                // tcp_pose = rtde_receive.getActualTCPPose(); // Get the TCP data of UR10e
                // ros::spinOnce(); // Get the ROS message data

                #if TCP_standard == 0
                AKin.ForwardK_T(&RArm); // qc -> Tc, xc (current X,Y,Z)
                #elif TCP_standard == 1
                AKin.Ycontact_ForwardK_T(&RArm);
                #endif

                AKin.Rotation2EulerAngle(&RArm); // Tc -> thc (current R,P,Y)

                /*====== printing ======*/
                if(printer_counter >= print_period)
                {
                    #if RT_printing
                    printf("======================================== \n");
                    printf("Now RUNNING MODE(%d), EXTERNAL MODE CMD: %d(%d) (%d/%d) \n",Actual_mode,ctrl,pre_ctrl,path_exe_counter,Path_point_num); //show the current mode data
                    printf("Current status: %s \n",message_status); //show the status message
                    printf("Selected force controller: %d \n",Contact_Fcon_mode);
                    printf("milisec: %.2f \n", milisec); // t 값을 디버깅하기 위해 출력
                    // printf("[DEBUG] hand_g_recording path: %s \n", hand_g_recording_path.c_str());


                    // UR10e actual joint angle monitoring
                    printf("A_q1: %.3f(%.1f), A_q2: %.3f(%.1f), A_q3: %.3f(%.1f), A_q4: %.3f(%.1f), A_q5: %.3f(%.1f), A_q6: %.3f(%.1f)\n",
                    RArm.qc(0),RArm.qc(0)*(180/PI), RArm.qc(1),RArm.qc(1)*(180/PI), RArm.qc(2),RArm.qc(2)*(180/PI),
                    RArm.qc(3),RArm.qc(3)*(180/PI), RArm.qc(4),RArm.qc(4)*(180/PI), RArm.qc(5),RArm.qc(5)*(180/PI));
                    // UR10e desired joint angle monitoring (output of inverse kinematics)
                    printf("D_q1: %.3f(%.1f), D_q2: %.3f(%.1f), D_q3: %.3f(%.1f), D_q4: %.3f(%.1f), D_q5: %.3f(%.1f), D_q6: %.3f(%.1f)\n",
                    RArm.qd(0),RArm.qd(0)*(180/PI), RArm.qd(1),RArm.qd(1)*(180/PI), RArm.qd(2),RArm.qd(2)*(180/PI),
                    RArm.qd(3),RArm.qd(3)*(180/PI), RArm.qd(4),RArm.qd(4)*(180/PI), RArm.qd(5),RArm.qd(5)*(180/PI));

                    // Handle FT sensor data monitoring
                    printf("HFx: %.2f, HFy: %.2f, HFz: %.2f, HMx: %.3f, HMy: %.3f, HMz: %.3f \n",
                    ftS1(0),ftS1(1),ftS1(2),ftS1(3),ftS1(4),ftS1(5));
                    // Contact FT sensor data monitoring
                    printf("CFx: %.2f, CFy: %.2f, CFz: %.2f, CMx: %.3f, CMy: %.3f, CMz: %.3f \n",
                    ftS2(0),ftS2(1),ftS2(2),ftS2(3),ftS2(4),ftS2(5));

                    // UR10e actual EE pose
                    printf("Act_X: %.3f, Act_Y: %.3f, Act_Z: %.3f, Act_R: %.3f(%.1f), Act_P: %.3f(%.1f), Act_Y: %.3f(%.1f)\n",
                    RArm.xc(0),RArm.xc(1),RArm.xc(2),
                    RArm.thc(0),RArm.thc(0)*(180/PI),RArm.thc(1),RArm.thc(1)*(180/PI),RArm.thc(2),RArm.thc(2)*(180/PI));
                    // UR10e desired EE pose
                    printf("Des_X: %.3f, Des_Y: %.3f, Des_Z: %.3f, Des_R: %.3f(%.1f), Des_P: %.3f(%.1f), Des_Y: %.3f(%.1f) \n",
                    Desired_XYZ(0),Desired_XYZ(1),Desired_XYZ(2),
                    Desired_RPY(0),Desired_RPY(0)*(180/PI),Desired_RPY(1),Desired_RPY(1)*(180/PI),Desired_RPY(2),Desired_RPY(2)*(180/PI));

                    // UR10e Handle Mass & Damper & tank energy
                    #if 0
                    printf("HM_fx: %.3f, HM_fy: %.3f, HM_fz: %.3f, HM_mx: %.3f, HM_my: %.3f, HM_mz: %.3f \n",
                    Hadmit_M(0),Hadmit_M(1),Hadmit_M(2),Hadmit_M(3),Hadmit_M(4),Hadmit_M(5));
                    printf("HD_fx: %.3f, HD_fy: %.3f, HD_fz: %.3f, HD_mx: %.3f, HD_my: %.3f, HD_mz: %.3f \n",
                    Hadmit_D(0),Hadmit_D(1),Hadmit_D(2),Hadmit_D(3),Hadmit_D(4),Hadmit_D(5));
                    printf("HE_fx: %.3f, HE_fy: %.3f, HE_fz: %.3f, HE: %.3f, HE_my: %.3f, HE_mz: %.3f \n",
                    AD_HG_et[0].tank_energy,AD_HG_et[1].tank_energy,AD_HG_et[2].tank_energy,
                    AD_HG_et[3].tank_energy,AD_HG_et[4].tank_energy,AD_HG_et[5].tank_energy);
                    #endif

                    // Power playback desired AC parameter
                    printf("PB_PMx: %.3f, PB_PMy: %.3f, PB_PMz: %.4f\n",Power_PB.PRamM[0],Power_PB.PRamM[1],Power_PB.PRamM[2]);
                    printf("PB_PDx: %.3f, PB_PDy: %.3f, PB_PDz: %.4f\n",Power_PB.PRamD[0],Power_PB.PRamD[1],Power_PB.PRamD[2]);
                    printf("PB_PKx: %.3f, PB_PKy: %.3f, PB_PKz: %.4f\n",Power_PB.PRamK[0],Power_PB.PRamK[1],Power_PB.PRamK[2]);

                    // Variable admittance conrtol with adaptive damping
                    printf("DB_AVA_sigma: %0.3f, DB_AVA_phi: %0.3f\n",DB_AVA_sigma,DB_AVA_phi);
                    printf("DB_PU3_x: %.3f, DB_PU3_y: %.3f, DB_PU3_z: %.3f\n", Power_PB.PU3(0), Power_PB.PU3(1), Power_PB.PU3(2));
                    // printf("DB_PU3_x: %.3f, DB_PU3_y: %.3f, DB_PU3_z: %.3f\n", CR_start(0), CR_start(1), CR_start(2));

                    // Surface normal force
                    printf("Surf. normal Fd: %.3f, Fext: %.3f \n",PPB_RTinput.PFd, PPB_surfN_Fext);

                    // Desired rotation matrix
                    #if 0
                    printf("RotX1: %.3f, RotX2: %.3f, RotX3: %.3f \n",Desired_rot(0,0),Desired_rot(1,0),Desired_rot(2,0));
                    printf("RotZ1: %.3f, RotZ2: %.3f, RotZ3: %.3f \n",Desired_rot(0,2),Desired_rot(1,2),Desired_rot(2,2));
                    #endif

                    // VR data
                    #if 0 // [Raw & quaternion value]
                    printf("VR_X: %.4f, VR_Y: %.4f, VR_Z: %.4f \n",VR_pose[0],VR_pose[1],VR_pose[2]);
                    printf("VR_Qw: %.4f, VR_Qx: %.4f, VR_Qy: %.4f, VR_Qz: %.4f \n",VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6]);
                    #endif
                    #if 1// [Calibrated value]
                    printf("VR_x: %.4f, VR_y: %.4f, VR_z: %.4f, VR_R: %.4f(%.2f), VR_P: %.4f(%.2f), VR_Y: %.4f(%.2f) \n",
                    VR_CalPoseRPY(0), VR_CalPoseRPY(1), VR_CalPoseRPY(2),
                    VR_CalPoseRPY(3),VR_CalPoseRPY(3)*(180/PI),VR_CalPoseRPY(4),VR_CalPoseRPY(4)*(180/PI),VR_CalPoseRPY(5),VR_CalPoseRPY(5)*(180/PI));
                    #endif

                    #endif
                    printer_counter = 0;
                }
                else
                {
                    printer_counter++;
                }
                /*====== Robot information publishing ======*/
                // UR10_Jangle_msg_.data.clear();
                UR10_pose_msg_.data.clear();
                UR10_wrench_msg_.data.clear();

                for(int i=0; i<6; i++)
                {
                    // UR10_Jangle_msg_.data.push_back(RArm.qc(i));
                    if (i<3) {UR10_pose_msg_.data.push_back(RArm.xc(i));}
                    else {UR10_pose_msg_.data.push_back(RArm.thc(i-3));}
                    UR10_wrench_msg_.data.push_back(ftS2(i)); // Contact sensor data
                }
                // UR10_Jangle_pub_->publish(UR10_Jangle_msg_); //// UR10_Jangle_pub.publish(UR10_Jangle_msg);
                UR10_pose_pub_->publish(UR10_pose_msg_);     //// UR10_pose_pub.publish(UR10_pose_msg);
                UR10_wrench_pub_->publish(UR10_wrench_msg_); //// UR10_wrench_pub.publish(UR10_wrench_msg);

                /*====== Control modes ======*/

                /* Inital state (stay at the inital pose) */
                if (ctrl == 0) {
                    speedmode = 0;
                    //RArm.qd = RArm.qc;
                    RArm.qt = RArm.qc;
                    RArm.dqc << 0,0,0,0,0,0;
                    pause_cnt=0;

                    #if Actual_mode == 0 // test mode
                    joint_q = {Init_qc(0), Init_qc(1), Init_qc(2), Init_qc(3), Init_qc(4), Init_qc(5)};
                    #elif Actual_mode == 1 // actual control mode
                    joint_q = {RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5)};
                    #endif

                    //// rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
                    //// rtde_control.waitPeriod(t_start);

                    pre_ctrl = ctrl; // for ctrl mode switching detection
                    // continue;
                }

                /* Cartesian position control mode */
                else if (ctrl == 1) {

                    // Path execution
                    if(path_done_flag == true)
                    {
                        if(path_exe_counter<Path_point_num)
                        {
                            /* The case of joint position control*/
                            if(mode_cmd == Joint_control_mode_cmd)
                            {
                                RArm.qd(0) = Joint_path_start(0) + ((double)(mjoint_cmd[0]==1))*J_single.Final_pos(path_exe_counter,1);
                                RArm.qd(1) = Joint_path_start(1) + ((double)(mjoint_cmd[0]==2))*J_single.Final_pos(path_exe_counter,1);
                                RArm.qd(2) = Joint_path_start(2) + ((double)(mjoint_cmd[0]==3))*J_single.Final_pos(path_exe_counter,1);
                                RArm.qd(3) = Joint_path_start(3) + ((double)(mjoint_cmd[0]==4))*J_single.Final_pos(path_exe_counter,1);
                                RArm.qd(4) = Joint_path_start(4) + ((double)(mjoint_cmd[0]==5))*J_single.Final_pos(path_exe_counter,1);
                                RArm.qd(5) = Joint_path_start(5) + ((double)(mjoint_cmd[0]==6))*J_single.Final_pos(path_exe_counter,1);

                                fprintf(path_recording_joint,"%10f %10f %10f %10f %10f %10f \n",
                                RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5));
                            }

                            /***** The case of EE posture control *****/
                            else if(mode_cmd == EE_Posture_control_mode_cmd)
                            {
                                // For inverse kinematics with generated path
                                Desired_XYZ << TCP_path_start(0), TCP_path_start(1),TCP_path_start(2);
                                Desired_RPY << TCP_path_start(3)+path_planning.Final_pos(path_exe_counter,1),TCP_path_start(4),TCP_path_start(5);

                                AKin.EulerAngle2Rotation(Desired_rot,Desired_RPY);
                                RArm.Td << Desired_rot(0,0),Desired_rot(0,1),Desired_rot(0,2),Desired_XYZ(0),
                                        Desired_rot(1,0),Desired_rot(1,1),Desired_rot(1,2),Desired_XYZ(1),
                                        Desired_rot(2,0),Desired_rot(2,1),Desired_rot(2,2),Desired_XYZ(2),
                                        0               ,0               ,0               ,1;


                                #if TCP_standard == 0
                                AKin.InverseK_min(&RArm); // input: Td , output: qd
                                #elif TCP_standard == 1
                                AKin.Ycontact_InverseK_min(&RArm);
                                #endif
                                fprintf(path_recording_joint,"%10f %10f %10f %10f %10f %10f \n",
                                RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5));
                            }

                            path_exe_counter++;
                        }
                        else
                        {
                            path_done_flag = false;
                            path_exe_counter = 0;
                        }

                    }

                    #if Actual_mode == 0 // test mode
                    joint_q = {Init_qc(0), Init_qc(1), Init_qc(2), Init_qc(3), Init_qc(4), Init_qc(5)};
                    #elif Actual_mode == 1 // actual control mode
                    joint_q = {RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5)};
                    #endif

                    //// rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
                    //// rtde_control.waitPeriod(t_start);

                    pre_ctrl = ctrl; // for ctrl mode switching detection
                    // continue;
                }



                /* Hand-guiding control mode */
                else if (ctrl == 2) {
                    speedmode = 0;
                    //RArm.qd = RArm.qc;
                    RArm.qt = RArm.qc;
                    RArm.dqc << 0,0,0,0,0,0;
                    pause_cnt=0;

                    /* Current actual posture data load */
                    Hadm_pos_act << RArm.xc(0),RArm.xc(1),RArm.xc(2),RArm.thc(0),RArm.thc(1),RArm.thc(2);

                    /*Hand-guiding initialization*/
                    if(pre_ctrl != ctrl)
                    {
                        Hadm_pos_cmd << RArm.xc(0),RArm.xc(1),RArm.xc(2),RArm.thc(0),RArm.thc(1),RArm.thc(2);
                        Hspring_mode_init_pos << RArm.xc(0),RArm.xc(1),RArm.xc(2),RArm.thc(0),RArm.thc(1),RArm.thc(2);

                        Hadm_past_pos_act = Hadm_pos_act;
                        for(int i=0;i<6;i++)
                        {
                            /* Hand-guding mode initial start */
                            Hadmit_force[i].adm_1D_init(0-Hadm_pos_cmd(i),0-Hadm_FT_data(i),dt);
                            /* Hand-guding mode initial end */

                            /* Handle VAC energy tank initial start */
                            // 1) High Pass Filter initialization
                            et_HFT_HPF[i].HPF_par.ts = dt; // Sampling time(s)
                            et_HFT_HPF[i].HPF_par.f_cut = 5; // Cut-off frequency(Hz)
                            et_HFT_HPF[i].HPF_par.zeta = 0.7; // Damping ratio of HPF

                            // 2) parameter adaptation parameter initialization
                            AD_HG_et[i].init_Md = Hadmit_M(i);
                            AD_HG_et[i].init_Dd = Hadmit_D(i);

                            PHadmit_M(i) = Hadmit_M(i);

                            /* Handle VAC energy tank initial end */
                        }
                    }



                    /*** UR10e moment conversion ***/
                    /** in UR10e Mx must be applied on -pitch & My must be applied Roll **/
                    /* For handle sensor */
                    Handle_Rot_force(0) = ftS1(0);
                    Handle_Rot_force(1) = ftS1(1);
                    Handle_Rot_force(2) = ftS1(2);

                    Handle_Rot_moment(0) = -ftS1(4);
                    Handle_Rot_moment(1) = ftS1(3);
                    Handle_Rot_moment(2) = ftS1(5);
                    /* For contact sensor */
                    Contact_Rot_force(0) = ftS2(0);
                    Contact_Rot_force(1) = ftS2(1);
                    Contact_Rot_force(2) = ftS2(2);

                    Contact_Rot_moment(0) = -ftS2(4);
                    Contact_Rot_moment(1) = ftS2(3);
                    Contact_Rot_moment(2) = ftS2(5);

                    /* Force shaping algorithm --- start */

                    shaped_F_dot = Handle_Rot_force(0)*Contact_Rot_force(0)+Handle_Rot_force(1)*Contact_Rot_force(1)+Handle_Rot_force(2)*Contact_Rot_force(2);
                    shaped_F_criteria = shaped_F_dot/Contact_Rot_force.norm();


                    if(shaped_F_criteria<0 && Contact_Rot_force.norm()>= 2.0)
                    {
                        //shaped_F_offset p control

                        double target_suf_F = 20; // N(norm), target pressing force for surf teaching
                        double alpha_P_gain = 0.01;
                        double offset_satur_max = 0.3;
                        double offset_satur_min = -0.01;


                        shaped_F_offset = shaped_F_offset + alpha_P_gain*(target_suf_F-Contact_Rot_force.norm()); //similar with D control

                        if(shaped_F_offset >= offset_satur_max) shaped_F_offset = offset_satur_max;

                        if(shaped_F_offset <= offset_satur_min) shaped_F_offset = offset_satur_min;


                        shaped_F_compen = Handle_Rot_force - shaped_F_dot*(Contact_Rot_force/pow(Contact_Rot_force.norm(),2));

                        Shaped_Force = Handle_Rot_force - shaped_F_dot*(Contact_Rot_force/pow(Contact_Rot_force.norm(),2))
                        - (tan(shaped_F_alpha)*shaped_F_compen.norm()+shaped_F_offset)*(Contact_Rot_force/Contact_Rot_force.norm());
                    }
                    else
                    {
                        Shaped_Force = Handle_Rot_force;
                    }
                    /* Force shaping algorithm --- end */

                    Hadm_FT_data(0) = Shaped_Force(0);
                    Hadm_FT_data(1) = Shaped_Force(1);
                    Hadm_FT_data(2) = Shaped_Force(2);

                    Hadm_FT_data(3) = Handle_Rot_moment(0);
                    Hadm_FT_data(4) = Handle_Rot_moment(1);
                    Hadm_FT_data(5) = Handle_Rot_moment(2);


                    #if Adm_mode == 1

                    for(int i=0;i<6;i++)
                    {
                        /* Energy tank start(The order is important) */

                        // 1) Tank input parameter calculation
                        et_car_vel(i) = (Hadm_pos_act(i)-Hadm_past_pos_act(i))/dt; // Actual velocity calculation
                        et_norm[i] = fabs(et_HFT_HPF[i].HighPassFilter(Hadm_FT_data(i))); // nominal equation must be like this form

                        // 2) Energy tank execution
                        AD_HG_et[i].Md_dot = (Hadmit_M(i)-PHadmit_M(i))/dt;
                        AD_HG_et[i].Dd = Hadmit_D[i];
                        AD_HG_et[i].current_vel = et_car_vel(i);
                        AD_HG_et[i].Energy_tank();

                        // Admittance parameter update
                        #if 1 // if you want to turn off the parameter adaptation -> 0
                        if(par_adap_stCounter >= 500) // after xx second, the energy thank execute the algorithm
                        {
                            if((et_norm[i] >= et_norm_threshold[i])&&(et_wait_counter[i]==0))
                            {
                                Md_ti[i] = Hadmit_M(i);
                                Md_tf[i] = Md_ti[i] + fabs(2*(AD_HG_et[i].tank_energy-AD_HG_et[i].init_energy)/(xm_dot[i]*xm_dot[i]));
                                if(fabs(Md_tf[i]-Md_ti[i]) >= M_divi_lim[i]) // mass saturation
                                {
                                    Md_tf[i] = Md_ti[i] + M_divi_lim[i];
                                }
                                Dd_tf[i] = Md_tf[i]*(AD_HG_et[i].init_Dd/AD_HG_et[i].init_Md);
                                par_adap_flag[i] = true;
                            }
                        }
                        else par_adap_stCounter++;

                        // Mass parameter transfer to past
                        PHadmit_M(i) = Hadmit_M(i);

                        // After delta T it must be applied
                        if(par_adap_flag[i] == true)
                        {
                            if((et_wait_counter[i] <= deltaT*1000)) et_wait_counter[i]++;
                            else
                            {
                                Hadmit_M(i) = Md_tf[i];
                                Hadmit_D(i) = Dd_tf[i];

                                par_adap_flag[i] = false;
                                et_wait_counter[i] = 0;
                                et_decre_counter[i] = 0;
                            }
                        }
                        #endif
                        /* Energy tank end */

                        /* Hand-guiding mode start */
                        Hadmit_force[i].adm_1D_MDK((double)Hadmit_M(i),(double)Hadmit_D(i),(double)Hadmit_K(i));
                        Hadm_pos_cmd(i)=Hadmit_force[i].adm_1D_control((double)0,(double)0, Hadm_FT_data(i));
                        /* Hand-guiding mode end */

                        /* Automatical decrement of mass & damper start */
                        if(Hadmit_M(i)>AD_HG_et[i].init_Md)
                        {
                            if(et_decre_counter[i] == 0)
                            {
                                init_divi_M[i] = Hadmit_M(i) - AD_HG_et[i].init_Md;
                            }

                            Hadmit_M(i) = AD_HG_et[i].init_Md + init_divi_M[i]*exp(-et_decre_par[i]*et_decre_counter[i]);
                            Hadmit_D(i) = Hadmit_M(i)*(AD_HG_et[i].init_Dd/AD_HG_et[i].init_Md);
                            et_decre_counter[i]++;
                        }
                        /* Automatical decrement of mass & damper end */
                    }

                    #endif

                    /* Inverse kinematics start */
                    Desired_XYZ << Hadm_pos_cmd(0),Hadm_pos_cmd(1),Hadm_pos_cmd(2);
                    Desired_RPY << Hadm_pos_cmd(3),Hadm_pos_cmd(4),Hadm_pos_cmd(5);

                    AKin.EulerAngle2Rotation(Desired_rot,Desired_RPY);
                    RArm.Td << Desired_rot(0,0),Desired_rot(0,1),Desired_rot(0,2),Desired_XYZ(0),
                            Desired_rot(1,0),Desired_rot(1,1),Desired_rot(1,2),Desired_XYZ(1),
                            Desired_rot(2,0),Desired_rot(2,1),Desired_rot(2,2),Desired_XYZ(2),
                            0               ,0               ,0               ,1;

                    #if TCP_standard == 0
                    AKin.InverseK_min(&RArm); // input: Td , output: qd
                    #elif TCP_standard == 1
                    AKin.Ycontact_InverseK_min(&RArm);
                    #endif
                    /* Inverse kinematics end */

                    /* Recording(Desired posture & Contact force) start */
                    if(path_recording_flag == true)
                    {
                        fprintf(hand_g_recording,"%10f %10f %10f %10f %10f %10f %10f %10f %10f \n",
                        Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2), Desired_RPY(0), Desired_RPY(1), Desired_RPY(2),
                        Contact_Rot_force(0),Contact_Rot_force(1),Contact_Rot_force(2));
                    }
                    /* Recording(Desired posture & Contact force) end */

                    /* Transfer the command to manipulator start */
                    #if Actual_mode == 0 // test mode
                    joint_q = {Init_qc(0), Init_qc(1), Init_qc(2), Init_qc(3), Init_qc(4), Init_qc(5)};
                    #elif Actual_mode == 1 // actual control mode
                    joint_q = {RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5)};
                    #endif

                    //// rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
                    /* Transfer the command to manipulator end */

                    /* Data backup to past start */
                    Hadm_past_pos_act = Hadm_pos_act;
                    /* Data backup to past end */

                    //// rtde_control.waitPeriod(t_start);

                    pre_ctrl = ctrl; // for ctrl mode switching detection
                    // continue;
                }

                /* Posture/Power playback control mode */
                else if (ctrl == 3)
                {
                    speedmode = 0;
                    //RArm.qd = RArm.qc;
                    RArm.qt = RArm.qc;
                    RArm.dqc << 0,0,0,0,0,0;
                    pause_cnt=0;

                    /*** UR10e moment conversion ***/
                    /** in UR10e Mx must be applied on -pitch & My must be applied Roll **/
                    /* For contact sensor */
                    Contact_Rot_force(0) = ftS2(0);
                    Contact_Rot_force(1) = ftS2(1);
                    Contact_Rot_force(2) = ftS2(2);

                    Contact_Rot_moment(0) = -ftS2(4);
                    Contact_Rot_moment(1) = ftS2(3);
                    Contact_Rot_moment(2) = ftS2(5);

                    /* Posture/Power playback control mode initialization */
                    if(pre_ctrl != ctrl)
                    {
                        #if Adm_mode == 1
                        for(int i=0;i<6;i++)
                        {
                            if(i<3) Cadmit_playback[i].adm_1D_init((double)0,0-Contact_Rot_force(i),dt);
                            else Cadmit_playback[i].adm_1D_init((double)0,0-Contact_Rot_moment(i-3),dt);
                        }
                        #endif

                        /* [Find contact region using Descre_P_recording point] */
                        KdToZero_flag = false;
                        ZeroToKd_flag = false;
                        KTZ_Fd_flag = false;
                        Kd_change_dist = 0.005; // Kd changed distance, Unit: m
                        KTZ_Kd_init = Power_PB.PRamK[2]; // Save the initial contact stiffness
                        KTZ_update_par = Power_PB.Ts/2; // Kd to Z, Z to Kd upadte parameter (half-Ts is proper)
                        KTZ_Z_init, KTZ_Z_end;
                        KTZ_Kd_threshold = 3;
                        KTZ_Kd_h = KTZ_Kd_init;

                        auto Descre_P_recording_path = NRS_recording["Descre_P_recording"].as<std::string>();
                        Descre_P_recording = fopen(Descre_P_recording_path.c_str(),"rt");
                        int CR_reti = 0;
                        int CR_reti_counter = 0;
                        double CR_LD_histoty[100] = {0,};

                        while(CR_reti != -1)
                        {
                            CR_reti = fscanf(Descre_P_recording, "%f %f %f %f %f %f %f\n", &LD_X, &LD_Y, &LD_Z, &LD_Roll, &LD_Pitch, &LD_Yaw, &LD_resi);
                            CR_LD_histoty[CR_reti_counter] = LD_Z;
                            if(CR_reti_counter == 1) {CR_start<< LD_X,LD_Y,LD_Z;}
                            CR_reti_counter++;
                        }
                        /* Contact start Z point load */
                        CR_startZP = CR_LD_histoty[1];
                        printf("CR_startZP : %f \n",CR_startZP);
                        /* Contact end Z point load */
                        CR_endZP = CR_LD_histoty[CR_reti_counter-3]; // Do not change the num 3!!
                        printf("CR_endZP : %f \n",CR_endZP);

                        /* [Dynamical system based adaptive variable damping admittance law initalization] */
                        if(Contact_Fcon_mode == 1)
                        {
                            DB_AVA_phi = 0;
                            DB_AVA_Dd_init = Power_PB.PRamD[2]; // Save the initial Z-damping
                            DB_AVA_Xc = Power_PB.PXc_0(2);
                            DB_AVA_Xc_pre = Power_PB.PXc_0(2);
                            DB_AVA_Xc_dot = 0;
                            /* Step 0 : Update rate calculation of damping variation */
                            DB_AVA_sigma = DB_AVA_Rd*(dt*Power_PB.PRamD[2])/((double)1.0 + dt*Power_PB.PRamD[2]);
                        }
                        /* [Dynamical system based adaptive variable stiffness admittance law initalization] */
                        else if(Contact_Fcon_mode == 2)
                        {
                            DB_AVA_phi = 0;
                            DB_AVA_Kd_init = Power_PB.PRamK[2]; // Save the initial Z-damping
                            DB_AVA_Xc = Power_PB.PXc_0(2);
                            DB_AVA_Xr = Power_PB.PXc_0(2);

                            /* Step 0 : Update rate calculation of stiffness variation */
                            DB_AVA_sigma = 0.5; // it must be handled later
                        }
                        /* [Fuzzy based adaptive variable stiffness admittance law initalization] */
                        else if(Contact_Fcon_mode == 3)
                        {
                            DB_AVA_phi = 0;
                            DB_AVA_Kd_init = Power_PB.PRamK[2]; // Save the initial Z-damping
                            DB_AVA_Xc = Power_PB.PXc_0(2);
                            DB_AVA_Xr = Power_PB.PXc_0(2);

                            Fuzzy_F_error = 0;
                            Fuzzy_F_Perror = 0;
                            Fuzzy_F_error_dot = 0;

                            /* Step 0 : Update rate calculation of stiffness variation */
                            DB_AVA_sigma = 0.5; // Sigma initialization
                        }
                        /* [Fuzzy based adaptive variable mass & damping admittance law initalization] */
                        else if(Contact_Fcon_mode == 4)
                        {
                            // DB_AVA_phi = 0;
                            // DB_AVA_Kd_init = Power_PB.PRamK[2]; // Save the initial Z-damping
                            DB_AVA_Xc = Power_PB.PXc_0(2);
                            DB_AVA_Xr = Power_PB.PXc_0(2);
                            DB_AVA_X = Power_PB.PXc_0(2);

                            // #if 0
                            // Fuzzy_F_error = 0;
                            // Fuzzy_F_Perror = 0;
                            // Fuzzy_F_error_dot = 0;

                            // Fuzzy_md_ratio = Power_PB.PRamD[2]/Power_PB.PRamM[2];
                            // #endif

                            // /* Step 0 : Update rate calculation of stiffness variation */
                            // DB_AVA_sigma = 0.5; // Sigma initialization

                            FAAC3step.FAAC_Init();
                        }

                    }

                    /* Current actual posture data load */
                    Hadm_pos_act << RArm.xc(0),RArm.xc(1),RArm.xc(2),RArm.thc(0),RArm.thc(1),RArm.thc(2);

                    /* Path execution */
                    int reti = 0;
                    if(PB_starting_path_done_flag == true)
                    {
                        double path_out[6] = {0,};
                        /* Initial movement to starting point */
                        if(Posture_PB.PTP_6D_path_exe(path_out))
                        {
                            Desired_XYZ << path_out[0], path_out[1], path_out[2];
                            Desired_RPY << path_out[3], path_out[4], path_out[5];

                            // At initial movement to starting point set the recorded force = 0
                            LD_CFx = 0;
                            LD_CFy = 0;
                            LD_CFz = 0;

                            // KdTo0 0ToKd flag initialization
                            //(Defualt set: KdToZero_flag = true, ZeroToKd_flag = false;)
                            KdToZero_flag = true;
                            ZeroToKd_flag = false;
                        }

                        /* Tracking the generated trajectory */
                        else
                        {
                            reti = fscanf(Hand_G_playback, "%f %f %f %f %f %f %f %f %f\n", &LD_X, &LD_Y, &LD_Z, &LD_Roll, &LD_Pitch, &LD_Yaw,
                            &LD_CFx, &LD_CFy, &LD_CFz);

                            if(reti != -1)
                            {
                                Desired_XYZ << LD_X, LD_Y, LD_Z;
                                Desired_RPY << LD_Roll, LD_Pitch, LD_Yaw;
                            }
                            else
                            {
                                fclose(Hand_G_playback);
                                PB_starting_path_done_flag = false;
                                path_exe_counter = 0;
                            }
                        }

                        if(reti != -1)
                        {
                            #if Playback_mode == 0 // Position playback mode

                            for(int i=0;i<6;i++)
                            {
                                if(i<3) Desired_XYZ(i)=Cadmit_playback[i].adm_1D_control((double)Desired_XYZ(i), (double)0, (double)Contact_Rot_force(i));
                                else Desired_RPY(i-3)=Cadmit_playback[i].adm_1D_control((double)Desired_RPY(i-3), (double)0, (double)Contact_Rot_moment(i-3));
                            }

                            #elif Playback_mode == 1 // Power playback mode

                            /* [Transform the desired RPY to rotation matrix] */
                            // Used in task normal direction and contact force vector direction
                            AKin.EulerAngle2Rotation(Desired_rot,Desired_RPY);

                            /* The case of force control mode 0, 1 */
                            if((Contact_Fcon_mode == 0) || (Contact_Fcon_mode == 1))
                            {
                                #if 0
                                /* [Variable stiffness (Kd -> 0) at contact region] */
                                if(KdToZero_flag == true)
                                {
                                    KTZ_Z_init = CR_startZP + fabs(Kd_change_dist);
                                    KTZ_Z_end = CR_startZP;
                                    if(fabs(Desired_XYZ(2)-CR_startZP) <= fabs(Kd_change_dist))
                                    {
                                        Power_PB.PRamK[2] = Power_PB.PRamK[2]*exp(-KTZ_update_par*
                                        fabs((KTZ_Z_init-Desired_XYZ(2))/(KTZ_Z_end-Desired_XYZ(2))));
                                    }
                                    if((Power_PB.PRamK[2] <= KTZ_Kd_threshold) || (PPB_RTinput.PFd >= 0.01)) // Set the Kd dead-zone, PPB_RTinput.PFd != 0
                                    {
                                        Power_PB.PRamK[2] = 0; // Saturate to 0
                                        KTZ_Kd_h = KTZ_Kd_init;
                                        KdToZero_flag = false; // Terminate Kd -> 0 change
                                        ZeroToKd_flag = true;
                                        KTZ_Fd_flag = false;
                                    }
                                }
                                /* [Variable stiffness (0 -> Kd) at contact region] */
                                if((KTZ_Fd_flag == false) && (PPB_RTinput.PFd >= 0.01)) {KTZ_Fd_flag = true;}

                                if((ZeroToKd_flag == true) && (PPB_RTinput.PFd < 0.01) && (KTZ_Fd_flag == true))
                                {
                                    KTZ_Z_init = CR_endZP;
                                    KTZ_Z_end = CR_endZP + fabs(Kd_change_dist);

                                    if(fabs(Desired_XYZ(2)-CR_endZP) < fabs(Kd_change_dist))
                                    {
                                        KTZ_Kd_h = KTZ_Kd_h*exp(-KTZ_update_par*
                                        fabs((KTZ_Z_init-Desired_XYZ(2))/(KTZ_Z_end-Desired_XYZ(2))));

                                        Power_PB.PRamK[2] = KTZ_Kd_init - KTZ_Kd_h;
                                    }
                                    if(fabs(KTZ_Kd_init-Power_PB.PRamK[2]) <= KTZ_Kd_threshold) // Set the Kd dead-zone
                                    {
                                        Power_PB.PRamK[2] = KTZ_Kd_init; // Saturate to KTZ_Kd_init
                                        KdToZero_flag = false;
                                        ZeroToKd_flag = false; // Terminate 0 -> Kd change
                                        KTZ_Fd_flag = false;
                                    }
                                    // printf("Kd: %f \n", Power_PB.PRamK[2]);
                                }
                                #endif

                                #if 1 // For 2024 NIST
                                /* [Variable stiffness (Kd -> 0) at contact region] */
                                if(KdToZero_flag == true)
                                {
                                    CR_start_dist = RArm.xc-CR_start;

                                    if(CR_start_dist.norm() <= fabs(Kd_change_dist))
                                    {
                                        Power_PB.PRamK[2] = Power_PB.PRamK[2]*exp(-KTZ_update_par*
                                        (CR_start_dist.norm()+fabs(Kd_change_dist))/(CR_start_dist.norm()));
                                    }
                                    if((Power_PB.PRamK[2] <= KTZ_Kd_threshold)) // Set the Kd dead-zone, PPB_RTinput.PFd != 0
                                    {
                                        Power_PB.PRamK[2] = 0; // Saturate to 0
                                        KTZ_Kd_h = KTZ_Kd_init;
                                        KdToZero_flag = false; // Terminate Kd -> 0 change
                                        ZeroToKd_flag = true;
                                        KTZ_Fd_flag = false;
                                    }
                                    // printf("Kd: %f \n", Power_PB.PRamK[2]);
                                }
                                /* [Variable stiffness (0 -> Kd) at contact region] */
                                if((KTZ_Fd_flag == false) && (PPB_RTinput.PFd >= 0.01)) {KTZ_Fd_flag = true;}

                                if((ZeroToKd_flag == true) && (PPB_RTinput.PFd < 0.01) && (KTZ_Fd_flag == true))
                                {
                                    CR_start_dist = RArm.xc-CR_start;
                                    if(CR_start_dist.norm() <= fabs(Kd_change_dist))
                                    {
                                        KTZ_Kd_h = KTZ_Kd_h*exp(-KTZ_update_par*
                                        CR_start_dist.norm()/(CR_start_dist.norm()+fabs(Kd_change_dist)));

                                        Power_PB.PRamK[2] = KTZ_Kd_init - KTZ_Kd_h;
                                    }
                                    if(fabs(KTZ_Kd_init-Power_PB.PRamK[2]) <= KTZ_Kd_threshold) // Set the Kd dead-zone
                                    {
                                        Power_PB.PRamK[2] = KTZ_Kd_init; // Saturate to KTZ_Kd_init
                                        KdToZero_flag = false;
                                        ZeroToKd_flag = false; // Terminate 0 -> Kd change
                                        KTZ_Fd_flag = false;
                                    }
                                    // printf("Kd: %f \n", Power_PB.PRamK[2]);
                                }
                                #endif

                                /* [Variable damping at contact region] */
                                if(Contact_Fcon_mode == 1)
                                {
                                    DB_AVA_Xc = Power_PB.PU3.transpose()*Power_PB.PXc_0;
                                    DB_AVA_Xc_dot = (DB_AVA_Xc - DB_AVA_Xc_pre)/dt;
                                    DB_AVA_Xc_pre = DB_AVA_Xc;

                                    if(PPB_RTinput.PFd >= 0.01)
                                    {
                                        /* Step 1 : Damping update with update rule - only surf. direction */
                                        DB_AVA_phi += DB_AVA_sigma*(PPB_RTinput.PFd - PPB_surfN_Fext)/DB_AVA_Dd_init; // it's ok to use the "PPB_surfN_Fext" as a previous external force

                                        /* Damping variation */
                                        Power_PB.PRamD[2] = DB_AVA_Dd_init + DB_AVA_phi*DB_AVA_Dd_init/DB_AVA_Xc_dot;

                                        /* Damping saturation part */
                                        if(Power_PB.PRamD[2] >= DB_AVA_Dsature[1])
                                        {Power_PB.PRamD[2] = DB_AVA_Dsature[1];}
                                        else if(Power_PB.PRamD[2] <= DB_AVA_Dsature[0])
                                        {Power_PB.PRamD[2] = DB_AVA_Dsature[0];}

                                    }
                                    else {Power_PB.PRamD[2] = DB_AVA_Dd_init;}
                                }

                                /* Data recording */
                                if((PPB_RTinput.PFd >= 0.01) || (Contact_Rot_force.norm() >= 2.0)) // In the case of over the force threshold
                                {
                                    if(EXPdata1_switch == 1) // 1: Recording
                                    {
                                        fprintf(EXPdata1,"%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
                                        PPB_RTinput.PFd, PPB_surfN_Fext, DB_AVA_Xc_dot, Power_PB.PRamK[2],
                                        Power_PB.PU1(0),Power_PB.PU1(1),Power_PB.PU1(2),
                                        Power_PB.PU3(0),Power_PB.PU3(1),Power_PB.PU3(2),
                                        Desired_rot(0,2),Desired_rot(1,2),Desired_rot(2,2));
                                    }
                                }
                            }

                            /* The case of force control mode 2 - variable stiffness */
                            else if(Contact_Fcon_mode == 2)
                            {
                                DB_AVA_Xc = Power_PB.PU3.transpose()*Power_PB.PXc_0; // Xc calculation
                                DB_AVA_Xr = Power_PB.PU3.transpose()*Desired_XYZ; // Xr calculation

                                if((PPB_RTinput.PFd >= 0.01) || (Contact_Rot_force.norm() >= 2.0)) // In the case of over the force threshold
                                {
                                    /* Step 1 : Stiffness update with update rule - only surf. direction */
                                    DB_AVA_phi += DB_AVA_sigma*(PPB_RTinput.PFd - PPB_surfN_Fext)/DB_AVA_Kd_init; // it's ok to use the "PPB_surfN_Fext" as a previous external force

                                    /* Stiffness variation */
                                    if((DB_AVA_Xc - DB_AVA_Xr) != 0)
                                    {Power_PB.PRamK[2] = DB_AVA_Kd_init + DB_AVA_phi*DB_AVA_Kd_init/(DB_AVA_Xc - DB_AVA_Xr);}

                                    /* Stiffness saturation part */
                                    if(Power_PB.PRamK[2] >= DB_AVA_Ksature[1])
                                    {Power_PB.PRamK[2] = DB_AVA_Ksature[1];}
                                    else if(Power_PB.PRamK[2] <= DB_AVA_Ksature[0])
                                    {Power_PB.PRamK[2] = DB_AVA_Ksature[0];}

                                    /* Data recording */
                                    if(EXPdata1_switch == 1) // 1: Recording
                                    {
                                        fprintf(EXPdata1,"%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
                                        PPB_RTinput.PFd, PPB_surfN_Fext, DB_AVA_Xc, DB_AVA_Xc_dot, Power_PB.PRamK[2],
                                        Power_PB.PU1(0),Power_PB.PU1(1),Power_PB.PU1(2),
                                        Power_PB.PU3(0),Power_PB.PU3(1),Power_PB.PU3(2),
                                        Desired_rot(0,2),Desired_rot(1,2),Desired_rot(2,2));
                                    }
                                }
                                std::vector<double> Mon1_input_data = {Power_PB.PRamM[2], Power_PB.PRamD[2], Power_PB.PRamK[2]};
                                std::vector<double> Mon2_input_data = {PPB_RTinput.PFd, PPB_surfN_Fext};

                                std::string Mon1_description = "Mass, Damping, Stiffness";
                                std::string Mon2_description = "Contact force, Surface normal force";

                                AdaptiveK_msg_->Mon1_publish(Mon1_input_data,Mon1_description,true);
                                AdaptiveK_msg_->Mon2_publish(Mon2_input_data,Mon2_description,true);

                            }
                            /* The case of force control mode 3 - Fuzzy variable stiffness */
                            else if(Contact_Fcon_mode == 3)
                            {
                                DB_AVA_Xc = Power_PB.PU3.transpose()*Power_PB.PXc_0; // Xc calculation -> Surf. normal direction
                                DB_AVA_Xr = Power_PB.PU3.transpose()*Desired_XYZ; // Xr calculation -> Surf. normal direction

                                /* Force error, Force error dot calculation - only surf. direction */
                                Fuzzy_F_error = PPB_RTinput.PFd - PPB_surfN_Fext;
                                Fuzzy_F_error_dot = (Fuzzy_F_error-Fuzzy_F_Perror)/dt;
                                Fuzzy_F_Perror = Fuzzy_F_error;

                                if((PPB_RTinput.PFd >= 0.01) || (Contact_Rot_force.norm() >= 2.0)) // In the case of over the force threshold
                                {
                                    /* Fuzzy DelK, DelSigma update */
                                    DB_AVA_sigma += FAK.FAAC_DelSigma_Cal(Fuzzy_F_error,Fuzzy_F_error_dot);
                                    DB_AVA_Kd_init += FAK.FAAC_DelK_Cal(Fuzzy_F_error,Fuzzy_F_error_dot);

                                    /*** DB_AVA_sigma, DB_AVA_Kd_init saturation ***/
                                    /* DB_AVA_Kd_init saturation - 이부분 좀 생각해보자.... */
                                    if(DB_AVA_Kd_init >= DB_AVA_Ksature[1])
                                    {DB_AVA_Kd_init = DB_AVA_Ksature[1];}
                                    else if(DB_AVA_Kd_init <= 10.0)
                                    {DB_AVA_Kd_init = 10.0;}
                                    /* DB_AVA_sigma saturation - 이부분 좀 생각해보자.... */
                                    if(DB_AVA_sigma >= 1.0)
                                    {DB_AVA_sigma = 1.0;}
                                    else if(DB_AVA_sigma <= 0.001)
                                    {DB_AVA_sigma = 0.001;}

                                    /* Stiffness update with update rule - only surf. direction */
                                    DB_AVA_phi += DB_AVA_sigma*(PPB_RTinput.PFd - PPB_surfN_Fext)/DB_AVA_Kd_init; // it's ok to use the "PPB_surfN_Fext" as a previous external force

                                    /* Stiffness variation - only surf. direction */
                                    if((DB_AVA_Xc - DB_AVA_Xr) != 0)
                                    {Power_PB.PRamK[2] = DB_AVA_Kd_init + DB_AVA_phi*DB_AVA_Kd_init/(DB_AVA_Xc - DB_AVA_Xr);}

                                    /* Stiffness saturation part */
                                    if(Power_PB.PRamK[2] >= DB_AVA_Ksature[1])
                                    {Power_PB.PRamK[2] = DB_AVA_Ksature[1];}
                                    else if(Power_PB.PRamK[2] <= DB_AVA_Ksature[0])
                                    {Power_PB.PRamK[2] = DB_AVA_Ksature[0];}

                                    /* Data recording */
                                    if(EXPdata1_switch == 1) // 1: Recording
                                    {
                                        fprintf(EXPdata1,"%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
                                        PPB_RTinput.PFd, PPB_surfN_Fext, DB_AVA_Xc_dot, Power_PB.PRamK[2],
                                        Power_PB.PU1(0),Power_PB.PU1(1),Power_PB.PU1(2),
                                        Power_PB.PU3(0),Power_PB.PU3(1),Power_PB.PU3(2),
                                        Desired_rot(0,2),Desired_rot(1,2),Desired_rot(2,2));
                                    }
                                }
                            }
                            /* The case of force control mode 4 - Fuzzy variable mass & damping */
                            else if(Contact_Fcon_mode == 4)
                            {
                                DB_AVA_Xc = Power_PB.PU3.transpose()*Power_PB.PXc_0; // Xc calculation -> Surf. normal direction
                                DB_AVA_Xr = Power_PB.PU3.transpose()*Desired_XYZ; // Xr calculation -> Surf. normal direction
                                DB_AVA_X = Power_PB.PU3.transpose()*PPB_RTinput.PX; // X calculation -> Surf. normal direction

                                // /* Fuzzy MDK update */
                                // FAMD.FAAC_Kd_variation(PPB_RTinput.PFd, Power_PB.PRamK[2], Power_PB.PXc_0, &Power_PB.PRamK[2]);
                                // FAMD.FAAC_MD_MainCal(PPB_RTinput.PFd, PPB_surfN_Fext, &Power_PB.PRamM[2], &Power_PB.PRamD[2]);

                                /* Three Step FAAC MDK update */
                                auto TSFAAC_MDK = FAAC3step.FAAC_MDKob_RUN(Power_PB.PTankE, PPB_surfN_Fext, PPB_RTinput.PFd, DB_AVA_Xc, DB_AVA_X);
                                Power_PB.PRamM[2] = TSFAAC_MDK.Mass;
                                Power_PB.PRamD[2] = TSFAAC_MDK.Damping;
                                Power_PB.PRamK[2] = TSFAAC_MDK.Stiffness;

                                if(PPB_RTinput.PFd >= 0.01) // In the case of over the force threshold
                                {
                                    /* Data recording */
                                    if(EXPdata1_switch == 1) // 1: Recording
                                    {
                                        fprintf(EXPdata1,"%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
                                        PPB_RTinput.PFd, PPB_surfN_Fext, DB_AVA_Xc_dot,
                                        Power_PB.PRamM[2],Power_PB.PRamD[2],Power_PB.PRamK[2],
                                        Power_PB.PU1(0),Power_PB.PU1(1),Power_PB.PU1(2),
                                        Power_PB.PU3(0),Power_PB.PU3(1),Power_PB.PU3(2),
                                        Desired_rot(0,2),Desired_rot(1,2),Desired_rot(2,2));
                                    }
                                }

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
                            else
                            {
                                printf("Wrong control mode was selected \n");
                                printf("Check the 'NRS_Fcon_setting.yaml' \n");
                                ////// raiseFlag(0); // Stop the servo & program
                            }

                            /* [Power playback real-time input upload] */

                            /* Position */
                            PPB_RTinput.PXr = Desired_XYZ; // Reference posi
                            PPB_RTinput.PX = RArm.xc; // Actual posi
                            PPB_RTinput.PFext = Contact_Rot_force; // Actual contact force
                            PPB_RTinput.PRtz << -Desired_rot(0,2),-Desired_rot(1,2),-Desired_rot(2,2); // Tool z-axis (if EE is heading to surf -> PRtz = [0,0,1])
                            PPB_RTinput.P_Tool_Rot = Desired_rot; // Tool rotation matrix

                            /* Surface normal force monitoring start */
                            PPB_surfN_Fext = Power_PB.PU3.transpose()*Contact_Rot_force;
                            YSurfN_Fext_msg_.data = PPB_surfN_Fext;
                            YSurfN_Fext_pub_->publish(YSurfN_Fext_msg_); //// YSurfN_Fext_pub.publish(YSurfN_Fext_msg);
                            /* Surface normal force monitoring end */

                            /* Desired contact force vector */

                            PPB_RTinput.PFd = LD_CFz;

                            /* Orientation */
                            PPB_RTinput.OXr = Desired_RPY; // Reference ori
                            PPB_RTinput.OX = RArm.thc; // Actual ori
                            PPB_RTinput.OFd = 0; // Desired contact force, But desired moment == 0
                            PPB_RTinput.OFext = Contact_Rot_moment; // Actual contact moment, 0이여야 되는지 확인 필요...

                            /* [Power playback execution] - position only */
                            if(Power_PB.playback_start(PPB_RTinput)) Desired_XYZ = Power_PB.PXc_0;

                            #endif

                            // fprintf(path_recording_pos,"%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
                            // Power_PB.PX(0),Power_PB.PX(1),Power_PB.PX(2),
                            // Power_PB.PXr(0),Power_PB.PXr(1),Power_PB.PXr(2),Power_PB.PTankE,
                            // Contact_Rot_force(0),Contact_Rot_force(1),Contact_Rot_force(2));
                            fprintf(path_recording_pos,"%10f %10f %10f %10f %10f %10f\n",
                            PPB_RTinput.PFd,Contact_Rot_force(2),
                            Power_PB.PTankE,Desired_rot(0,2), Desired_rot(1,2), Desired_rot(2,2));

                            #if 0
                            // Final output monitoring
                            fprintf(path_recording_pos,"%10f %10f %10f %10f %10f %10f\n",
                            Desired_XYZ(0),Desired_XYZ(1),Desired_XYZ(2),Desired_RPY(0),Desired_RPY(1),Desired_RPY(2));
                            #endif

                            AKin.EulerAngle2Rotation(Desired_rot,Desired_RPY);
                            RArm.Td << Desired_rot(0,0),Desired_rot(0,1),Desired_rot(0,2),Desired_XYZ(0),
                                       Desired_rot(1,0),Desired_rot(1,1),Desired_rot(1,2),Desired_XYZ(1),
                                       Desired_rot(2,0),Desired_rot(2,1),Desired_rot(2,2),Desired_XYZ(2),
                                       0               ,0               ,0               ,1;

                            /* [Inverse kinematics] */
                            #if TCP_standard == 0
                            AKin.InverseK_min(&RArm); // input: Td , output: qd
                            #elif TCP_standard == 1
                            AKin.Ycontact_InverseK_min(&RArm);
                            #endif
                        }
                        else
                        {
                            KdToZero_flag = true;
                            ZeroToKd_flag = false;

                            if(PB_iter_cmd > PB_iter_cur) // iterate the playback
                            {
                                UR10e_mode_msg_.data = Playback_mode_cmd;
                                PB_iter_cur++;
                            }
                            else // stop the playback
                            {
                                UR10e_mode_msg_.data = Motion_stop_cmd;
                                PB_iter_cur = 1; // 1 is right
                            }
                            sprintf(Playback_iteration,"playback iter:(cur: %d/tot: %d)",PB_iter_cur,PB_iter_cmd);
                            memcpy(message_status,Playback_iteration,sizeof(Playback_iteration));
                            UR10e_mode_pub_->publish(UR10e_mode_msg_); //// UR10e_mode_pub.publish(UR10e_mode_msg);
                        }
                    }

                    /* [Transmit the command to robot] */
                    #if Actual_mode == 0 // test mode
                    joint_q = {Init_qc(0), Init_qc(1), Init_qc(2), Init_qc(3), Init_qc(4), Init_qc(5)};
                    #elif Actual_mode == 1 // actual control mode

                    printf("ctrl: %d, pre_ctrl: %d \n", ctrl, pre_ctrl);
                    // joint_q = {RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5)};

                    /////////////////////////////////////////////////////
                    // joint_commands_pub_->publish(RArm.qd); // Add on 2025.07.09
                    // RArm.qd를 JointState 메시지로 변환하여 publish
                    joint_state_.header.stamp = node_->now();  // timestamp 갱신

                    for (int i = 0; i < 6; ++i) {
                        joint_state_.position[i] = RArm.qd(i);
                    }

                    joint_commands_pub_->publish(joint_state_);
                    //////////////////////////////////////////////////////

                    // joint_q = {RArm.qd(0), RArm.qd(1), RArm.qd(2), RArm.qd(3), RArm.qd(4), RArm.qd(5)};

                    #endif

                    //// rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
                    //// rtde_control.waitPeriod(t_start);

                    pre_ctrl = ctrl; // for ctrl mode switching detection
                    // continue;
                }

                else{
                    speedmode = 0;
                    RArm.qd = RArm.qc;
                    RArm.qt = RArm.qc;
                    RArm.dqc << 0,0,0,0,0,0;
                    pause_cnt=0;
                }

                double errsum=0;
                double maxerr=0;
                for(int i=0;i<6;i++){
                    double err=RArm.qd(i)-RArm.qc(i);
                    if(maxerr<fabs(err)){
                        maxerr=fabs(err);
                    }
                }

                // double maxerrmin = 0.001;

                //// rtde_control.waitPeriod(t_start);
            // }
            // break;

        // default:
        //     break;
        // }

        // std::cout << "Control interrupted!" << std::endl;

        //// rtde_control.servoStop();
        //// rtde_control.stopScript();
    }
    catch(std::exception& e)
    {
        std::cerr << "error: " << e.what() << "\n";
        ////// raiseFlag(0);
    }
    catch(...)
    {
        std::cerr << "Exception of unknown type!\n";
    }

    // return 0;
}
