#include "JointControl.h"
#include <iostream>
#include <memory>

JointControl::JointControl()
: Node("joint_control_node")
{
    // Publishers
    YSurfN_Fext_pub_ = this->create_publisher<std_msgs::msg::Float64>("YSurfN_Fext", 20);
    UR10e_mode_pub_ = this->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 20);
    UR10_Jangle_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_Jangle", 20);
    UR10_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_pose", 20);
    UR10_wrench_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("UR10_wrench", 20);

    // Subscribers
    // UR10e_mode_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
    //     "/Yoon_UR10e_mode", 20,
    //     std::bind(&JointControl::cmdModeCallback, this, std::placeholders::_1));

    PB_iter_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/Yoon_PbNum_cmd", 100,
        std::bind(&JointControl::PbIterCallback, this, std::placeholders::_1));

    joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/yoon_UR10e_joint_cmd", 100,
        std::bind(&JointControl::JointCmdCallback, this, std::placeholders::_1));

    VR_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vive/pos0", 100,
        std::bind(&JointControl::VRdataCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", rclcpp::SensorDataQoS(),
        std::bind(&JointControl::JointStateCallback, this, std::placeholders::_1));

    // // Timer
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(2),
    //     std::bind(&JointControl::CalculateAndPublishJoint, this));
}
JointControl::~JointControl() {}


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


void JointControl::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // std::lock_guard<std::mutex> lock(joint_state_mutex);
    // latest_joint_state = *msg;
}



void JointControl::CalculateAndPublishJoint()
{
    // 작업 수행에 따라 ctrl 값을 기준으로 제어 분기 가능
    // ex) if(ctrl == 1) { ... }
}
