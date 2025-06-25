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
    UR10e_mode_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/Yoon_UR10e_mode", 20,
        std::bind(&JointControl::cmdModeCallback, this, std::placeholders::_1));

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

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&JointControl::CalculateAndPublishJoint, this));
}
JointControl::~JointControl() {}

void JointControl::cmdModeCallback(std_msgs::msg::UInt16::SharedPtr msg)
{
    mode_cmd = msg->data;

    if(mode_cmd == Joint_control_mode_cmd) {} // Joint angle control mode (wit0.0016,0.0016,0.0016

    else if(mode_cmd == EE_Posture_control_mode_cmd) // E.E. posture control mode (with path blender)
	{
        #if 0
        ctrl = 1;

        double Tar_pos[] = {0.1,0.1}; // Target position array, unit: m (this value is applied as absolute value)
        double Tar_vel[] = {-0.02,-0.02}; // Target velocity array, unit: m/s (the direction is controled by cmd velocity)
        double Waiting_time[] = {0,0}; // Waiting time, unit : s, 0이 아닌 값을 넣으려면 동일 array 위치의 pos와 vel은 0 

        TCP_path_start << RArm.xc(0),RArm.xc(1),RArm.xc(2),RArm.thc(0),RArm.thc(1),RArm.thc(2);

        Path_point_num = path_planning.Single_blended_path(Tar_pos,Tar_vel,Waiting_time,(int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));
        if(Path_point_num != -1)
        {
            memcpy(message_status,path_gen_done,sizeof(path_gen_done));
            path_done_flag = true;
        }
        #endif
	}
    else if(mode_cmd == Hand_guiding_mode_cmd)
	{
        ctrl = 2;
        memcpy(message_status,Hand_guiding_mode,sizeof(Hand_guiding_mode));
	}
	else if(mode_cmd == Continuous_reording_start) // data recording flag on
	{
        path_recording_flag = true;
        // Hand_G_recording = fopen("/home/gene/catkin_ws/src/rtde_handarm/src/Hand_G_recording.txt","wt");
        auto Hand_G_recording_path = NRS_recording["Hand_G_recording"].as<std::string>();
        Hand_G_recording = fopen(Hand_G_recording_path.c_str(),"wt");
        memcpy(message_status,Data_recording_on,sizeof(Data_recording_on));

	}
	else if(mode_cmd == Continusous_recording_end) // data recording flag off
	{
        path_recording_flag = false;
        fclose(Hand_G_recording);
        memcpy(message_status,Data_recording_off,sizeof(Data_recording_off));
	}

    /* Way point teaching with "Teaching handle" [start] */
    else if(mode_cmd == Descrete_reording_start) // Recording the descrete way points start
    {
        if(Num_RD_points != 0)
        {
            Inst_RD_points = Decr_RD_points;
            Decr_RD_points.resize(Num_RD_points+1,6);
            Decr_RD_points.topRows(Num_RD_points) = Inst_RD_points;
        }
        else{Decr_RD_points.topRows(Num_RD_points+1) = Inst_RD_points;}

        // Decr_RD_points.bottomRows(1) << Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2), Desired_RPY(0), Desired_RPY(1), Desired_RPY(2);
        Decr_RD_points.bottomRows(1) << RArm.xc(0), RArm.xc(1), RArm.xc(2), RArm.thc(0), RArm.thc(1), RArm.thc(2);
        Num_RD_points ++;

        sprintf(Saved_way_point,"Saved way point: %d",Num_RD_points);
        memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
        std::cout << "\n" <<Decr_RD_points << std::endl;
    }

    else if(mode_cmd == Descrete_recording_save) // Save the recorded decrete way points
    {
        /* Open the text file */
        auto Descre_P_recording_path = NRS_recording["Descre_P_recording"].as<std::string>();
        Descre_P_recording = fopen(Descre_P_recording_path.c_str(),"wt");

        /* Save the data to text file */
        for(int i = 0; i<Num_RD_points;i++)
        {
            // Last space is for desired force
            fprintf(Descre_P_recording,"%10f %10f %10f %10f %10f %10f %10f\n",
            Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2), Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), (double)0.0);
        }
        /* Close the text file */
        fclose(Descre_P_recording);
        Num_RD_points = 0;
        Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
        Inst_RD_points = Eigen::MatrixXd::Zero(1,6);

        printf("\n Descrete points was save to txt file \n");
    }
    /* Way point teaching with "Teaching handle" [end] */

    /* Way point teaching with "VR tracker" [start] */
    else if(mode_cmd == VRTeac_reording_start) // Recording the descrete way points start
    {
        if(Num_RD_points != 0)
        {
            Inst_RD_points = Decr_RD_points;
            Decr_RD_points.resize(Num_RD_points+1,6);
            Decr_RD_points.topRows(Num_RD_points) = Inst_RD_points;
        }
        else{Decr_RD_points.topRows(Num_RD_points+1) = Inst_RD_points;}

        // In here we must save the VR quaternion to X,Y,Z, Roll, Pitch, Yaw

        Decr_RD_points.bottomRows(1) <<
        VR_CalPoseRPY(0), VR_CalPoseRPY(1), VR_CalPoseRPY(2), VR_CalPoseRPY(3), VR_CalPoseRPY(4), VR_CalPoseRPY(5);
        Num_RD_points ++;

        sprintf(Saved_way_point,"Saved way point: %d",Num_RD_points);
        memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
        std::cout << "\n" <<Decr_RD_points << std::endl;
    }

    else if(mode_cmd == VRTeac_recording_save) // Save the recorded decrete way points
    {
        /* Open the text file */
        auto Descre_P_recording_path = NRS_recording["Descre_P_recording"].as<std::string>();
        Descre_P_recording = fopen(Descre_P_recording_path.c_str(),"wt");

        /* Save the data to text file */
        for(int i = 0; i<Num_RD_points;i++)
        {
            // Last space is for desired force
            fprintf(Descre_P_recording,"%10f %10f %10f %10f %10f %10f %10f\n",
            Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2), Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), (double)0.0);
        }
        /* Close the text file */
        fclose(Descre_P_recording);
        Num_RD_points = 0;
        Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
        Inst_RD_points = Eigen::MatrixXd::Zero(1,6);

        printf("\n Descrete points was save to txt file \n");
    }
    /* Way point teaching with "VR tracker" [end] */

//     ////////////////////////////////////////////////////////////여기////////////////////
//     /* Continuous teaching with "VR tracker" [start] */
//     else if(mode_cmd == VRConTeaching_mode) // Recording the descrete way points start
//     {
//         if(Num_RD_points != 0)
//         {
//             Inst_RD_points = Decr_RD_points;
//             Decr_RD_points.resize(Num_RD_points+1,6);
//             Decr_RD_points.topRows(Num_RD_points) = Inst_RD_points;
//         }
//         else{Decr_RD_points.topRows(Num_RD_points+1) = Inst_RD_points;}

//         // In here we must save the VR quaternion to X,Y,Z, Roll, Pitch, Yaw

//         Decr_RD_points.bottomRows(1) <<
//         VR_CalPoseRPY(0), VR_CalPoseRPY(1), VR_CalPoseRPY(2), VR_CalPoseRPY(3), VR_CalPoseRPY(4), VR_CalPoseRPY(5);
//         Num_RD_points ++;

//         sprintf(Saved_way_point,"Saved way point: %d",Num_RD_points);
//         memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
//         std::cout << "\n" <<Decr_RD_points << std::endl;
//     }

//     else if(mode_cmd == VRConTeac_recording_save) // Save the recorded decrete way points
//     {
//         /* Open the text file */
//         auto Descre_P_recording_path = NRS_recording["Descre_P_recording"].as<std::string>();
//         Descre_P_recording = fopen(Descre_P_recording_path.c_str(),"wt");

//         /* Save the data to text file */
//         for(int i = 0; i<Num_RD_points;i++)
//         {
//             // Last space is for desired force
//             fprintf(Descre_P_recording,"%10f %10f %10f %10f %10f %10f %10f\n",
//             Decr_RD_points(i,0), Decr_RD_points(i,1), Decr_RD_points(i,2), Decr_RD_points(i,3), Decr_RD_points(i,4), Decr_RD_points(i,5), (double)0.0);
//         }
//         /* Close the text file */
//         fclose(Descre_P_recording);
//         Num_RD_points = 0;
//         Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
//         Inst_RD_points = Eigen::MatrixXd::Zero(1,6);

//         printf("\n Descrete points was save to txt file \n");
//     }
//     /* Continuous teaching with "VR tracker" [end] */

// ////////////////////////////////////////////////////////////까지//////


    /* VR calibrarion point save with "Teaching handle" [start] */
    else if(mode_cmd == VRCali_reording_start) // Recording the descrete points start
    {
        // [Save the actual robot EE data]
        // Note: In VR calibration, we save the actual TF values (Not desired)
        if(Num_EE_points != 0)
        {
            Inst_EE_points = Decr_EE_points;
            Decr_EE_points.resize(Num_EE_points+1,12);
            Decr_EE_points.topRows(Num_EE_points) = Inst_EE_points;
        }
        else{Decr_EE_points.topRows(Num_EE_points+1) = Inst_EE_points;}

        Decr_EE_points.bottomRows(1) <<
        RArm.Tc(0,0),RArm.Tc(1,0),RArm.Tc(2,0), // Rx
        RArm.Tc(0,1),RArm.Tc(1,1),RArm.Tc(2,1), // Ry
        RArm.Tc(0,2),RArm.Tc(1,2),RArm.Tc(2,2), // Rz
        RArm.Tc(0,3),RArm.Tc(1,3),RArm.Tc(2,3); // P

        Num_EE_points ++;

        sprintf(Saved_way_point,"Saved cali. points: %d",Num_EE_points);
        memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
        std::cout << "\n" <<Decr_EE_points << std::endl;

        // [Save the VR quaternion data]
        if(Num_VR_points != 0)
        {
            Inst_VR_points = Decr_VR_points;
            Decr_VR_points.resize(Num_VR_points+1,7);
            Decr_VR_points.topRows(Num_VR_points) = Inst_VR_points;
        }
        else{Decr_VR_points.topRows(Num_VR_points+1) = Inst_VR_points;}

        Decr_VR_points.bottomRows(1) <<
        VR_pose[0],VR_pose[1],VR_pose[2], // VR position data load
        VR_pose[3],VR_pose[4],VR_pose[5],VR_pose[6]; // VR orientation data load

        Num_VR_points ++;

        sprintf(Saved_way_point,"Saved VR points: %d",Num_VR_points);
        memcpy(message_status,Saved_way_point,sizeof(Saved_way_point));
        std::cout << "\n" <<Decr_VR_points << std::endl;
    }

    else if(mode_cmd == VRCali_recording_save) // Save the recorded decrete points
    {
        // [Save the actual robot EE data]
        /* Open the text file */
        auto VRCali_UR10CB_EE_path = NRS_recording["VRCali_UR10CB_EE"].as<std::string>();
        VRCali_UR10CB_EE = fopen(VRCali_UR10CB_EE_path.c_str(),"wt");

        /* Save the data to text file */
        for(int i = 0; i<Num_EE_points;i++)
        {
            // Last space is for desired force
            fprintf(VRCali_UR10CB_EE,"%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n", // 12 cols.(9 rot. + 3 posi.)
            Decr_EE_points(i,0), Decr_EE_points(i,1),  Decr_EE_points(i,2),
            Decr_EE_points(i,3), Decr_EE_points(i,4),  Decr_EE_points(i,5),
            Decr_EE_points(i,6), Decr_EE_points(i,7),  Decr_EE_points(i,8),
            Decr_EE_points(i,9), Decr_EE_points(i,10), Decr_EE_points(i,11));
        }
        /* Close the text file */
        fclose(VRCali_UR10CB_EE);
        Num_EE_points = 0;
        Decr_EE_points = Eigen::MatrixXd::Zero(1,12);
        Inst_EE_points = Eigen::MatrixXd::Zero(1,12);

        printf("\n Descrete EE points was save to txt file \n");

        // [Save the VR quaternion data]
        /* Open the text file */
        auto VRCali_UR10CB_VR_path = NRS_recording["VRCali_UR10CB_VR"].as<std::string>();
        VRCali_UR10CB_VR = fopen(VRCali_UR10CB_VR_path.c_str(),"wt");

        /* Save the data to text file */
        for(int i = 0; i<Num_VR_points;i++)
        {
            // Last space is for desired force
            fprintf(VRCali_UR10CB_VR,"%10f %10f %10f %10f %10f %10f %10f\n",
            Decr_VR_points(i,0), Decr_VR_points(i,1), Decr_VR_points(i,2), Decr_VR_points(i,3), Decr_VR_points(i,4), Decr_VR_points(i,5), Decr_VR_points(i,6));
        }
        /* Close the text file */
        fclose(VRCali_UR10CB_VR);
        Num_VR_points = 0;
        Decr_VR_points = Eigen::MatrixXd::Zero(1,7);
        Inst_VR_points = Eigen::MatrixXd::Zero(1,7);

        printf("\n Cali points was save to txt file \n");
    }
    /* VR calibrarion point save with "Teaching handle" [end] */

    else if(mode_cmd == Playback_mode_cmd) // Power playback
	{
        /*** Parameter upload form yaml ***/

        /* Trajectory directory load */
        auto Hand_G_recording_path = NRS_recording["Hand_G_recording"].as<std::string>();

        /* Contact admittance parameter laod */
        Power_PB.PRamM[0]= NRS_Fcon_setting["ContactDesiredMass"]["LamdaM1"].as<double>();
        Power_PB.PRamM[1]= NRS_Fcon_setting["ContactDesiredMass"]["LamdaM2"].as<double>();
        Power_PB.PRamM[2]= NRS_Fcon_setting["ContactDesiredMass"]["LamdaM3"].as<double>();

        Power_PB.PRamD[0]= NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD1"].as<double>();
        Power_PB.PRamD[1]= NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD2"].as<double>();
        Power_PB.PRamD[2]= NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3"].as<double>();

        Power_PB.PRamK[0]= NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK1"].as<double>();
        Power_PB.PRamK[1]= NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK2"].as<double>();
        Power_PB.PRamK[2]= NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>();

        /* Data load */
        float LD_X,LD_Y,LD_Z,LD_Roll,LD_Pitch,LD_Yaw,LD_CFx,LD_CFy,LD_CFz; // Loaded XYZRPY
        int reti;

        Hand_G_playback = fopen(Hand_G_recording_path.c_str(),"rt"); // Open the trajectory file

        for(int i = 0; i<3;i++) // For safe data acquisition
        {
            reti = fscanf(Hand_G_playback, "%f %f %f %f %f %f %f %f %f \n", &LD_X, &LD_Y, &LD_Z, &LD_Roll, &LD_Pitch, &LD_Yaw,
            &LD_CFx, &LD_CFy, &LD_CFz); // Get the starting point
        }
        printf("%f %f %f %f %f %f %f %f %f\n", LD_X, LD_Y, LD_Z, LD_Roll, LD_Pitch, LD_Yaw, LD_CFx, LD_CFy, LD_CFz);

        /* Trajectory generation to start point */

        double Linear_travel_vel = 0.03; // m/s
        double Linear_travel_time;
        double Tar_pos[6] = {LD_X,LD_Y,LD_Z,LD_Roll,LD_Pitch,LD_Yaw};
        double Init_pos[6] = {RArm.xc(0),RArm.xc(1),RArm.xc(2),RArm.thc(0),RArm.thc(1),RArm.thc(2)};

        Linear_travel_time = sqrt(pow(Init_pos[0]-Tar_pos[0],2)+pow(Init_pos[1]-Tar_pos[1],2)+pow(Init_pos[2]-Tar_pos[2],2))/Linear_travel_vel;
        if(Linear_travel_time < 3) Linear_travel_time = 3;

        PB_starting_path_done_flag = Posture_PB.PTP_6D_path_init(Init_pos, Tar_pos, Linear_travel_time);

        printf("Playback init path generation done \n");
        // path_recording_pos = fopen("/home/gene/catkin_ws/src/rtde_handarm/src/test_path.txt","wt");
        auto test_path_path = NRS_recording["test_path"].as<std::string>();
        path_recording_pos = fopen(test_path_path.c_str(),"wt");
        memcpy(message_status,ST_path_gen_done,sizeof(ST_path_gen_done));

        #if Playback_mode == 1
        /* Power playback initialization */
        Power_PB.playback_init(RArm.xc, RArm.thc);
        #endif

        ctrl = 3;
	}
    else if(mode_cmd == Motion_stop_cmd) // Motion stop
	{
        ctrl = 0;
        memcpy(message_status,Motion_stop_mode,sizeof(Motion_stop_mode));
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

void JointControl::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // std::lock_guard<std::mutex> lock(joint_state_mutex);
    // latest_joint_state = *msg;
}

void JointControl::CalculateAndPublishJoint()
{
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
	////// getActualQ();

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
    // spring mode control parameter
    VectorXd Hspring_mode_init_pos(6);

    printf("While loop start\n");

    printf("MODE SELECT: Select Your Task\n");
    printf("1 : For Servoj,  q : Quit    \n");

    key_MODE=getchar();

    /* Recording file open */
    auto test_path_joint_path = NRS_recording["test_path_joint"].as<std::string>();
    path_recording_joint = fopen(test_path_joint_path.c_str(),"wt");

    auto EXPdata1_path = NRS_recording["EXPdata1_path"].as<std::string>();
    EXPdata1 = fopen(EXPdata1_path.c_str(),"wt");






}
