#include "HbuttonCmd.h"

//// HbuttonCmd::HbuttonCmd(const rclcpp::Node::SharedPtr& node, int loop_rate)
//// : Node("hbutton_node"), node_(node), loop_rate(loop_rate)
HbuttonCmd::HbuttonCmd()
: Node("hbutton_node"), loop_rate(100.0), fin1(NRS_Record_Printing_loc), fin2(NRS_Fcon_desired_loc)
{
    ////// 1. UART init ///////
#if (Handle_OnOff == 1)
    Yuart = new Yoon_UART("/dev/ttyACM0", 115200);
#endif

    ////// 2. YAML file load ///////
    NRS_recording = YAML::Load(fin1);
    NRS_Fcon_desired = YAML::Load(fin2);

    ////// 3. ROS Publishers ///////
    yoon_mode_pub = this->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 20);
    PbNum_command_pub = this->create_publisher<std_msgs::msg::UInt32>("Yoon_PbNum_cmd", 20);
    Clicked_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("clicked_point", 20);

    ////// 4. ROS Subscriber ///////
    VRPose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pos_cal_rviz", 20,
        std::bind(&HbuttonCmd::VRPose_Callback, this, std::placeholders::_1));

    ////// 5. ROS Service Servers ///////
    Aidin_gui_srv1 = this->create_service<std_srvs::srv::Empty>(
        "teaching_mode",
        std::bind(&HbuttonCmd::SRV1_Handle, this, std::placeholders::_1, std::placeholders::_2));

    Aidin_gui_srv3 = this->create_service<std_srvs::srv::Empty>(
        "save_waypoint",
        std::bind(&HbuttonCmd::SRV3_Handle, this, std::placeholders::_1, std::placeholders::_2));

    Aidin_gui_srv4 = this->create_service<std_srvs::srv::Empty>(
        "trajectory_generation",
        std::bind(&HbuttonCmd::SRV4_Handle, this, std::placeholders::_1, std::placeholders::_2));

    Aidin_gui_srv11 = this->create_service<std_srvs::srv::Empty>(
        "Iteration_set",
        std::bind(&HbuttonCmd::SRV11_Handle, this, std::placeholders::_1, std::placeholders::_2));

    Aidin_gui_srv12 = this->create_service<std_srvs::srv::Empty>(
        "Playback_execution",
        std::bind(&HbuttonCmd::SRV12_Handle, this, std::placeholders::_1, std::placeholders::_2));

    ////// 6. State & Mode Initialization ///////
    #if (TEACHING_MODE == 0)
        #if (Handle_OnOff == 0)
            current_status = "Stanby mode - No Handle";
            mode0 = "Stanby mode - No Handle";
            mode1 = "Teaching mode - No Handle";
        #elif (Handle_OnOff == 1)
            current_status = "Handle stanby mode";
            mode0 = "Handle stanby mode";
            mode1 = "Handle-guiding mode";
        #endif
    #elif (TEACHING_MODE == 1)
        current_status = "VR stanby mode";
        mode0 = "VR stanby mode";
        mode1 = "VR-teaching mode";
    #endif

    mode2 = "Playback ready";
    mode3 = "Playback execution";
    mode4 = "Path generation done";
    modeErr1 = "Wrong iter number(1~9)";

    Hmode0 = "MODE 0";  // Default value
    Hmode1 = "MODE 1";  // Mode change
    Hmode2 = "MODE 2";  // Way point save
    Hmode3 = "MODE 3";  // Playback start
    Hmode4 = "MODE 4";  // Iteration number
    Hmode5 = "MODE 5";  // Trajectory generation
}

HbuttonCmd::~HbuttonCmd()
{
    #if (Handle_OnOff == 1)
        if (Yuart != nullptr) {
            Yuart->YUART_terminate();
            delete Yuart;
            Yuart = nullptr;  // 안전을 위해 포인터 초기화
        }
    #endif
}


void HbuttonCmd::catch_signal(int sig)
{
    //// if(sig == 0){printf("Desired posture is under 4 \n");}
    //// #if(Handle_OnOff == 1)
    //// Yuart->YUART_terminate();
    //// #endif
    //// printf("Program was terminated !! \n");
    //// exit(1);

    if (sig == 0) {
        RCLCPP_WARN(this->get_logger(), "Desired posture is under 4");
    }

    #if (Handle_OnOff == 1)
    if (Yuart) {
        Yuart->YUART_terminate();
    }
    #endif

    RCLCPP_INFO(this->get_logger(), "Program was terminated!!");
    rclcpp::shutdown();  // ROS2 종료
    exit(1);
}

/* ROS_MSG functions */
void HbuttonCmd::VRPose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    VRPose_point.x = msg->pose.position.x;
    VRPose_point.y = msg->pose.position.y;
    VRPose_point.z = msg->pose.position.z;

    // RCLCPP_INFO(this->get_logger(), "Received VR Pose: (%.3f, %.3f, %.3f)", VRPose_point.x, VRPose_point.y, VRPose_point.z);
}

/* Service functions */
bool HbuttonCmd::SRV1_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req; (void)res;  // 사용하지 않음

    #if(TEACHING_MODE == 0)
        Mode_chage();
    #elif(TEACHING_MODE == 1)
        VR_mode_change();
    #endif
    return true;
}

bool HbuttonCmd::SRV3_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req; (void)res;

    #if(TEACHING_MODE == 0)
        Way_point_save();
    #elif(TEACHING_MODE == 1)
        VR_point_save();
    #endif
    return true;
}

bool HbuttonCmd::SRV4_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req; (void)res;

    #if(TEACHING_MODE == 0)
        Trajectory_gen();
    #endif
    return true;
}


/* Iteration number set */
bool HbuttonCmd::SRV11_Handle(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    #if(TEACHING_MODE == 0)
    Iter_num_set();
    #endif
    return true;
}

/* Playback execution */
bool HbuttonCmd::SRV12_Handle(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    #if(TEACHING_MODE == 0)
    Playback_exe();
    #endif
    return true;
}


void HbuttonCmd::Mode_chage()
{
    if (!guiding_mode) // change to guiding mode
    {
        guiding_mode = true;
        current_status = mode1;

        #if(Handle_OnOff == 1)
        yoon_mode_msg.data = Hand_guiding_mode_cmd;
        yoon_mode_pub->publish(yoon_mode_msg);

        //// ros::spinOnce();
        rclcpp::spin_some(this->get_node_base_interface());  // ROS 2에서 spinOnce 대신
        #endif
    }
    else // change to standby mode
    {
        guiding_mode = false;
        current_status = mode0;

        #if(Handle_OnOff == 1)
        // Stanby mode message publish 
        yoon_mode_msg.data = Motion_stop_cmd;
        yoon_mode_pub->publish(yoon_mode_msg);
        //// ros::spinOnce();
        rclcpp::spin_some(this->get_node_base_interface());
        #endif

        if (point_counter != 0)
        {
            // Way point save termination message publish
            yoon_mode_msg.data = Descrete_recording_save;
            yoon_mode_pub->publish(yoon_mode_msg);
            //// ros::spinOnce();
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }
}
void HbuttonCmd::VR_mode_change()
{
    // if (guiding_mode == 1)
    // {
    //     guiding_mode = 0;
    //     RCLCPP_INFO(this->get_logger(), "VR mode OFF");
    // }
    // else
    // {
    //     guiding_mode = 1;
    //     RCLCPP_INFO(this->get_logger(), "VR mode ON");
    // }

    if(guiding_mode == false) // change to guiding mode
    {
        guiding_mode = true;
        current_status = mode1;
    }
    else // change to stanby mode
    {
        guiding_mode = false;
        current_status = mode0;
        point_counter = 0;
    }

}

void HbuttonCmd::Way_point_save()  // to solve error: (2025.06.27 23:35)
{
    // yoon_mode = 3;
    // RCLCPP_INFO(this->get_logger(), "=== Waypoint Saved ===");

    // yoon_path.header.stamp = this->get_clock()->now();
    // yoon_path.header.frame_id = "base_link";
    // yoon_path.poses.push_back(pose_msg);

    yoon_mode_msg.data = Descrete_reording_start;
    yoon_mode_pub->publish(yoon_mode_msg);
    //// ros::spinOnce();
    rclcpp::spin_some(this->get_node_base_interface());
    point_counter ++;

}

void HbuttonCmd::VR_point_save()
{
    Clicked_msg.point.x = VRPose_point.x;
    Clicked_msg.point.y = VRPose_point.y;
    Clicked_msg.point.z = VRPose_point.z;
    Clicked_pub->publish(Clicked_msg);
    //// ros::spinOnce();
    rclcpp::spin_some(this->get_node_base_interface());
    point_counter ++;
}

void HbuttonCmd::Trajectory_gen()
{
    /*** STEP0 : Load the desired values from YAML file ***/

    /* Desired force setting */
    double Desired_Fx = NRS_Fcon_desired["AAC_Des_Force"]["Fx"].as<double>();
    double Desired_Fy = NRS_Fcon_desired["AAC_Des_Force"]["Fy"].as<double>();
    double Desired_Fz = NRS_Fcon_desired["AAC_Des_Force"]["Fz"].as<double>();
    
    /* Desired motion velocity setting */
    double Star2Cont_vel = NRS_Fcon_desired["AAC_Des_velocity"]["Start2Contact"].as<double>();
    double Cont2Term_vel = NRS_Fcon_desired["AAC_Des_velocity"]["Contact2Termin"].as<double>();
    double MMotion_vel = NRS_Fcon_desired["AAC_Des_velocity"]["MMotion_vel"].as<double>();

    /*** STEP1 : Load the waypoints ***/
    // Tot_txt_load txt_loader("/home/gene/catkin_ws/src/rtde_handarm/src/Descre_P_recording.txt");
    auto YamlDescre_P_recording_path = NRS_recording["Descre_P_recording"].as<std::string>();
    char* Descre_P_recording_path = const_cast<char*>(YamlDescre_P_recording_path.c_str());
    Tot_txt_load txt_loader(Descre_P_recording_path);
    Eigen::MatrixXd Decr_RD_points, Decr_RD_points_tem;
    txt_loader.Tot_data_read(Decr_RD_points_tem);

    std::cout << Decr_RD_points_tem << std::endl;

    /*** STEP2 : Path generation ***/
    Yoon_path Descr_RD_blending;
    /* trajectory planning && recording to text */ 

    /* Open the text file */
    // FILE* Hand_G_recording = fopen("/home/gene/catkin_ws/src/rtde_handarm/src/Hand_G_recording.txt","wt");
    auto Hand_G_recording_path = NRS_recording["Hand_G_recording"].as<std::string>();
    FILE* Hand_G_recording = fopen(Hand_G_recording_path.c_str(),"wt");
    
    /**** Points to path profile ****/

    /* 1) Contact points path profile with force */
    int MPoint_num = Decr_RD_points_tem.rows(); // number of points (actal num : MPoint_num)

    /* Desired velocity & force update */
    if(MPoint_num<4) 
    {
        /* # of points is under 3 */
        if(MPoint_num < 3) {catch_signal(0);}
        else
        {
            Decr_RD_points = Eigen::MatrixXd::Zero(4,Decr_RD_points_tem.cols());
            Decr_RD_points.row(0) = Decr_RD_points_tem.row(0);
            Decr_RD_points.row(1) = Decr_RD_points_tem.row(1);
            Decr_RD_points.row(2) = Decr_RD_points_tem.row(1);
            Decr_RD_points.row(3) = Decr_RD_points_tem.row(2);

            MPoint_num = 4;
        }
    }
    else {Decr_RD_points = Decr_RD_points_tem;}

    double MTar_vel[MPoint_num-1] = {0,};
    double MWaiting_time[MPoint_num-1] = {0,};
    double PPB_des_force[MPoint_num-1] = {0,};

    // Desired velocity & force update
    for(int i=0;i<MPoint_num-1;i++) // Point number : MPoint_num
    {
        if(i == 0) 
        {
            MTar_vel[i] = Star2Cont_vel;
            PPB_des_force[i] = (double)0.0;
        }
        else if(i == MPoint_num-2) 
        {
            MTar_vel[i] = Cont2Term_vel;
            PPB_des_force[i] = (double)0.0;
        }
        else 
        {
            MTar_vel[i] = MMotion_vel;
            PPB_des_force[i] = Desired_Fz;
        }
    }

    std::cout << Decr_RD_points << std::endl;
    if(Descr_RD_blending.PPB_path_init(Decr_RD_points.block(0,0,MPoint_num,6),MTar_vel,PPB_des_force,MWaiting_time,MPoint_num))
    {
        double path_out[7] = {0,};
        while(Descr_RD_blending.PPB_path_exe(path_out))
        {
            fprintf(Hand_G_recording,"%10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
            path_out[0], path_out[1], path_out[2], path_out[3], path_out[4], path_out[5],
            Desired_Fx, Desired_Fy, path_out[6]);
        }
    }

    fclose(Hand_G_recording);

    point_counter = 0; // selected way point num init
    current_status = mode4;
}

void HbuttonCmd::Iter_num_set()
{
    if(PB_exe_counter == 0)
    {
        if(iter_num<9) iter_num ++;
        else iter_num = 0;
    }
}

void HbuttonCmd::Playback_exe()
{
    if(iter_num > 0 && iter_num < 10)
    {
        if(PB_exe_counter == 0)
        {
            // Playback iteration number publish
            PbNum_command_msg.data = iter_num;
            PbNum_command_pub->publish(PbNum_command_msg);
            //// PbNum_command_pub.publish(PbNum_command_msg);
            //// ros::spinOnce();
            current_status = mode2;
            PB_exe_counter++;
        }
        else if(PB_exe_counter == 1)
        {
            /*** STEP1 : Playback start ***/
            yoon_mode_msg.data = Playback_mode_cmd;
            yoon_mode_pub->publish(yoon_mode_msg);
            //// yoon_mode_pub.publish(yoon_mode_msg);
            //// ros::spinOnce();
            rclcpp::spin_some(this->get_node_base_interface());  // ROS 2에서 spinOnce 대신
            current_status = mode3;
            iter_num = 0;
            PB_exe_counter = 0;
        }
        else PB_exe_counter = 0;


    }
    else current_status = modeErr1;
}

void HbuttonCmd::HButton_main()
{
    while(1)
    {
        /** Real time monitoring **/
        printf("\n============================================================\n");
        printf("************************************************************\n");
        printf("NOTE : Before start the main node, the screen of handle \n");
        printf("       must be 'STANBY MODE' \n");
        printf("************************************************************\n");
        printf("Current status: %s \n",current_status.c_str());
        printf("Selected way points: %d, Iteration number(1~9): %d \n",point_counter,iter_num);
        printf("Mode value: %d \n",Mode_val);
        printf("\n============================================================\n");

        #if(Handle_OnOff == 1)
        if(Yuart->YUART_start(buffer)) 
        {
            // printf("%s\n",buffer);

            /* Raw mode aquisition */
            if(strcmp(buffer,Hmode0.c_str())==0) Mode_val=0;
            else if(strcmp(buffer,Hmode1.c_str())==0) Mode_val=1;
            else if(strcmp(buffer,Hmode2.c_str())==0) Mode_val=2;
            else if(strcmp(buffer,Hmode3.c_str())==0) Mode_val=3;
            else if(strcmp(buffer,Hmode4.c_str())==0) Mode_val=4;
            else if(strcmp(buffer,Hmode5.c_str())==0) Mode_val=5;


            if((Pre_Mode_val == 0) && (Mode_val == 1)) // Mode change
            {
                #if(TEACHING_MODE == 0)
                Mode_chage();
                #elif(TEACHING_MODE == 1)
                VR_mode_change();
                #endif
            }
            if((Pre_Mode_val == 0) && (Mode_val == 2)) // Way point save
            {
                #if(TEACHING_MODE == 0)
                Way_point_save();
                #elif(TEACHING_MODE == 1)
                VR_point_save();
                #endif
            }
            if((Pre_Mode_val == 0) && (Mode_val == 5)) // Generate the trajectory
            {
                #if(TEACHING_MODE == 0)
                Trajectory_gen();
                #endif
            }
            if((Pre_Mode_val == 0) && (Mode_val == 4)) // Iteration number setting
            {
                Iter_num_set();
            }
            if((Pre_Mode_val == 0) && (Mode_val == 3)) // Playback execution
            {   
                Playback_exe();
            }

            Pre_Mode_val = Mode_val; // mode update to past

        }
        #endif
        // ros::spinOnce(); // For service callback
        rclcpp::spin_some(this->get_node_base_interface());  // ROS 2에서 spinOnce 대신
    }
    exit(0);
    #if(Handle_OnOff == 1)
    Yuart->YUART_terminate();
    #endif
}

// void catch_signal(int sig)
// {
//     printf("Program was terminated \n");
//     exit(1);
// }
