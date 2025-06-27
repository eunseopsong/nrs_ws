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
void HbuttonCmd::SRV1_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req; (void)res;  // 사용하지 않음

    #if(TEACHING_MODE == 0)
        Mode_chage();
    #elif(TEACHING_MODE == 1)
        VR_mode_change();
    #endif
}

void HbuttonCmd::SRV3_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    // (void)req; (void)res;

    // #if(TEACHING_MODE == 0)
    //     Way_point_save();
    // #elif(TEACHING_MODE == 1)
    //     VR_point_save();
    // #endif
}

void HbuttonCmd::SRV4_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    // (void)req; (void)res;

    // #if(TEACHING_MODE == 0)
    //     Trajectory_gen();
    // #endif
}




void HbuttonCmd::SRV11_Handle(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    // 아직 미구현
}

void HbuttonCmd::SRV12_Handle(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    // 아직 미구현
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
        rclcpp::spin_some(this->get_node_base_interface());  // ROS 2에서 spinOnce 대신
        #endif
    }
    else // change to standby mode
    {
        guiding_mode = false;
        current_status = mode0;

        #if(Handle_OnOff == 1)
        yoon_mode_msg.data = Motion_stop_cmd;
        yoon_mode_pub->publish(yoon_mode_msg);
        rclcpp::spin_some(this->get_node_base_interface());
        #endif

        if (point_counter != 0)
        {
            yoon_mode_msg.data = Descrete_recording_save;
            yoon_mode_pub->publish(yoon_mode_msg);
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }
}
void HbuttonCmd::VR_mode_change()  // to solve error: there is no definition of guide_mode (2025.06.27 23:32)
{
    // if (guide_mode == 1)
    // {
    //     guide_mode = 0;
    //     RCLCPP_INFO(this->get_logger(), "VR mode OFF");
    // }
    // else
    // {
    //     guide_mode = 1;
    //     RCLCPP_INFO(this->get_logger(), "VR mode ON");
    // }
}
void HbuttonCmd::Way_point_save()
{
    // yoon_mode = 3;
    // RCLCPP_INFO(this->get_logger(), "=== Waypoint Saved ===");

    // yoon_path.header.stamp = this->get_clock()->now();
    // yoon_path.header.frame_id = "base_link";
    // yoon_path.poses.push_back(pose_msg);



    //// yoon_mode_msg.data = Descrete_reording_start;
    //// yoon_mode_pub.publish(yoon_mode_msg);
    //// ros::spinOnce();
    //// point_counter ++;

}










