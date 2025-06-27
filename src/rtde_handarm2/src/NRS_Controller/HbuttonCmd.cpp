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


void HbuttonCmd::VRPose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // 아직 미구현
}

void HbuttonCmd::SRV1_Handle(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    // 아직 미구현
}

void HbuttonCmd::SRV3_Handle(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    // 아직 미구현
}

void HbuttonCmd::SRV4_Handle(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    // 아직 미구현
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

