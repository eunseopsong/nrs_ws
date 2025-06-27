#include "HbuttonCmd.h"

HbuttonCmd::HbuttonCmd()
: Node("hbutton_node"), loop_rate_(100.0)  // 기본 루프 주기 설정 (예: 100Hz)
{
    ///////////////// 1. ROS Publisher 초기화 //////////////////
    yoon_mode_pub = this->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 10);
    PbNum_command_pub = this->create_publisher<std_msgs::msg::UInt32>("Yoon_PbNum_cmd", 10);
    Clicked_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("clicked_point", 10);

    ///////////////// 2. ROS Subscriber 초기화 //////////////////
    VRPose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vive/pos0", 10,
        std::bind(&HbuttonCmd::VRPose_Callback, this, std::placeholders::_1));

    ///////////////// 3. ROS Service 서버 초기화 //////////////////
    Aidin_gui_srv1 = this->create_service<std_srvs::srv::Empty>(
        "Aidin_gui_srv1",
        std::bind(&HbuttonCmd::SRV1_Handle, this, std::placeholders::_1, std::placeholders::_2));

    Aidin_gui_srv3 = this->create_service<std_srvs::srv::Empty>(
        "Aidin_gui_srv3",
        std::bind(&HbuttonCmd::SRV3_Handle, this, std::placeholders::_1, std::placeholders::_2));

    Aidin_gui_srv4 = this->create_service<std_srvs::srv::Empty>(
        "Aidin_gui_srv4",
        std::bind(&HbuttonCmd::SRV4_Handle, this, std::placeholders::_1, std::placeholders::_2));

    Aidin_gui_srv11 = this->create_service<std_srvs::srv::Empty>(
        "Aidin_gui_srv11",
        std::bind(&HbuttonCmd::SRV11_Handle, this, std::placeholders::_1, std::placeholders::_2));

    Aidin_gui_srv12 = this->create_service<std_srvs::srv::Empty>(
        "Aidin_gui_srv12",
        std::bind(&HbuttonCmd::SRV12_Handle, this, std::placeholders::_1, std::placeholders::_2));
}
