#include "HbuttonCmd.h"

//// HbuttonCmd::HbuttonCmd(const rclcpp::Node::SharedPtr& node, int loop_rate)
//// : Node("hbutton_node"), node_(node), loop_rate(loop_rate)
HbuttonCmd::HbuttonCmd()
: Node("hbutton_node"), loop_rate(100.0), fin1(NRS_Record_Printing_loc), fin2(NRS_Fcon_desired_loc)
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

