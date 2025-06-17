#ifndef NRS_HBUTTON_CMD_HPP
#define NRS_HBUTTON_CMD_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <memory>

class NRS_Hbutton_cmd
{
public:
    NRS_Hbutton_cmd(const rclcpp::Node::SharedPtr &node, int loop_rate_val);

    void HButton_main();
    void VRPose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void VR_point_save();

    bool SRV1_Handle(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool SRV3_Handle(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool SRV4_Handle(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool SRV11_Handle(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool SRV12_Handle(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);

private:
    rclcpp::Node::SharedPtr node_;
    int loop_rate_val_;

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr yoon_mode_pub;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr PbNum_command_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr Clicked_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr VRPose_sub;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv1;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv3;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv4;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv11;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv12;

    geometry_msgs::msg::Point VRPose_point;
    geometry_msgs::msg::PointStamped Clicked_msg;
    std_msgs::msg::UInt16 yoon_mode_msg;
    std_msgs::msg::UInt32 PbNum_command_msg;

    // YAML
    std::string fin1, fin2;
    YAML::Node NRS_recording, NRS_Fcon_desired;

    // UART
    Yoon_UART* Yuart;


  int point_counter;
  // 기타 변수 및 경로 관련 멤버들 필요 시 여기에 선언
};

#endif // NRS_HBUTTON_CMD_HPP
