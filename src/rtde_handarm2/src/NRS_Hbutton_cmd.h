#ifndef NRS_HBUTTON_CMD_H
#define NRS_HBUTTON_CMD_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

// extern 변수들 (globals.cpp에서 정의됨)
extern YAML::Node NRS_recording;
extern YAML::Node NRS_Fcon_setting;
extern std::ifstream fin1;
extern std::ifstream fin2;

class NRS_Hbutton_cmd
{
public:
    NRS_Hbutton_cmd(const rclcpp::Node::SharedPtr &node, int loop_rate_val);

    void VRPose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void VR_point_save();

    // ROS 서비스 핸들러
    bool SRV1_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                     std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool SRV3_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                     std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool SRV4_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                     std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool SRV11_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool SRV12_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);

private:
    rclcpp::Node::SharedPtr node_;
    int loop_rate_val_;

    // ROS 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr yoon_mode_pub;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr PbNum_command_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr Clicked_pub;

    // ROS 서브스크라이버
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr VRPose_sub;

    // ROS 서비스 서버
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv1;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv3;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv4;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv11;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv12;

    // 메시지 변수
    geometry_msgs::msg::PointStamped Clicked_msg;

    // 위치 저장 관련
    double VRPose_point_x, VRPose_point_y, VRPose_point_z;
    int point_counter = 0;
};

#endif // NRS_HBUTTON_CMD_H
