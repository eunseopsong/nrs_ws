// #ifndef NRS_HBUTTON_CMD_H
// #define NRS_HBUTTON_CMD_H

// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/u_int16.hpp>
// #include <std_msgs/msg/u_int32.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <std_srvs/srv/empty.hpp>
// #include <yaml-cpp/yaml.h>
// #include <fstream>
// #include <string>

// // Forward declaration of your UART class
// class Yoon_UART;

// class NRS_Hbutton_cmd
// {
// public:
//     NRS_Hbutton_cmd(const rclcpp::Node::SharedPtr &node, int loop_rate);
//     ~NRS_Hbutton_cmd();

//     rclcpp::Node::SharedPtr node_;
//     // rclcpp::Duration loop_period_ = rclcpp::Duration::from_seconds(0.01);  // 예: 10ms
//     rclcpp::Duration loop_period_;
//     rclcpp::Time last_time_;

//     void VRPose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
//     bool SRV1_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
//     bool SRV3_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
//     bool SRV4_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
//     bool SRV11_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//                       std::shared_ptr<std_srvs::srv::Empty::Response> response);
//     bool SRV12_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//                       std::shared_ptr<std_srvs::srv::Empty::Response> response);

//     void HButton_main();
//     void Mode_chage();
//     void VR_mode_change();
//     void Way_point_save();
//     void VR_point_save();
//     void Trajectory_gen();
//     void Iter_num_set();
//     void Playback_exe();
//     void catch_signal(int sig);

//     rclcpp::Rate loop_rate{10};
//     Yoon_UART* Yuart;
//     std::ifstream fin1, fin2;
//     YAML::Node NRS_recording, NRS_Fcon_desired;

//     rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr yoon_mode_pub;
//     rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr PbNum_command_pub;
//     rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr Clicked_pub;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr VRPose_sub;

//     geometry_msgs::msg::PointStamped Clicked_msg;
//     std_msgs::msg::UInt16 yoon_mode_msg;
//     std_msgs::msg::UInt32 PbNum_command_msg;

//     rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv1;
//     rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv3;
//     rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv4;
//     rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv11;
//     rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv12;

//     geometry_msgs::msg::Point VRPose_point;
//     char buffer[1024] = {0};
//     bool pre_button_val = false;
//     bool button_val = false;
//     int Mode_val = 0;
//     int Pre_Mode_val = 0;
//     bool guiding_mode = false;
//     int point_counter = 0;
//     int iter_num = 0;
//     int PB_exe_counter = 0;

//     std::string current_status;
//     std::string mode0, mode1, mode2, mode3, mode4;
//     std::string modeErr1;
//     std::string Hmode0, Hmode1, Hmode2, Hmode3, Hmode4, Hmode5;
// };

// #endif // NRS_HBUTTON_CMD_H
