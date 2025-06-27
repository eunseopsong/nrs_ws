#ifndef HBUTTONCMD_H
#define HBUTTONCMD_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "Yoon_UR10e_cmd.h"
#include <rclcpp/rclcpp.hpp> //// #include "ros/ros.h"

// For ROS message
#include "std_msgs/msg/multi_array_layout.hpp"    //// #include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/msg/multi_array_dimension.hpp" //// #include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/msg/float64_multi_array.hpp"   //// #include "std_msgs/Float64MultiArray.h"
#include "std_msgs/msg/float64.hpp"               //// #include "std_msgs/Float64.h"
#include "std_msgs/msg/u_int16.hpp"               //// #include "std_msgs/UInt16.h"
#include "std_msgs/msg/u_int32.hpp"               //// #include "std_msgs/UInt32.h"

#include "geometry_msgs/msg/point_stamped.hpp"   //// #include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/msg/pose_stamped.hpp"    //// #include "geometry_msgs/PoseStamped.h"
// #include "rtde_handarm2/VrPosRtMsgQua.msg"         //// #include "rtde_handarm/VRposRtMsg_Qua.h" // 보류

// For ROS service - AIDIN GUI SERVER
#include "std_srvs/srv/empty.hpp"                //// #include <std_srvs/Empty.h>

// For trajectory generation
#include "Text_loader.h"
#include "Yoon_path.h"

// For UART communication
#include "Yoon_communi.h"

// Yaml file headers
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "NRS_yaml_location.h"

#define DOF 6
#define PI 3.141592
#define TEACHING_MODE 0 // Direct Teaching: 0, VR-Vision Teaching: 1 (Danang Demo)
#define Handle_OnOff 1  // Handle on : 1, Handle off : 0

////class NRS_Hbutton_cmd : public std::enable_shared_from_this<NRS_Hbutton_cmd>
class HbuttonCmd : public rclcpp::Node
{
public:
    HbuttonCmd(); //// HbuttonCmd(const rclcpp::Node::SharedPtr& node, int loop_rate_);
    ~HbuttonCmd();

    /*** Functions definition ***/

    /* ROS_MSG Callback functions */
    void VRPose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /* Service handles */
    bool SRV1_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                     std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool SRV3_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                     std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool SRV4_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                     std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool SRV11_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                      std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool SRV12_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                      std::shared_ptr<std_srvs::srv::Empty::Response> res);

    /* Main function */
    void HButton_main();

    /* Mode functions */
    void Mode_chage();
    void VR_mode_change();
    void Way_point_save();
    void VR_point_save();
    void Trajectory_gen();
    void Iter_num_set();
    void Playback_exe();
    void catch_signal(int sig);

private:
    //////// ROS2 Node ////////
    rclcpp::Node::SharedPtr node_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Rate loop_rate;

    /*** Parameters setting ***/
    /* UART instance */
    Yoon_UART* Yuart;

    /* Yaml-file instance */
    std::ifstream fin1;
    std::ifstream fin2;

    YAML::Node NRS_recording;
    YAML::Node NRS_Fcon_desired;

    /* ROS Message instance */
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr              yoon_mode_pub;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr              PbNum_command_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr   Clicked_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr VRPose_sub;

    geometry_msgs::msg::PointStamped Clicked_msg;
    std_msgs::msg::UInt16            yoon_mode_msg;
    std_msgs::msg::UInt32            PbNum_command_msg;

    /* ROS Service instance */
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv1;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv3;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv4;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv11;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv12;

    /* Normal parameters */
    geometry_msgs::msg::Point VRPose_point;
    char buffer[1024] = {0};
    bool pre_button_val = false;
    bool button_val = false;

    int Mode_val = 0;
    int Pre_Mode_val = 0;

    bool guiding_mode = false; // false: standby mode, true: guiding mode
    int point_counter = 0;     // Saved way points
    int iter_num = 0;          // Iteration number
    int PB_exe_counter = 0;    // 1: ready status, 2: execution

    /* State & mode */
    std::string current_status;
    std::string mode0, mode1, mode2, mode3, mode4;
    std::string modeErr1;

    std::string Hmode0, Hmode1, Hmode2, Hmode3, Hmode4, Hmode5;
};

#endif // HBUTTONCMD_H
