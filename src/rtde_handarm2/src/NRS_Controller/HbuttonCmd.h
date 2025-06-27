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
#define Handle_OnOff 1 // Handle on : 1, Handle off : 0

class HbuttonCmd : public rclcpp::Node
{
public:
    HbuttonCmd();

private:
    //////// ROS2 Node 및 동기화 자료 ////////


    //////// Subscribers ////////


    //////// Publishers ////////


    //////// Timer ////////
    // rclcpp::TimerBase::SharedPtr timer_;

    //////// 콜백 함수 ////////


    //////// 상태 변수 및 제어 파라미터 ////////

};

#endif // HBUTTONCMD_H
