#ifndef NRS_CALLBACK_H
#define NRS_CALLBACK_H

#include "nrs_io.h"
#include "nrs_geodesic.h"
#include "nrs_interpolation.h"
#include "nrs_visualization.h"

#include <rclcpp/rclcpp.hpp>                                // #include <ros/ros.h>
#include "std_srvs/srv/empty.hpp"                           // #include <std_srvs/Empty.h>
#include <sensor_msgs/msg/point_cloud2.hpp>                 // #include <sensor_msgs/PointCloud2.h>
#include "ament_index_cpp/get_package_share_directory.hpp"  // #include <ros/package.h>

class nrs_callback
{
public:
    nrs_callback();
    nrs_io n_io;
    nrs_geodesic n_geodesic;
    nrs_interpolation n_interpolation;
    nrs_visualization n_visualization;

    /*-------------------------------path generation-------------------------------*/
    std::string mesh_file_path; // path_planning할 때 사용되는 mesh 파일

    ros::Publisher geodesic_waypoints_pub;        // geodesic_Waypoints publish할 때 사용되는 publisher
    nrs_path2::Waypoints waypoints_msg;          // geodesic_waypoints를 publish할 때 사용되는 msg
    std::vector<Eigen::Vector3d> selected_points; // clicked_point를 전처리해서 path_generation할 때 사용
    std::string geodesic_waypoints_file_path;     // geodesic path를 저장하기 위한 file path
    /*-------------------------------path interpolation-------------------------------*/

    ros::Publisher interpolated_waypoints_pub; // interpolated_waypoints publish할 때 사용되는 publisher
    nrs_path2::Waypoints geodesic_path;       // geodesic waypoints를 interpolation하기 위해 사용

    double desired_interval, Fx, Fy, Fz;
    std::string interpolated_waypoints_file_path;
  /*-------------------------------path simulation-------------------------------*/
  
    bool splinePathServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool straightPathServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool PathInterpolationCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool pathDeleteCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
   
};

#endif // NRS_CALLBACK_H
