#ifndef NRS_CALLBACK_H
#define NRS_CALLBACK_H

#include "nrs_io.h"
#include "nrs_geodesic.h"
#include "nrs_interpolation.h"
#include "nrs_visualization.h"

#include <rclcpp/rclcpp.hpp>                                //// #include <ros/ros.h>
#include "std_srvs/srv/empty.hpp"                           //// #include "std_srvs/Empty.h"
#include <sensor_msgs/msg/point_cloud2.hpp>                 //// #include <sensor_msgs/PointCloud2.h>
#include "ament_index_cpp/get_package_share_directory.hpp"  //// #include <ros/package.h>

#include "nrs_path2/msg/waypoint.hpp"
#include "nrs_path2/msg/waypoints.hpp"

class nrs_callback
{
public:
    nrs_callback();    // basic 생성자
    //////////// service node Initialization (for ROS2) ////////////
    explicit nrs_callback(rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;  // 노드 저장용

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr spline_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr straight_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr interpolation_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr path_delete_service;
    ////////////////////////////////////////////////////////////////

    nrs_io n_io;
    nrs_geodesic n_geodesic;
    nrs_interpolation n_interpolation;
    nrs_visualization n_visualization;

    /*-------------------------------path generation-------------------------------*/
    std::string mesh_file_path;

    // geodesic_Waypoints publish할 때 사용되는 publisher
    rclcpp::Publisher<nrs_path2::msg::Waypoints>::SharedPtr geodesic_waypoints_pub;   //// ros::Publisher geodesic_waypoints_pub;

    // geodesic_waypoints를 publish할 때 사용되는 msg
    nrs_path2::msg::Waypoints waypoints_msg;                                          //// nrs_path::Waypoints waypoints_msg;

    // clicked_point를 전처리해서 path_generation할 때 사용
    std::vector<Eigen::Vector3d> selected_points;

    // geodesic path를 저장하기 위한 file path
    std::string geodesic_waypoints_file_path;

    /*-------------------------------path interpolation-------------------------------*/
    // interpolated_waypoints publish할 때 사용되는 publisher
    rclcpp::Publisher<nrs_path2::msg::Waypoints>::SharedPtr interpolated_waypoints_pub; //// ros::Publisher interpolated_waypoints_pub;

    // geodesic waypoints를 interpolation하기 위해 사용
    nrs_path2::msg::Waypoints geodesic_path;                                            //// nrs_path::Waypoints geodesic_path;

    double desired_interval, fx, fy, fz;
    std::string interpolated_waypoints_file_path;

    /*-------------------------------path simulation-------------------------------*/
    //// bool splinePathServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool splinePathServiceCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);

    //// bool straightPathServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool straightPathServiceCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);

    //// bool PathInterpolationCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool PathInterpolationCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);

    //// bool pathDeleteCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool pathDeleteCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);
};

#endif // NRS_CALLBACK_H
