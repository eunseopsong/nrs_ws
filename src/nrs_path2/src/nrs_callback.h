#ifndef NRS_CALLBACK_H
#define NRS_CALLBACK_H

#include "nrs_io.h"
#include "nrs_geodesic.h"
#include "nrs_interpolation.h"
#include "nrs_visualization.h"

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/empty.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

class nrs_callback
{
public:
    explicit nrs_callback(rclcpp::Node::SharedPtr node);  // ROS 2 노드 주입

    // ROS 2 Node 로거 사용을 위한 포인터
    rclcpp::Node::SharedPtr node_;

    // 주요 객체
    nrs_io n_io;
    nrs_geodesic n_geodesic;
    nrs_interpolation n_interpolation;
    nrs_visualization n_visualization;

    /*-------------------------------path generation-------------------------------*/
    std::string mesh_file_path;

    rclcpp::Publisher<nrs_path::msg::Waypoints>::SharedPtr geodesic_waypoints_pub;
    nrs_path::Waypoints waypoints_msg;
    std::vector<Eigen::Vector3d> selected_points;
    std::string geodesic_waypoints_file_path;

    /*-------------------------------path interpolation-------------------------------*/
    rclcpp::Publisher<nrs_path::msg::Waypoints>::SharedPtr interpolated_waypoints_pub;
    nrs_path::Waypoints geodesic_path;
    double desired_interval, Fx, Fy, Fz;
    std::string interpolated_waypoints_file_path;

    /*-------------------------------path simulation-------------------------------*/
    bool splinePathServiceCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>);

    bool straightPathServiceCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>);

    bool PathInterpolationCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>);

    bool pathDeleteCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>);
};

#endif // NRS_CALLBACK_H
