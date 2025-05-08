#include "nrs_visualization.h"

#include <rclcpp/rclcpp.hpp>                    //// #include <ros/ros.h>
#include "geometry_msgs/msg/point_stamped.hpp"  //// #include <geometry_msgs/PointStamped.h>
#include "sensor_msgs/msg/point_cloud2.hpp"     //// #include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <iostream>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("nrs_node_visualization");
    //// ros::init(argc, argv, "nrs_node_visualization");
    //// ros::NodeHandle nh;

    auto visualizer = std::make_shared<nrs_visualization>();
    visualizer->init(node);
    //// nrs_visualization visualizer;
    //// visualizer.init(nh);

    auto subscription = node->create_subscription<nrs_path2::msg::Waypoints>(
        "interpolated_waypoints", 10,
        std::bind(&nrs_visualization::waypointsCallback, visualizer.get(), std::placeholders::_1));
    //// ros::Subscriber vis_waypoints_sub = nh.subscribe("interpolated_waypoints", 10, &nrs_visualization::waypointsCallback, &visualizer);

    RCLCPP_INFO(node->get_logger(), "nrs_node_visualization started. Visualization functionalities are active.");
    //// ROS_INFO("nrs_node_visualiztion started.visualization functionalities are active.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    ////ros::spin();

    return 0;
}
