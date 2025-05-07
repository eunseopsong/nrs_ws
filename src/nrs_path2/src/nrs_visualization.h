#ifndef NRS_VISUALIZATION_H
#define NRS_VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>

#include "nrs_path2/msg/waypoint.hpp"
#include "nrs_path2/msg/waypoints.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <array>
#include <string>

class nrs_visualization
{
public:
    nrs_visualization();
    ~nrs_visualization() = default;

    // Node를 외부에서 설정
    rclcpp::Node::SharedPtr node_;

    // Callback 함수들
    void waypointsCallback(const nrs_path2::msg::Waypoints::SharedPtr msg);
    void visualizeClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void deleteMarkers();

    // 시각화 함수
    void visualizePath(const std::vector<geometry_msgs::msg::Point> &path,
                       const std::string &ns, int id,
                       float r, float g, float b, float a);

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub1;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub2;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr clicked_marker_pub;

    std::vector<geometry_msgs::msg::Point> path;
    int clicked_marker_id = 0;
};

#endif // NRS_VISUALIZATION_H
