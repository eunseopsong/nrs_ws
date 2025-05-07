#include "nrs_visualization.h"

nrs_visualization::nrs_visualization() : clicked_marker_id(0)
{
    // 퍼블리셔는 외부에서 node_ 설정 후 직접 초기화
}

// 경로 시각화
void nrs_visualization::visualizePath(const std::vector<geometry_msgs::msg::Point> &path,
                                      const std::string &ns, int id,
                                      float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "base_link";
    path_marker.header.stamp = node_->get_clock()->now();
    path_marker.ns = ns;
    path_marker.id = id;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.003;
    path_marker.color.r = r;
    path_marker.color.g = g;
    path_marker.color.b = b;
    path_marker.color.a = a;

    for (const auto &pt : path)
    {
        path_marker.points.push_back(pt);
    }

    if (marker_pub1)
        marker_pub1->publish(path_marker);
}

// 마커 삭제
void nrs_visualization::deleteMarkers()
{
    path.clear();
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "base_link";
    delete_marker.header.stamp = node_->get_clock()->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

    delete_marker.ns = "geodesic_path";
    delete_marker.id = 0;
    if (marker_pub1)
        marker_pub1->publish(delete_marker);

    delete_marker.ns = "clicked_points";
    delete_marker.id = 0;
    if (marker_pub2)
        marker_pub2->publish(delete_marker);

    RCLCPP_INFO(node_->get_logger(), "All markers and internal path cleared");
}

// Waypoints 콜백
void nrs_visualization::waypointsCallback(const nrs_path2::msg::Waypoints::SharedPtr msg)
{
    path.clear();
    for (const auto &wp : msg->waypoints)
    {
        geometry_msgs::msg::Point pt;
        pt.x = wp.x;
        pt.y = wp.y;
        pt.z = wp.z;
        path.push_back(pt);
    }
    visualizePath(path, "geodesic_path", 0, 0.0f, 1.0f, 0.0f, 1.0f);
}

// 클릭된 점 시각화
void nrs_visualization::visualizeClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "clicked_points";
    marker.id = clicked_marker_id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = msg->point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    if (marker_pub2)
        marker_pub2->publish(marker);
}
