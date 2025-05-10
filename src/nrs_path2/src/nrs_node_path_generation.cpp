#include "nrs_callback.h"
#include "nrs_visualization.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <fstream>
#include <iostream>

// CGAL 및 nrs_mesh 관련 헤더는 nrs_callback.h 에서 이미 include 되었다고 가정

// === 전역 변수 ===
nrs_callback callback_handler;
Triangle_mesh tmesh;
std::string mesh_file_path = "/home/eunseop/nrs_ws/src/nrs_path2/mesh/workpiece.stl";

// [생성] : "/clicked_point" 토픽 콜백 함수 (경로 생성)
void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("clicked_point"), "Clicked point received");

    Kernel::Point_3 clicked_point(msg->point.x, msg->point.y, msg->point.z);

    face_descriptor face;
    Surface_mesh_shortest_path::Barycentric_coordinates location;

    if (!callback_handler.n_geodesic.locate_face_and_point(clicked_point, face, location, tmesh))
    {
        RCLCPP_ERROR(rclcpp::get_logger("clicked_point"), "Failed to locate clicked point on mesh.");
        return;
    }

    Kernel::Point_3 v1, v2, v3;
    int i = 0;
    for (auto v : vertices_around_face(tmesh.halfedge(face), tmesh))
    {
        if (i == 0) v1 = tmesh.point(v);
        else if (i == 1) v2 = tmesh.point(v);
        else if (i == 2) v3 = tmesh.point(v);
        ++i;
    }

    Kernel::Point_3 projected_point = CGAL::barycenter(v1, location[0], v2, location[1], v3, location[2]);
    Eigen::Vector3d projected_point_eigen(projected_point.x(), projected_point.y(), projected_point.z());
    callback_handler.selected_points.push_back(projected_point_eigen);
}

// === 지오데식 경로 콜백 ===
void geodesicPathCallback(const nrs_path2::msg::Waypoints::SharedPtr msg)
{
    for (const auto &wp : msg->waypoints)
    {
        callback_handler.geodesic_path.waypoints.push_back(wp);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("nrs_combined_node");

    // === 서비스 등록을 위한 생성자 호출 === (2025.05.10 22:12)
    callback_handler = nrs_callback(node);

    // // Spline Path Callback
    // spline_service = node_->create_service<std_srvs::srv::Empty>(
    //     "spline",
    //     std::bind(&nrs_callback::splinePathServiceCallback, this,
    //               std::placeholders::_1, std::placeholders::_2));

    // // Geodesic (직선) 경로 생성 서비스 콜백 함수
    // straight_service = node_->create_service<std_srvs::srv::Empty>(
    //     "straight",
    //     std::bind(&nrs_callback::straightPathServiceCallback, this,
    //             std::placeholders::_1, std::placeholders::_2));

    // // Waypoints 콜백 함수 구현
    // interpolation_service = node_->create_service<std_srvs::srv::Empty>(
    //     "interpolation",
    //     std::bind(&nrs_callback::PathInterpolationCallback, this,
    //             std::placeholders::_1, std::placeholders::_2));

    // // Path Delete Callback
    // path_delete_service = node_->create_service<std_srvs::srv::Empty>(
    //     "delete",
    //     std::bind(&nrs_callback::pathDeleteCallback, this,
    //             std::placeholders::_1, std::placeholders::_2));


    // === Path Generation ===
    callback_handler.node_ = node;
    callback_handler.mesh_file_path = mesh_file_path;
    callback_handler.geodesic_waypoints_file_path = "/home/eunseop/nrs_ws/src/nrs_path2/data/geodesic_waypoints.txt";
    callback_handler.geodesic_waypoints_pub =
        node->create_publisher<nrs_path2::msg::Waypoints>("geodesic_path", 10);

    // === Path Interpolation ===
    callback_handler.interpolated_waypoints_file_path = "/home/eunseop/nrs_ws/src/nrs_path2/data/final_waypoints.txt";
    callback_handler.desired_interval = 0.00004;
    callback_handler.fx = 0.0;
    callback_handler.fy = 0.0;
    callback_handler.fz = 10.0;
    callback_handler.interpolated_waypoints_pub =
        node->create_publisher<nrs_path2::msg::Waypoints>("interpolated_waypoints", 10);

    // === Load Mesh ===
    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open mesh file: %s", mesh_file_path.c_str());
        return -1;
    }
    callback_handler.n_geodesic.load_stl_file(input, tmesh);

    // === Register Services ===
    node->create_service<std_srvs::srv::Empty>(
        "spline",
        std::bind(&nrs_callback::splinePathServiceCallback, &callback_handler,
                  std::placeholders::_1, std::placeholders::_2));

    node->create_service<std_srvs::srv::Empty>(
        "straight",
        std::bind(&nrs_callback::straightPathServiceCallback, &callback_handler,
                  std::placeholders::_1, std::placeholders::_2));

    node->create_service<std_srvs::srv::Empty>(
        "interpolate",
        std::bind(&nrs_callback::PathInterpolationCallback, &callback_handler,
                  std::placeholders::_1, std::placeholders::_2));

    node->create_service<std_srvs::srv::Empty>(
        "delete",
        std::bind(&nrs_callback::pathDeleteCallback, &callback_handler,
                  std::placeholders::_1, std::placeholders::_2));

    // === Subscriptions ===
    auto sub_clicked = node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, clickedPointCallback);

    auto sub_geodesic_path = node->create_subscription<nrs_path2::msg::Waypoints>(
        "/geodesic_path", 10, geodesicPathCallback);

    // === Visualization Subscriptions ===
    nrs_visualization visualizer;
    visualizer.node_ = node;

    auto vis_waypoints_sub = node->create_subscription<nrs_path2::msg::Waypoints>(
        "interpolated_waypoints", 10,
        std::bind(static_cast<void (nrs_visualization::*)(nrs_path2::msg::Waypoints::SharedPtr)>
            (&nrs_visualization::waypointsCallback), &visualizer, std::placeholders::_1));

    auto vis_clicked_point_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        std::bind(static_cast<void (nrs_visualization::*)(geometry_msgs::msg::PointStamped::SharedPtr)>
            (&nrs_visualization::visualizeClickedPoint), &visualizer, std::placeholders::_1));

    // === Info Log ===
    RCLCPP_INFO(node->get_logger(), "Combined nrs node started. Generation, interpolation and visualization functionalities are active.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
