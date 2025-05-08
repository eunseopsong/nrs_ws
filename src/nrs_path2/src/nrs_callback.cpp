#include "nrs_callback.h"

nrs_callback::nrs_callback(rclcpp::Node::SharedPtr node)
: node_(node) // node 멤버에 전달된 node 포인터 저장
{
    spline_service_ = node_->create_service<std_srvs::srv::Empty>(
        "spline",  // 서비스 이름
        std::bind(&nrs_callback::splinePathServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

bool nrs_callback::splinePathServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    ROS_INFO("Spline path service called. Generating Hermite Spline Path...");
    std::ifstream input(mesh_file_path, std::ios::binary);
    Triangle_mesh tmesh;
    n_geodesic.load_stl_file(input, tmesh);
    Tree *tree;
    Surface_mesh_shortest_path *shortest_paths;
    tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    shortest_paths = new Surface_mesh_shortest_path(tmesh);
    // 웨이포인트 메시지 초기화
    waypoints_msg.waypoints.clear();

    // 최소 3개 이상의 선택된 점이 필요
    if (selected_points.size() > 2)
    {
        // Hermite Spline 경로 생성 (selected_points와 tmesh 사용)
        nrs_path::Waypoints path_points = n_geodesic.GenerateHermiteSplinePath(selected_points, tmesh);

        // 생성된 웨이포인트 퍼블리시
        geodesic_waypoints_pub.publish(path_points);
        ROS_INFO("Published Hermite Spline Path with %zu waypoints", path_points.waypoints.size());

        n_io.clearFile(geodesic_waypoints_file_path);
        n_io.saveWaypointsToFile(path_points, geodesic_waypoints_file_path);
        return true;
    }
    else
    {
        ROS_WARN("Not enough selected points for Hermite Spline Path generation.");
        return false;
    }
}

// bool nrs_callback::straightPathServiceCallback(
//     const std::shared_ptr<std_srvs::srv::Empty::Request> req,
//     std::shared_ptr<std_srvs::srv::Empty::Response> res)
// {
//     RCLCPP_INFO(node_->get_logger(), "Straight path service called. Generating Geodesic Path...");

//     std::ifstream input(mesh_file_path, std::ios::binary);
//     Triangle_mesh tmesh;
//     n_geodesic.load_stl_file(input, tmesh);

//     Tree *tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
//     tree->accelerate_distance_queries();
//     Surface_mesh_shortest_path *shortest_paths = new Surface_mesh_shortest_path(tmesh);

//     waypoints_msg.waypoints.clear();

//     if (selected_points.size() > 1)
//     {
//         nrs_path2::msg::Waypoints path_points = n_geodesic.GenerateStraightGeodesicPath(selected_points, tmesh);
//         geodesic_waypoints_pub->publish(path_points);

//         RCLCPP_INFO(node_->get_logger(), "Published Geodesic Path with %zu waypoints", path_points.waypoints.size());

//         n_io.clearFile(geodesic_waypoints_file_path);
//         n_io.saveWaypointsToFile(path_points, geodesic_waypoints_file_path);
//         return true;
//     }
//     else
//     {
//         RCLCPP_WARN(node_->get_logger(), "Not enough selected points for Geodesic Path generation.");
//         return false;
//     }
// }

// bool nrs_callback::PathInterpolationCallback(
//     const std::shared_ptr<std_srvs::srv::Empty::Request> req,
//     std::shared_ptr<std_srvs::srv::Empty::Response> res)
// {
//     std::ifstream input(mesh_file_path, std::ios::binary);
//     if (!input.is_open())
//     {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to open mesh file: %s", mesh_file_path.c_str());
//         return false;
//     }

//     Triangle_mesh tmesh;
//     if (!n_geodesic.load_stl_file(input, tmesh))
//     {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to load mesh from file: %s", mesh_file_path.c_str());
//         return false;
//     }
//     input.close();

//     Tree *tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
//     tree->accelerate_distance_queries();
//     Surface_mesh_shortest_path *shortest_paths = new Surface_mesh_shortest_path(tmesh);

//     auto start_time_point = std::chrono::high_resolution_clock::now();

//     nrs_path2::msg::Waypoints final_waypoints = n_interpolation.interpolateEnd2End(geodesic_path, desired_interval, tmesh, Fx, Fy, Fz);

//     if (final_waypoints.waypoints.empty())
//     {
//         RCLCPP_ERROR(node_->get_logger(), "Interpolation produced no waypoints.");
//         return false;
//     }

//     interpolated_waypoints_pub->publish(final_waypoints);
//     n_io.clearFile(interpolated_waypoints_file_path);
//     n_io.saveWaypointsToFile(final_waypoints, interpolated_waypoints_file_path);

//     auto end_time_point = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_point - start_time_point).count();
//     RCLCPP_INFO(node_->get_logger(), "Interpolation & Normal smoothing time: %ld s", duration);

//     return true;
// }

// bool nrs_callback::pathDeleteCallback(
//     const std::shared_ptr<std_srvs::srv::Empty::Request> req,
//     std::shared_ptr<std_srvs::srv::Empty::Response> res)
// {
//     n_visualization.deleteMarkers();
//     selected_points.clear();
//     geodesic_path.waypoints.clear();

//     nrs_path2::msg::Waypoints empty_waypoints;
//     interpolated_waypoints_pub->publish(empty_waypoints);

//     RCLCPP_INFO(node_->get_logger(), "Path deleted via pathDeleteCallback");

//     return true;
// }