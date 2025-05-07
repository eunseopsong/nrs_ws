#include "nrs_callback.h"

nrs_callback::nrs_callback(rclcpp::Node::SharedPtr node)
: node_(node)
{
}

bool nrs_callback::splinePathServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    RCLCPP_INFO(node_->get_logger(), "Spline path service called. Generating Hermite Spline Path...");

    std::ifstream input(mesh_file_path, std::ios::binary);
    Triangle_mesh tmesh;
    n_geodesic.load_stl_file(input, tmesh);

    Tree *tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    Surface_mesh_shortest_path *shortest_paths = new Surface_mesh_shortest_path(tmesh);

    waypoints_msg.waypoints.clear();

    if (selected_points.size() > 2)
    {
        nrs_path2::Waypoints path_points = n_geodesic.GenerateHermiteSplinePath(selected_points, tmesh);
        geodesic_waypoints_pub->publish(path_points);

        RCLCPP_INFO(node_->get_logger(), "Published Hermite Spline Path with %zu waypoints", path_points.waypoints.size());

        n_io.clearFile(geodesic_waypoints_file_path);
        n_io.saveWaypointsToFile(path_points, geodesic_waypoints_file_path);
        return true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Not enough selected points for Hermite Spline Path generation.");
        return false;
    }
}

bool nrs_callback::straightPathServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    RCLCPP_INFO(node_->get_logger(), "Straight path service called. Generating Geodesic Path...");

    std::ifstream input(mesh_file_path, std::ios::binary);
    Triangle_mesh tmesh;
    n_geodesic.load_stl_file(input, tmesh);

    Tree *tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    Surface_mesh_shortest_path *shortest_paths = new Surface_mesh_shortest_path(tmesh);

    waypoints_msg.waypoints.clear();

    if (selected_points.size() > 1)
    {
        nrs_path2::Waypoints path_points = n_geodesic.GenerateStraightGeodesicPath(selected_points, tmesh);
        geodesic_waypoints_pub->publish(path_points);

        RCLCPP_INFO(node_->get_logger(), "Published Geodesic Path with %zu waypoints", path_points.waypoints.size());

        n_io.clearFile(geodesic_waypoints_file_path);
        n_io.saveWaypointsToFile(path_points, geodesic_waypoints_file_path);
        return true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Not enough selected points for Geodesic Path generation.");
        return false;
    }
}

bool nrs_callback::PathInterpolationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    std::ifstream input(mesh_file_path, std::ios::binary);
    if (!input.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open mesh file: %s", mesh_file_path.c_str());
        return false;
    }

    Triangle_mesh tmesh;
    if (!n_geodesic.load_stl_file(input, tmesh))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load mesh from file: %s", mesh_file_path.c_str());
        return false;
    }
    input.close();

    Tree *tree = new Tree(tmesh.faces().begin(), tmesh.faces().end(), tmesh);
    tree->accelerate_distance_queries();
    Surface_mesh_shortest_path *shortest_paths = new Surface_mesh_shortest_path(tmesh);

    auto start_time_point = std::chrono::high_resolution_clock::now();

    nrs_path2::Waypoints final_waypoints = n_interpolation.interpolateEnd2End(geodesic_path, desired_interval, tmesh, Fx, Fy, Fz);

    if (final_waypoints.waypoints.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Interpolation produced no waypoints.");
        delete tree;
        delete shortest_paths;
        return false;
    }

    interpolated_waypoints_pub->publish(final_waypoints);
    n_io.clearFile(interpolated_waypoints_file_path);
    n_io.saveWaypointsToFile(final_waypoints, interpolated_waypoints_file_path);

    auto end_time_point = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_point - start_time_point).count();
    std::cout << "Interpolation & Normal smoothing time: " << duration << " s" << std::endl;

    delete tree;
    delete shortest_paths;
    return true;
}

bool nrs_callback::pathDeleteCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    // 시각화 마커 제거
    n_visualization.deleteMarkers();

    // 내부 데이터 초기화
    selected_points.clear();
    geodesic_path.waypoints.clear();

    // 빈 메시지 생성 및 발행
    nrs_path2::msg::Waypoints empty_waypoints;
    interpolated_waypoints_pub->publish(empty_waypoints);

    // 로그 출력 ( )
    RCLCPP_INFO_STREAM(rclcpp::get_logger("nrs_callback"), "Path deleted via pathDeleteCallback");
    // RCLCPP_INFO(node_->get_logger(), "Path deleted via pathDeleteCallback");

    return true;
}
// bool nrs_callback::pathDeleteCallback(
//     const std::shared_ptr<std_srvs::srv::Empty::Request> req,
//     std::shared_ptr<std_srvs::srv::Empty::Response> res)
// {
//     n_visualization.deleteMarkers();
//     selected_points.clear();
//     geodesic_path.waypoints.clear();
//     nrs_path2::Waypoints empty_waypoints;
//     interpolated_waypoints_pub->publish(empty_waypoints);

//     RCLCPP_INFO(node_->get_logger(), "Path deleted via pathDeleteCallback");
//     return true;
// }
