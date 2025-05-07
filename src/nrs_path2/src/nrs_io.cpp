#include "nrs_io.h"
#include <fstream>
#include <sstream>
#include <iostream>

void nrs_io::clearFile(const std::string &file_path)
{
    std::ofstream file(file_path, std::ofstream::trunc);
    if (!file.is_open())
        std::cerr << "Failed to open file: " << file_path << std::endl;
    else
        file.close();
}

void nrs_io::saveWaypointsToFile(const nrs_path2::msg::Waypoints &final_waypoints,
                                 const std::string &file_path)
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file " << file_path << std::endl;
        return;
    }
    for (size_t i = 0; i < final_waypoints.waypoints.size(); ++i)
    {
        const auto &wp = final_waypoints.waypoints[i];
        double roll, pitch, yaw;
        roll = pitch = yaw = 0;
        n_math.quaternionToRPY(wp.qx, wp.qy, wp.qz, wp.qw, roll, pitch, yaw);

        file << wp.x << " " << wp.y << " " << wp.z << " "
             << roll << " " << pitch << " " << yaw << " "
             << wp.fx << " " << wp.fy << " " << wp.fz << "\n";
    }
    file.close();
    std::cout << "Waypoints saved to " << file_path << std::endl;
}

void nrs_io::sendFile(const std::string &file_path,
                      const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &file_pub,
                      const rclcpp::Node::SharedPtr &node)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open file: %s", file_path.c_str());
        return;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string file_data = buffer.str();
    file.close();

    std_msgs::msg::String msg;
    msg.data = file_data;

    RCLCPP_INFO(node->get_logger(), "Sending file data...");
    file_pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "File data sent.");
}
