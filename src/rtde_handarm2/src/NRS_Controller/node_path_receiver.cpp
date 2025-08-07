#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <iostream>

class PathReceiver : public rclcpp::Node
{
public:
    PathReceiver()
    : Node("path_receiver")
    {
        // 서브스크라이버 생성
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "path_publisher",
            10,
            std::bind(&PathReceiver::fileCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for file data...");
    }

private:
    void fileCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        //// std::string output_file_path = "/home/nrsur10/catkin_ws/src/rtde_handarm/src/Control_data/hand_g_recording.txt";
        std::string output_file_path = "/home/eunseop/nrs_ws/src/rtde_handarm2/src/Control_data/hand_g_recording.txt";
        std::ofstream output_file(output_file_path);
        if (!output_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", output_file_path.c_str());
            return;
        }

        output_file << msg->data;
        output_file.close();

        RCLCPP_INFO(this->get_logger(), "File data received and saved to %s", output_file_path.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathReceiver>());
    rclcpp::shutdown();
    return 0;
}
