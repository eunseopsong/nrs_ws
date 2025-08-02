#include <rclcpp/rclcpp.hpp>        //// #include <ros/ros.h>
#include <std_msgs/msg/string.hpp>  //// #include <std_msgs/String.h>
#include <fstream>
#include <iostream>

void fileCallback(const std_msgs::String::ConstPtr& msg)
{
    // 수신된 데이터를 저장할 파일 경로
    //// std::string output_file_path = "/home/nrsur10/catkin_ws/src/rtde_handarm/src/Control_data/Hand_G_recording.txt";
    // std::string output_file_path = "/home/nrsur10/catkin_ws/src/rtde_handarm/src/Control_data/Hand_G_recording.txt";
    std::string output_file_path = "/home/eunseop/nrs_ws/src/rtde_handarm2/src/Control_data/Hand_G_recording.txt";

    // 파일 열기
    std::ofstream output_file(output_file_path);
    if (!output_file.is_open())
    {
        ROS_ERROR("Failed to open file for writing: %s", output_file_path.c_str());
        return;
    }

    // 수신된 파일 데이터를 파일에 쓰기
    output_file << msg->data;
    output_file.close();

    ROS_INFO("File data received and saved to %s", output_file_path.c_str());
}

int main(int argc, char** argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "path_receiver");
    ros::NodeHandle nh;

    // "path_publisher" 토픽 구독
    ros::Subscriber sub = nh.subscribe("path_publisher", 10, fileCallback);

    ROS_INFO("Waiting for file data...");

    ros::spin();  // 파일 데이터 수신 대기
    return 0;
}
