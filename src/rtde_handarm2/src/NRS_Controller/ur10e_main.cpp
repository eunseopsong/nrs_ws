#include "JointControl.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("nrs_control_node");

    // 생성자에 node 넘기기
    auto joint_ctrl = std::make_shared<JointControl>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

