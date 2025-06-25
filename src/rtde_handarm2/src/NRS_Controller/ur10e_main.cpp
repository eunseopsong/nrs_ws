#include "JointControl.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto joint_control_node = std::make_shared<JointControl>();
    joint_control_node->initializeMonitoring();  // 이제 OK

    rclcpp::spin(joint_control_node);
    rclcpp::shutdown();

    return 0;
}



// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<JointControl>();
//     node->initializeMonitoring();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
