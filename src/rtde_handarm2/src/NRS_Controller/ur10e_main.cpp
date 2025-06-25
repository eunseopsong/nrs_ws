#include "JointControl.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<JointControl>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    node->initializeMonitoring();  // 꼭 make_shared 이후에

    exec.spin();
    rclcpp::shutdown();
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
