#include "JointControl.h"

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<JointControl>();
//     rclcpp::executors::SingleThreadedExecutor exec;
//     exec.add_node(node);

//     node->initializeMonitoring();  // 꼭 make_shared 이후에

//     exec.spin();
//     rclcpp::shutdown();
// }




int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Yoon_UR10e_main");

    // 생성자에 node 넘기기
    auto joint_ctrl = std::make_shared<JointControl>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

