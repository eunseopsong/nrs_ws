#include "JointControl.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // 노드 생성
    auto node = std::make_shared<rclcpp::Node>("nrs_control_node");

    // JointControl 클래스 객체 생성
    auto joint_ctrl = std::make_shared<JointControl>(node);

    // 멀티스레드 executor 생성
    rclcpp::executors::MultiThreadedExecutor executor;

    // 노드 추가
    executor.add_node(node);

    // 콜백을 병렬로 처리
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

