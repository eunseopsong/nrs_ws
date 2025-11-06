#include "HbuttonCmd.h"
#include <signal.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>

std::shared_ptr<HbuttonCmd> node;

void catch_signal(int sig) {
    printf("Program was terminated \n");
    rclcpp::shutdown();
    exit(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<HbuttonCmd>();

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    // HButton_main() 루프를 별도 스레드로 실행
    std::thread hbutton_thread([&]() {
        node->HButton_main();
    });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // service 콜백 등 처리

    hbutton_thread.join();  // main 종료 시 루프도 종료되도록 join
    rclcpp::shutdown();
    return 0;
}
