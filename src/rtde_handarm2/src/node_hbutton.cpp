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


// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<HbuttonCmd>();

//     // 시그널 핸들러 등록
//     signal(SIGTERM, catch_signal);
//     signal(SIGINT, catch_signal);

//     // 사용자 정의 main loop 실행 (while 루프)
//     node->HButton_main();

//     rclcpp::shutdown();
//     return 0;
// }

//// int main(int argc, char **argv)
//// {
////     ros::init(argc,argv,"NRS_Hbutton_cmd");
////     ros::NodeHandle _nh;
////     NRS_Hbutton_cmd NRS_HB_cmd(_nh,100);

////     signal(SIGTERM, catch_signal);// Termination
//// 	signal(SIGINT, catch_signal);// Active

////     NRS_HB_cmd.HButton_main();

////     return 0;
//// }
