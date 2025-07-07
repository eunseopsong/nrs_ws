#include "HbuttonCmd.h"
#include <signal.h>

void catch_signal(int sig) {
    printf("Program was terminated \n");
    rclcpp::shutdown();
    exit(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HbuttonCmd>();

    // 시그널 핸들러 등록
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    // 사용자 정의 main loop 실행 (while 루프)
    node->HButton_main();

    rclcpp::shutdown();
    return 0;
}

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
