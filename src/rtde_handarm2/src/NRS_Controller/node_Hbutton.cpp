#include "HbuttonCmd.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HbuttonCmd>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

// int main(int argc, char **argv)
// {
//     ros::init(argc,argv,"NRS_Hbutton_cmd");
//     ros::NodeHandle _nh;
//     NRS_Hbutton_cmd NRS_HB_cmd(_nh,100);

//     signal(SIGTERM, catch_signal);// Termination
// 	signal(SIGINT, catch_signal);// Active

//     NRS_HB_cmd.HButton_main();

//     return 0;
// }
