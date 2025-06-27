#include "NRS_Hbutton_cmd.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NRS_Hbutton_cmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
