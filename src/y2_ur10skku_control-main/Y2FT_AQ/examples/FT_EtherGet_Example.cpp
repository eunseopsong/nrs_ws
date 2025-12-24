#include <rclcpp/rclcpp.hpp>
#include "Y2FT_AQ/FT_EtherGet.hpp"

class FT_EtherGet_Example: public rclcpp::Node
{
    public:
        FT_EtherGet_Example(): Node("FT_EtherGet_Example_node")
        {}
        void run(){
            FT_EtherGet ftrawGet("192.168.1.100",8890);
            while(rclcpp::ok())
            {
                ftrawGet.FT_init(50);
                auto ft_value = ftrawGet.FTGet();
                printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f \n",
                ft_value.Fx,ft_value.Fy,ft_value.Fz,ft_value.Mx,ft_value.My,ft_value.Mz);
            }
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FT_EtherGet_Example>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}