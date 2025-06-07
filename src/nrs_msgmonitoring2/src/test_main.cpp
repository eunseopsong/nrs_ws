#include "nrs_msgmonitoring2/msg_monitoring.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("msg_monitoring_test_node");

  std::string topic_name = "test_topic";
  nrs_msgmonitoring2::MsgMonitoring msg_monitoring(node, topic_name);

  std::vector<double> test_data = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::string test_string = "test_string";

  rclcpp::Rate loop_rate(10);  // 10 Hz

  while (rclcpp::ok())
  {
    msg_monitoring.Mon1_publish(test_data, test_string, true);
    msg_monitoring.Mon2_publish(test_data, test_string, true);
    msg_monitoring.Mon3_publish(test_data, test_string, true);
    msg_monitoring.Mon4_publish(test_data, test_string, true);
    msg_monitoring.Mon5_publish(test_data, test_string, true);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
