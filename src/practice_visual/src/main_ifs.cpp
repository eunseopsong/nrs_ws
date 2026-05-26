#include "ifs_algorithm.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 우리가 만든 IfsAlgorithm 클래스를 ROS 2 노드로 띄움
  auto node = std::make_shared<IfsAlgorithm>();
  
  RCLCPP_INFO(node->get_logger(), "Contactsensing node started!!");
  
  // 노드가 종료될 때까지 무한 대기하며 콜백과 타이머 처리
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}