#include "FT_Processing.hpp"
#include <signal.h>
#include <rclcpp/rclcpp.hpp>

static std::shared_ptr<rclcpp::Node> g_node = nullptr;

void catch_signal(int sig)
{
  rclcpp::shutdown();
  if (g_node) {
    RCLCPP_ERROR(g_node->get_logger(), "Program was terminated (signal %d)", sig);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ft_aq_main");
  g_node = node;

  // 시그널 처리
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);

  // 파라미터
  int handle_id_i  = node->declare_parameter<int>("HandleID", 0x01);
  int contact_id_i = node->declare_parameter<int>("ContactID", 0x11);
  double sensor_sampling = node->declare_parameter<double>("Sensor_sampling", 0.001);
  bool hacc_switch = node->declare_parameter<bool>("HandleACC", false);
  bool cacc_switch = node->declare_parameter<bool>("ContactACC", false);

  unsigned char Handle_ID  = static_cast<unsigned char>(handle_id_i);
  unsigned char Contact_ID = static_cast<unsigned char>(contact_id_i);

  // FT_processing 은 우리가 ROS2 스타일로 바꿔둔 헤더 버전 기준
  FT_processing ftp(node, sensor_sampling, Handle_ID, Contact_ID, hacc_switch, cacc_switch);
  ftp.FT_run();   // 이 안에서 계속 도는 구조라고 가정

  rclcpp::shutdown();
  return 0;
}
