#include "FT_Processing.hpp"
#include <signal.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // 파라미터를 yaml에서 그냥 받아들이게 하는 옵션
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("ft_aq", options);

  int HandleID = 0x01;
  int ContactID = 0x11;
  // ★ 기본 샘플링을 0.002s (= 500 Hz)로 설정
  double Sensor_sampling = 0.002;
  bool HaccSwitch = false;
  bool CaccSwitch = false;

  // yaml에 있으면 위 값들이 덮어써진다
  node->get_parameter("HandleID", HandleID);
  node->get_parameter("ContactID", ContactID);
  node->get_parameter("Sensor_sampling", Sensor_sampling);
  node->get_parameter("HandleACC", HaccSwitch);
  node->get_parameter("ContactACC", CaccSwitch);

  unsigned char handle_id_uc = static_cast<unsigned char>(HandleID);
  unsigned char contact_id_uc = static_cast<unsigned char>(ContactID);

  auto ftp = std::make_shared<FT_processing>(
      node,
      Sensor_sampling,
      handle_id_uc,
      contact_id_uc,
      HaccSwitch,
      CaccSwitch);

  ftp->FT_run();

  rclcpp::shutdown();
  return 0;
}
