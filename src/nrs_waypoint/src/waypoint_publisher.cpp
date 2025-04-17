#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <array>
#include <iostream>

using std::placeholders::_1;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class WaypointPublisher : public rclcpp::Node
{
public:
  WaypointPublisher()
  : Node("waypoint_publisher")
  {
    // 퍼블리셔 생성
    pub_ = create_publisher<Float64MultiArray>("waypoints", 10);

    // 사용자로부터 입력받기
    get_user_input();

    // 타이머 생성 (1Hz)
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WaypointPublisher::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "WaypointPublisher ready, publishing at 1Hz.");
  }

private:
  void get_user_input()
  {
    int num;
    std::cout << "생성할 waypoint의 개수를 입력하세요: ";
    std::cin >> num;

    for (int i = 0; i < num; ++i) {
      std::array<double, 3> pt;
      std::cout << "Waypoint " << i+1 << " (x y z): ";
      std::cin >> pt[0] >> pt[1] >> pt[2];
      waypoints_.push_back(pt);
    }

    RCLCPP_INFO(this->get_logger(), "입력된 waypoints:");
    for (const auto& pt : waypoints_) {
      RCLCPP_INFO(this->get_logger(), "[%.1f, %.1f, %.1f]", pt[0], pt[1], pt[2]);
    }
  }

  void on_timer()
  {
    Float64MultiArray msg;
    for (const auto& pt : waypoints_) {
      msg.data.push_back(pt[0]);
      msg.data.push_back(pt[1]);
      msg.data.push_back(pt[2]);
    }
    pub_->publish(msg);
  }

  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::array<double, 3>> waypoints_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
