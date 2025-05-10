#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vector>
#include <array>
#include <iostream>

using std::placeholders::_1;
using PointStamped = geometry_msgs::msg::PointStamped;

class ClickedPointPublisher : public rclcpp::Node
{
public:
  ClickedPointPublisher()
  : Node("clicked_point_publisher")
  {
    pub_ = create_publisher<PointStamped>("/clicked_point", 10);
    get_user_input();

    index_ = 0;
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ClickedPointPublisher::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "Publishing waypoints to /clicked_point at 1Hz...");
  }

private:
  void get_user_input()
  {
    int num;
    std::cout << "입력할 waypoint 개수: ";
    std::cin >> num;

    for (int i = 0; i < num; ++i) {
      std::array<double, 3> pt;
      std::cout << "Waypoint " << i + 1 << " (x y z): ";
      std::cin >> pt[0] >> pt[1] >> pt[2];
      waypoints_.push_back(pt);
    }
  }

  void on_timer()
  {
    if (index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "모든 waypoint를 퍼블리시했습니다.");
      rclcpp::shutdown();
      return;
    }

    const auto& pt = waypoints_[index_];
    PointStamped msg;
    msg.header.frame_id = "base_link";  // 또는 "map" 등 상황에 맞게 조정
    msg.header.stamp = this->get_clock()->now();
    msg.point.x = pt[0];
    msg.point.y = pt[1];
    msg.point.z = pt[2];

    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published waypoint %d: [%.2f, %.2f, %.2f]", index_ + 1, pt[0], pt[1], pt[2]);
    ++index_;
  }

  rclcpp::Publisher<PointStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::array<double, 3>> waypoints_;
  size_t index_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClickedPointPublisher>());
  rclcpp::shutdown();
  return 0;
}
