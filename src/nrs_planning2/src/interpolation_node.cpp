#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <array>

using std::placeholders::_1;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class InterpolationNode : public rclcpp::Node
{
public:
  InterpolationNode()
  : Node("interpolation_node"),
    current_index_(0)
  {
    sub_ = create_subscription<Float64MultiArray>(
      "waypoints", 10,
      std::bind(&InterpolationNode::on_waypoints, this, _1));

    pub_ = create_publisher<Float64MultiArray>(
      "planned_trajectory", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&InterpolationNode::on_timer, this));
  }

private:
  void on_waypoints(const Float64MultiArray::SharedPtr msg)
  {
    trajectory_.clear();
    auto &d = msg->data;
    if (d.size() % 3 != 0) {
      RCLCPP_ERROR(get_logger(),
        "데이터 길이가 3의 배수가 아님: %zu", d.size());
      return;
    }
    std::vector<std::array<double,3>> wpts;
    for (size_t i = 0; i < d.size(); i += 3) {
      wpts.push_back({d[i], d[i+1], d[i+2]});
    }

    const int steps = 100;
    for (size_t i = 0; i + 1 < wpts.size(); ++i) {
      auto [x0,y0,z0] = wpts[i];
      auto [x1,y1,z1] = wpts[i+1];
      for (int s = 0; s < steps; ++s) {  // 👈 여기만 수정됨!
        double t = static_cast<double>(s) / steps;
        double x = x0*(1-t) + x1*t;
        double y = y0*(1-t) + y1*t;
        double z = z0*(1-t) + z1*t;
        trajectory_.push_back({x,y,z, 0.0,0.0,0.0});
      }
    }

    current_index_ = 0;
    RCLCPP_INFO(get_logger(),
      "총 %zu 포인트로 보간 완료. 타이머 콜백으로 1점씩 퍼블리시합니다.",
      trajectory_.size());
  }

  void on_timer()
  {
    if (current_index_ < trajectory_.size()) {
      auto &pt = trajectory_[current_index_];
      Float64MultiArray out;
      out.data = { pt[0], pt[1], pt[2], pt[3], pt[4], pt[5] };
      pub_->publish(out);

      RCLCPP_INFO(get_logger(),
        "time_step %zu: x=%.3f y=%.3f z=%.3f r=%.2f p=%.2f y=%.2f",
        current_index_,
        pt[0], pt[1], pt[2],
        pt[3], pt[4], pt[5]);

      ++current_index_;
    }
  }

  rclcpp::Subscription<Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::array<double,6>> trajectory_;
  size_t current_index_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterpolationNode>());
  rclcpp::shutdown();
  return 0;
}
