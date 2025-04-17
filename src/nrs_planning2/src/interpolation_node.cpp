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
  : Node("interpolation_node")
  {
    sub_ = create_subscription<Float64MultiArray>(
      "waypoints", 10,
      std::bind(&InterpolationNode::on_waypoints, this, _1));
    pub_ = create_publisher<Float64MultiArray>("planned_trajectory", 10);
    RCLCPP_INFO(get_logger(), "InterpolationNode started, waiting for /waypoints");
  }

private:
  void on_waypoints(const Float64MultiArray::SharedPtr msg)
  {
    auto &d = msg->data;
    if (d.size() % 3 != 0) {
      RCLCPP_ERROR(get_logger(), "Received data size %zu not a multiple of 3", d.size());
      return;
    }
    // group into [(x,y,z), ...]
    std::vector<std::array<double,3>> wpts;
    for (size_t i = 0; i < d.size(); i += 3) {
      wpts.push_back({d[i], d[i+1], d[i+2]});
    }
    RCLCPP_INFO(get_logger(), "Got %zu waypoints", wpts.size());
    // interpolate
    std::vector<double> traj = interpolate(wpts);
    Float64MultiArray out;
    out.data = std::move(traj);
    pub_->publish(out);
    RCLCPP_INFO(get_logger(), "Published %zu trajectory points", out.data.size()/6);
  }

  std::vector<double> interpolate(const std::vector<std::array<double,3>>& wpts)
  {
    std::vector<double> result;
    const int steps = 100;
    for (size_t i = 0; i + 1 < wpts.size(); ++i) {
      auto [x0,y0,z0] = wpts[i];
      auto [x1,y1,z1] = wpts[i+1];
      for (int s = 0; s <= steps; ++s) {
        double t = double(s) / steps;
        double x = x0*(1-t) + x1*t;
        double y = y0*(1-t) + y1*t;
        double z = z0*(1-t) + z1*t;
        // roll, pitch, yaw = 0
        result.push_back(x);
        result.push_back(y);
        result.push_back(z);
        result.push_back(0.0);
        result.push_back(0.0);
        result.push_back(0.0);
      }
    }
    return result;
  }

  rclcpp::Subscription<Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterpolationNode>());
  rclcpp::shutdown();
  return 0;
}
