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
    RCLCPP_INFO(get_logger(),
      "InterpolationNode started; waiting for /waypoints");
  }

private:
  void on_waypoints(const Float64MultiArray::SharedPtr msg)
  {
    const auto &d = msg->data;
    if (d.size() % 3 != 0) {
      RCLCPP_ERROR(get_logger(),
        "Received data size %zu not a multiple of 3", d.size());
      return;
    }
    // (x,y,z) 리스트로 변환
    std::vector<std::array<double,3>> wpts;
    for (size_t i = 0; i < d.size(); i += 3) {
      wpts.push_back({d[i], d[i+1], d[i+2]});
    }
    RCLCPP_INFO(get_logger(),
      "Got %zu waypoints; interpolating...", wpts.size());

    // 보간 수행
    std::vector<double> traj = interpolate(wpts);

    // 1) 전체 trajectory를 한 번에 publish
    Float64MultiArray out;
    out.data = traj;
    pub_->publish(out);
    RCLCPP_INFO(get_logger(),
      "Published %zu trajectory points", traj.size()/6);

    // 2) 보간된 각 점을 터미널에 출력 (publish 주기에 맞춰)
    for (size_t idx = 0; idx < traj.size(); idx += 6) {
      double x    = traj[idx + 0];
      double y    = traj[idx + 1];
      double z    = traj[idx + 2];
      double roll = traj[idx + 3];
      double pitch= traj[idx + 4];
      double yaw  = traj[idx + 5];
      RCLCPP_INFO(get_logger(),
        "  [%3zu] x=%6.3f, y=%6.3f, z=%6.3f, r=%4.2f, p=%4.2f, y=%4.2f",
        idx/6, x, y, z, roll, pitch, yaw);
    }
  }

  std::vector<double> interpolate(
    const std::vector<std::array<double,3>>& wpts)
  {
    std::vector<double> result;
    const int steps = 100;
    for (size_t i = 0; i + 1 < wpts.size(); ++i) {
      const auto& a = wpts[i];
      const auto& b = wpts[i+1];
      for (int s = 0; s <= steps; ++s) {
        double t = double(s) / steps;
        double x = a[0] * (1-t) + b[0] * t;
        double y = a[1] * (1-t) + b[1] * t;
        double z = a[2] * (1-t) + b[2] * t;
        // roll/pitch/yaw = 0
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
