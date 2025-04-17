#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class WaypointPublisher : public rclcpp::Node
{
public:
  WaypointPublisher()
  : Node("waypoint_publisher")
  {
    pub_ = this->create_publisher<Float64MultiArray>("waypoints", 10);

    // 사용자로부터 waypoint 한 번만 입력받기
    read_waypoints();

    // 1Hz 타이머로 계속 publish
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WaypointPublisher::on_timer, this)
    );

    RCLCPP_INFO(this->get_logger(), "WaypointPublisher ready, publishing at 1Hz.");
  }

private:
  void read_waypoints()
  {
    int count;
    std::cout << "생성할 waypoint의 개수를 입력하세요: ";
    std::cin >> count;

    coords_.clear();
    for (int i = 1; i <= count; ++i) {
      double x, y, z;
      std::cout << "Waypoint " << i << " (x y z): ";
      std::cin >> x >> y >> z;
      coords_.push_back(x);
      coords_.push_back(y);
      coords_.push_back(z);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "입력된 waypoints: [%s]",
      vector_to_string(coords_).c_str()
    );
  }

  void on_timer()
  {
    auto msg = Float64MultiArray();
    msg.data = coords_;
    pub_->publish(msg);
  }

  static std::string vector_to_string(const std::vector<double>& v)
  {
    std::ostringstream ss;
    for (size_t i = 0; i < v.size(); ++i) {
      ss << v[i] << (i + 1 < v.size() ? ", " : "");
    }
    return ss.str();
  }

  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<double> coords_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointPublisher>());
  rclcpp::shutdown();
  return 0;
}
