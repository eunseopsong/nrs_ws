#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <string>
#include <fstream>
#include <iomanip>

class ClickedPointsLogger : public rclcpp::Node
{
public:
  ClickedPointsLogger()
  : Node("clicked_points_logger")
  {
    // ==========================================
    // 1. 파라미터 선언 및 가져오기 (ROS 2 방식)
    // ==========================================
    // ROS 2에서는 파라미터를 먼저 '선언(declare)'해야 사용할 수 있습니다.
    this->declare_parameter<std::string>("input_topic", "/clicked_point");
    this->declare_parameter<std::string>("marker_topic", "/visualization_marker");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("waypoint_file", "selected_waypoints.txt");
    this->declare_parameter<double>("marker_scale", 0.005);
    this->declare_parameter<bool>("clear_file_on_start", true);

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("marker_topic", marker_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("waypoint_file", waypoint_file_);
    this->get_parameter("marker_scale", marker_scale_);
    this->get_parameter("clear_file_on_start", clear_on_start_);

    // ==========================================
    // 2. Publisher & Subscriber 설정
    // ==========================================
    // QoS 설정을 기본값(10)으로 합니다.
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
    
    // std::bind를 사용하여 콜백 함수 연결
    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      input_topic_, 10,
      std::bind(&ClickedPointsLogger::cbPoint, this, std::placeholders::_1));

    // ==========================================
    // 3. 초기화 로직
    // ==========================================
    if (clear_on_start_) {
      writeAllToFile(); // empty list -> file cleared
      RCLCPP_INFO_STREAM(this->get_logger(), "Truncated file at start: " << waypoint_file_);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Listening: " << input_topic_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing markers on: " << marker_topic_ << " with frame_id=" << frame_id_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Writing waypoints to: " << waypoint_file_);
  }

private:
  void cbPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // 포인트 저장
    points_.push_back(msg->point);

    // 마커 생성
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->now(); // ROS 2 시간
    marker.ns = "clicked_points";
    marker.id = next_id_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = msg->point;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // 0초는 영구 지속을 의미 (ROS 1과 동일 개념)
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);

    marker_pub_->publish(marker);

    // 파일 쓰기
    if (!writeAllToFile()) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to write waypoints to " << waypoint_file_);
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "Saved " << points_.size() << " waypoint(s) to " << waypoint_file_);
    }
  }

  bool writeAllToFile()
  {
    std::ofstream ofs(waypoint_file_, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) return false;

    ofs << std::fixed << std::setprecision(6);
    ofs << "# x y z\n";
    for (const auto& p : points_) {
      ofs << p.x << " " << p.y << " " << p.z << "\n";
    }
    ofs.close();
    return true;
  }

private:
  // ROS 2 Publisher/Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::string input_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  std::string waypoint_file_;
  double marker_scale_;
  bool clear_on_start_;

  std::vector<geometry_msgs::msg::Point> points_;
  int next_id_ {0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClickedPointsLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}