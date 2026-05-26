#include <array>
#include <cmath>
#include <memory>
#include <vector>

<<<<<<< HEAD

=======
>>>>>>> origin/main
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class FeasibilityTestNode : public rclcpp::Node
{
public:
  FeasibilityTestNode()
  : Node("feasibility_test_node")
  {
    cmd_motion_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/ur10skku/cmdMotion", 10);

    current_p_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ur10skku/currentP", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 6) {
          for (size_t i = 0; i < current_pos_.size(); ++i) {
            current_pos_[i] = msg->data[i];
          }
          has_current_pos_ = true;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "currentP 데이터 길이가 부족합니다. expected >= 6, got %zu", msg->data.size());
        }
      });

    ft_data_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/ur10skku/ftdata", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        current_ft_ = {
          msg->wrench.force.x,
          msg->wrench.force.y,
          msg->wrench.force.z,
          msg->wrench.torque.x,
          msg->wrench.torque.y,
          msg->wrench.torque.z};
        has_current_ft_ = true;
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&FeasibilityTestNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "피지빌리티 테스트 노드 실행: 수직 강하 대기 중...");
  }

private:
  std::array<double, 3> contactPointDetectionAlgorithm(
    const std::array<double, 6> & pos,
    const std::array<double, 6> & /*ft*/)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "접촉 유지 중: 접촉점 탐지 알고리즘 구동...");
    return {pos[0], pos[1], pos[2]};
  }

  void controlLoop()
  {
    if (!has_current_pos_ || !has_current_ft_) {
      return;
    }

    const double x = current_pos_[0];
    const double y = current_pos_[1];
    const double z = current_pos_[2];
    const double fz = current_ft_[2];

    std_msgs::msg::Float64MultiArray cmd_msg;

    if (std::abs(fz) > contact_threshold_) {
      if (!is_contacted_) {
        RCLCPP_INFO(
          get_logger(),
          "표면 접촉 감지! (Fz: %.2fN). 탐지 알고리즘으로 전환합니다.", fz);
        is_contacted_ = true;
      }

      const auto target = contactPointDetectionAlgorithm(current_pos_, current_ft_);
      cmd_msg.data = {
        target[0], target[1], target[2],
        fixed_wx_, fixed_wy_, fixed_wz_,
        fixed_fx_, fixed_fy_, fixed_fz_};
    } else {
      is_contacted_ = false;
      const double target_z = z - z_step_mm_;
      cmd_msg.data = {
        x, y, target_z,
        fixed_wx_, fixed_wy_, fixed_wz_,
        fixed_fx_, fixed_fy_, fixed_fz_};
    }

    cmd_motion_pub_->publish(cmd_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_motion_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_data_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 6> current_pos_{};
  std::array<double, 6> current_ft_{};
  bool has_current_pos_{false};
  bool has_current_ft_{false};
  bool is_contacted_{false};

  const double fixed_wx_{0.0};
  const double fixed_wy_{0.0};
  const double fixed_wz_{1.57};
  const double fixed_fx_{0.0};
  const double fixed_fy_{0.0};
  const double fixed_fz_{0.0};
  const double z_step_mm_{0.5};
  const double contact_threshold_{2.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeasibilityTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
