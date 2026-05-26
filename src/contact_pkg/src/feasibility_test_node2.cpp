#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class FeasibilityTestNode2 : public rclcpp::Node
{
public:
  FeasibilityTestNode2()
  : Node("feasibility_test_node2")
  {
    interpolation_duration_sec_ = declare_parameter<double>("interpolation_duration_sec", 3.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);

    cmd_motion_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/ur10skku/cmdMotion", 10);
    current_p_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ur10skku/currentP", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < current_pose_.size()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "currentP 데이터 길이가 부족합니다. expected >= 6, got %zu", msg->data.size());
          return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < current_pose_.size(); ++i) {
          current_pose_[i] = msg->data[i];
        }
        has_current_pose_ = true;
      });

    input_thread_ = std::thread(&FeasibilityTestNode2::waitForUserInput, this);

    RCLCPP_INFO(
      get_logger(),
      "9D command P2P 보간 입력 모드 실행. 예: 870 340 100 0 0 1.57 0 0 10");
    RCLCPP_INFO(
      get_logger(),
      "P2P interpolation: duration=%.3fs, rate=%.1fHz",
      interpolation_duration_sec_, publish_rate_hz_);
  }

  ~FeasibilityTestNode2() override
  {
    if (input_thread_.joinable()) {
      input_thread_.detach();
    }
  }

private:
  static bool parseCommand(const std::string & line, std::array<double, 9> & command)
  {
    std::istringstream stream(line);

    for (double & value : command) {
      if (!(stream >> value)) {
        return false;
      }
    }

    std::string extra;
    return !(stream >> extra);
  }

  static double smoothStep(const double t)
  {
    return t * t * (3.0 - 2.0 * t);
  }

  std::array<double, 9> makeStartCommand(const std::array<double, 9> & target)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    std::array<double, 9> start = target;

    if (has_current_pose_) {
      for (size_t i = 0; i < current_pose_.size(); ++i) {
        start[i] = current_pose_[i];
      }
    } else if (has_last_command_) {
      for (size_t i = 0; i < 6; ++i) {
        start[i] = last_command_[i];
      }
      RCLCPP_WARN(
        get_logger(),
        "currentP를 아직 받지 못해 직전 명령 pose에서 보간합니다.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "currentP/직전 명령이 없어 입력 target pose를 시작점으로 사용합니다.");
    }

    for (size_t i = 6; i < start.size(); ++i) {
      start[i] = has_last_command_ ? last_command_[i] : 0.0;
    }

    return start;
  }

  void publishInterpolatedCommand(const std::array<double, 9> & target)
  {
    const auto start = makeStartCommand(target);

    const double duration = std::max(0.01, interpolation_duration_sec_);
    const double rate = std::max(1.0, publish_rate_hz_);
    const int steps = std::max(1, static_cast<int>(duration * rate));
    const auto period = std::chrono::duration<double>(1.0 / rate);

    RCLCPP_INFO(
      get_logger(),
      "P2P cmdMotion start -> target: "
      "[%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f] -> "
      "[%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f], steps=%d",
      start[0], start[1], start[2], start[3], start[4], start[5], start[6], start[7], start[8],
      target[0], target[1], target[2], target[3], target[4], target[5], target[6], target[7], target[8],
      steps);

    for (int step = 1; rclcpp::ok() && step <= steps; ++step) {
      const double ratio = smoothStep(static_cast<double>(step) / static_cast<double>(steps));

      std::array<double, 9> command{};
      for (size_t i = 0; i < command.size(); ++i) {
        command[i] = start[i] + (target[i] - start[i]) * ratio;
      }

      std_msgs::msg::Float64MultiArray cmd_msg;
      cmd_msg.data.assign(command.begin(), command.end());
      cmd_motion_pub_->publish(cmd_msg);

      std::this_thread::sleep_for(period);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_command_ = target;
      has_last_command_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "P2P cmdMotion target reached: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
      target[0], target[1], target[2], target[3], target[4],
      target[5], target[6], target[7], target[8]);
  }

  void waitForUserInput()
  {
    while (rclcpp::ok()) {
      std::cout << "\n9D command 입력: ";

      std::string line;
      if (!std::getline(std::cin, line)) {
        rclcpp::shutdown();
        return;
      }

      std::array<double, 9> command{};
      if (!parseCommand(line, command)) {
        std::cout << "입력 오류: 숫자 9개를 공백으로 구분해서 입력해주세요.\n";
        std::cout << "예: 870 340 100 0 0 1.57 0 0 10\n";
        continue;
      }

      publishInterpolatedCommand(command);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_motion_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;

  std::mutex state_mutex_;
  std::array<double, 6> current_pose_{};
  std::array<double, 9> last_command_{};
  bool has_current_pose_{false};
  bool has_last_command_{false};

  double interpolation_duration_sec_{3.0};
  double publish_rate_hz_{100.0};

  std::thread input_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeasibilityTestNode2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
