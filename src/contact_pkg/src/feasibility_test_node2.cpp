#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class FeasibilityTestNode2 : public rclcpp::Node
{
public:
  FeasibilityTestNode2()
  : Node("feasibility_test_node2")
  {
    cmd_motion_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/ur10skku/cmdMotion", 10);

    current_p_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ur10skku/currentP", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "currentP 데이터 길이가 부족합니다. expected >= 6, got %zu", msg->data.size());
          return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < current_pos_.size(); ++i) {
          current_pos_[i] = msg->data[i];
        }
        has_current_pos_ = true;
      });

    ft_data_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/ur10skku/ftdata", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_ft_ = {
          msg->wrench.force.x,
          msg->wrench.force.y,
          msg->wrench.force.z,
          msg->wrench.torque.x,
          msg->wrench.torque.y,
          msg->wrench.torque.z};
        has_current_ft_ = true;
      });

    input_thread_ = std::thread(&FeasibilityTestNode2::waitForUserInput, this);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&FeasibilityTestNode2::controlLoop, this));

    RCLCPP_INFO(get_logger(), "피지빌리티 테스트 노드 V2 (원위치 복귀 기능 탑재) 실행 완료.");
  }

  ~FeasibilityTestNode2() override
  {
    stop_input_.store(true);
    if (input_thread_.joinable()) {
      input_thread_.detach();
    }
  }

private:
  static double parseOrDefault(const std::string & input, double default_value)
  {
    if (input.empty()) {
      return default_value;
    }
    return std::stod(input);
  }

  void waitForUserInput()
  {
    while (rclcpp::ok() && !stop_input_.load()) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (has_current_pos_) {
          home_pos_ = current_pos_;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    while (rclcpp::ok() && !stop_input_.load()) {
      std::cout << "\n" << std::string(50, '=') << "\n";
      std::cout << " [로봇 제어 커맨드 라인]\n";
      std::cout << "  s : 새로운 테스트 시작 (목표 위치 지정 후 강하)\n";
      std::cout << "  h : 원위치(Home)로 강제 복귀\n";
      std::cout << std::string(50, '=') << "\n";

      std::string cmd;
      std::cout << "명령어 입력 (s 또는 h): ";
      if (!std::getline(std::cin, cmd)) {
        return;
      }

      if (cmd == "s") {
        try {
          std::array<double, 6> pos{};
          double current_fixed_wx = 0.0;
          double current_fixed_wy = 0.0;
          double current_fixed_wz = 1.57;
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            pos = current_pos_;
            current_fixed_wx = fixed_wx_;
            current_fixed_wy = fixed_wy_;
            current_fixed_wz = fixed_wz_;
          }

          std::string in_x;
          std::string in_y;
          std::string in_wx;
          std::string in_wy;
          std::string in_wz;

          std::cout << "이동할 목표 X 위치 (mm, 그대로 두려면 엔터): ";
          std::getline(std::cin, in_x);
          std::cout << "이동할 목표 Y 위치 (mm, 그대로 두려면 엔터): ";
          std::getline(std::cin, in_y);
          std::cout << "\n자세(Orientation) 설정도 변경하시겠습니까? (rad 단위)\n";
          std::cout << "목표 WX (기본값 " << current_fixed_wx << "): ";
          std::getline(std::cin, in_wx);
          std::cout << "목표 WY (기본값 " << current_fixed_wy << "): ";
          std::getline(std::cin, in_wy);
          std::cout << "목표 WZ (기본값 " << current_fixed_wz << "): ";
          std::getline(std::cin, in_wz);

          const double new_target_x = parseOrDefault(in_x, pos[0]);
          const double new_target_y = parseOrDefault(in_y, pos[1]);
          const double new_target_wx = parseOrDefault(in_wx, current_fixed_wx);
          const double new_target_wy = parseOrDefault(in_wy, current_fixed_wy);
          const double new_target_wz = parseOrDefault(in_wz, current_fixed_wz);

          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            target_x_ = new_target_x;
            target_y_ = new_target_y;
            target_wx_ = new_target_wx;
            target_wy_ = new_target_wy;
            target_wz_ = new_target_wz;
            start_z_ = current_pos_[2];
            fixed_wx_ = target_wx_;
            fixed_wy_ = target_wy_;
            fixed_wz_ = target_wz_;
            locked_pos_.reset();
            state_ = 1;
          }

          std::cout << "\n[명령 접수] X:" << new_target_x
                    << ", Y:" << new_target_y << " 로 이동 후 하강합니다.\n";
        } catch (const std::exception &) {
          std::cout << "입력 오류: 숫자를 정확히 입력해주세요.\n";
        }
      } else if (cmd == "h") {
        std::cout << "\n[인터럽트] 로봇을 최초 초기 위치(Home)로 강제 복귀시킵니다.\n";
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_ = 4;
      } else {
        std::cout << "알 수 없는 명령어입니다. 's' 또는 'h'를 입력해주세요.\n";
      }
    }
  }

  void controlLoop()
  {
    std::array<double, 6> current_pos{};
    std::array<double, 6> current_ft{};
    std::array<double, 6> home_pos{};
    std::optional<std::array<double, 6>> locked_pos;
    int state = 0;
    double target_x = 0.0;
    double target_y = 0.0;
    double target_wx = 0.0;
    double target_wy = 0.0;
    double target_wz = 1.57;
    double start_z = 0.0;
    double fixed_wx = 0.0;
    double fixed_wy = 0.0;
    double fixed_wz = 1.57;
    double fixed_fx = 0.0;
    double fixed_fy = 0.0;
    double fixed_fz = 10.0;
    bool has_home_pos = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!has_current_pos_ || !has_current_ft_) {
        return;
      }

      current_pos = current_pos_;
      current_ft = current_ft_;
      if (home_pos_.has_value()) {
        home_pos = home_pos_.value();
        has_home_pos = true;
      }
      locked_pos = locked_pos_;
      state = state_;
      target_x = target_x_;
      target_y = target_y_;
      target_wx = target_wx_;
      target_wy = target_wy_;
      target_wz = target_wz_;
      start_z = start_z_;
      fixed_wx = fixed_wx_;
      fixed_wy = fixed_wy_;
      fixed_wz = fixed_wz_;
      fixed_fx = fixed_fx_;
      fixed_fy = fixed_fy_;
      fixed_fz = fixed_fz_;
    }

    const double x = current_pos[0];
    const double y = current_pos[1];
    const double z = current_pos[2];
    const double wx = current_pos[3];
    const double wy = current_pos[4];
    const double wz = current_pos[5];
    const double fz = current_ft[2];

    std_msgs::msg::Float64MultiArray cmd_msg;

    if (state == 0) {
      return;
    }

    if (state == 1) {
      const double error_x = std::abs(target_x - x);
      const double error_y = std::abs(target_y - y);
      const double error_wx = std::abs(target_wx - wx);
      const double error_wy = std::abs(target_wy - wy);
      const double error_wz = std::abs(target_wz - wz);

      if (error_x < 1.0 && error_y < 1.0 &&
        error_wx < 0.05 && error_wy < 0.05 && error_wz < 0.05)
      {
        RCLCPP_INFO(get_logger(), "목표 위치 도달 완료! Z축 하강을 시작합니다.");
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_ = 2;
      } else {
        cmd_msg.data = {
          target_x, target_y, start_z,
          target_wx, target_wy, target_wz,
          fixed_fx, fixed_fy, fixed_fz};
        cmd_motion_pub_->publish(cmd_msg);
      }
      return;
    }

    if (state == 2) {
      if (std::abs(fz) > contact_threshold_) {
        RCLCPP_INFO(get_logger(), "표면 접촉 감지! (Fz: %.2fN). 로봇 위치를 고정합니다.", fz);
        std::lock_guard<std::mutex> lock(data_mutex_);
        locked_pos_ = current_pos_;
        state_ = 3;
      } else {
        const double target_z = z - z_step_mm_;
        cmd_msg.data = {
          target_x, target_y, target_z,
          fixed_wx, fixed_wy, fixed_wz,
          fixed_fx, fixed_fy, fixed_fz};
        cmd_motion_pub_->publish(cmd_msg);
      }
      return;
    }

    if (state == 3) {
      if (!locked_pos.has_value()) {
        return;
      }

      cmd_msg.data = {
        locked_pos->at(0), locked_pos->at(1), locked_pos->at(2),
        locked_pos->at(3), locked_pos->at(4), locked_pos->at(5),
        fixed_fx, fixed_fy, fixed_fz};
      cmd_motion_pub_->publish(cmd_msg);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "로봇 정지 및 접촉 유지 중... (원위치 복귀는 터미널에 h 입력)");
      return;
    }

    if (state == 4) {
      if (!has_home_pos) {
        return;
      }

      const double error_x = std::abs(home_pos[0] - x);
      const double error_y = std::abs(home_pos[1] - y);
      const double error_z = std::abs(home_pos[2] - z);

      if (error_x < 1.0 && error_y < 1.0 && error_z < 1.0) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!home_reached_) {
          RCLCPP_INFO(get_logger(), "원위치 복귀 완료! 터미널에서 새 명령(s)을 입력하세요.");
          home_reached_ = true;
        }
        state_ = 0;
      } else {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          home_reached_ = false;
        }
        cmd_msg.data = {
          home_pos[0], home_pos[1], home_pos[2],
          home_pos[3], home_pos[4], home_pos[5],
          0.0, 0.0, 0.0};
        cmd_motion_pub_->publish(cmd_msg);
      }
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_motion_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_data_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread input_thread_;
  std::atomic_bool stop_input_{false};
  std::mutex data_mutex_;

  std::array<double, 6> current_pos_{};
  std::array<double, 6> current_ft_{};
  std::optional<std::array<double, 6>> home_pos_;
  std::optional<std::array<double, 6>> locked_pos_;
  bool has_current_pos_{false};
  bool has_current_ft_{false};
  bool home_reached_{false};

  int state_{0};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_wx_{0.0};
  double target_wy_{0.0};
  double target_wz_{1.57};
  double start_z_{0.0};
  double fixed_wx_{0.0};
  double fixed_wy_{0.0};
  double fixed_wz_{1.57};
  double fixed_fx_{0.0};
  double fixed_fy_{0.0};
  double fixed_fz_{10.0};

  const double z_step_mm_{0.5};
  const double contact_threshold_{2.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeasibilityTestNode2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
