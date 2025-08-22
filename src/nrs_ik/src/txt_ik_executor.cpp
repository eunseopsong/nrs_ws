#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>
#include <cmath>
#include <iterator>

// ====== 네가 제공한 헤더들 (경로는 프로젝트에 맞게 수정) ======
#include "Arm_class.h"
#include "Kinematics.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

struct PoseRPY {
  size_t line_no;     // ✅ 추가: 원본 파일의 행 번호
  double x, y, z;
  double r, p, yaw;
};

class TxtIKExecutor : public rclcpp::Node {
public:
  TxtIKExecutor()
  : Node("txt_ik_executor")
  {
    // --- Parameters ---
    txt_path_    = this->declare_parameter<std::string>("txt_path",
                    "/home/eunseop/nrs_ws/src/nrs_path2/data/visual_final_waypoints.txt");
    hz_          = this->declare_parameter<double>("hz", 100.0);
    use_degrees_ = this->declare_parameter<bool>("use_degrees", false);

    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/isaac_joint_commands", rclcpp::QoS(10).reliable());

    joint_state_.name = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    joint_state_.position.assign(6, 0.0);

    // CArm / Kinematics 초기화
    //  - CArm은 네 코드 기준으로 Eigen 타입 멤버(qc, qd, Td, q, Tc/Td 등)를 가진다고 가정
    init_arm_state_();

    // Trajectory 로드
    if (!load_txt(txt_path_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load txt: %s", txt_path_.c_str());
      throw std::runtime_error("txt load failed");
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu poses from %s", poses_.size(), txt_path_.c_str());

    // 타이머 시작
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TxtIKExecutor::step_once, this)
    );

    start_time_ = last_time_ = std::chrono::high_resolution_clock::now();
  }

private:
  // Params
  std::string txt_path_;
  double hz_;
  bool use_degrees_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_state_;

  // Trajectory
  std::vector<PoseRPY> poses_;
  size_t idx_{0};

  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_, last_time_;

  // Kinematics
  Kinematic_func kin_;   // 네가 준 클래스
  CArm arm_;             // 네가 준 클래스 인스턴스

  void init_arm_state_() {
    // CArm 멤버들 초기화 (크기/단위 맞춰줌)
    // qd, qc, q 등은 6x1, Td는 4x4, Tc/Td/QuatM/QuatM4 등 Eigen 할당을 가정
    arm_.qc = Eigen::Vector<double, 6>::Zero();
    arm_.qd = Eigen::Vector<double, 6>::Zero();
    arm_.q  = Eigen::Vector<double, 6*8>::Zero(); // 해 8개까지 담는 공간(UR계열 통상 8해)
    arm_.Td = Matrix4d::Identity();
    arm_.Tc = Matrix4d::Identity();

    // R2E 관련 플래그도 무난히 초기화
    arm_.R2E_init_flag = false;
    arm_.pre_thc = Eigen::Vector3d::Zero();
    arm_.thc     = Eigen::Vector3d::Zero();
  }

  bool load_txt(const std::string& path) {
    std::ifstream infile(path);
    if (!infile.is_open()) return false;

    std::string line;
    size_t line_no = 0;
    poses_.clear();
    poses_.reserve(10000);

    while (std::getline(infile, line)) {
      ++line_no;
      // 공백/주석 라인 스킵
      if (line.empty()) continue;
      if (line[0] == '#') continue;

      std::istringstream iss(line);
      std::vector<double> cols{ std::istream_iterator<double>(iss), std::istream_iterator<double>() };

      if (cols.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "Line %zu has <6 columns, skipped.", line_no);
        continue;
      }

      PoseRPY p;
      p.line_no = line_no;   // ✅ 행 번호 저장
      p.x   = cols[0];
      p.y   = cols[1];
      p.z   = cols[2];
      p.r   = cols[3];
      p.p   = cols[4];
      p.yaw = cols[5];

      if (use_degrees_) {
        constexpr double D2R = M_PI / 180.0;
        p.r   *= D2R;
        p.p   *= D2R;
        p.yaw *= D2R;
      }

      poses_.push_back(p);
    }
    return !poses_.empty();
  }

  void step_once() {
    if (idx_ >= poses_.size()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "All poses processed.");
      timer_->cancel();
      return;
    }

    auto now = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - start_time_).count();
    double delta_ms   = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - last_time_).count();
    last_time_ = now;

    const auto& P = poses_[idx_++];

    // ✅ 요청한 간결 출력: 현재 줄 번호 + x y z r p y
    RCLCPP_INFO(this->get_logger(),
      "[line %zu] x=%.6f y=%.6f z=%.6f r=%.6f p=%.6f y=%.6f",
      P.line_no, P.x, P.y, P.z, P.r, P.p, P.yaw);

    // 1) RPY -> Rotation
    Vector3d th;   // roll, pitch, yaw
    th << P.r, P.p, P.yaw;

    Matrix3d R;
    kin_.EulerAngle2Rotation(R, th);  // 제공 함수 그대로 사용 (roll-pitch-yaw 순서)

    // 2) Td 구성 (pos: x,y,z / rot: R)
    arm_.Td.setIdentity();
    arm_.Td.block<3,3>(0,0) = R;
    arm_.Td(0,3) = P.x;
    arm_.Td(1,3) = P.y;
    arm_.Td(2,3) = P.z;

    // 3) 현재자세(qc)를 직전 해(qd)로 이어서 연속해 찾기
    arm_.qc = arm_.qd;

    // 4) InverseK_min (해 중에서 현재자세와 가장 가까운 해 선택)
    int nsol = kin_.InverseK_min(&arm_);

    if (nsol <= 0) {
      RCLCPP_WARN(this->get_logger(), "[line %zu] IK failed. Keeping last q.", P.line_no);
      // 실패 시: arm_.qd 유지 (이전 명령값 유지)
    } else {
      // arm_.qd 에 최소오차 해가 저장됨
    }

    // 5) Publish
    joint_state_.position[0] = arm_.qd(0);
    joint_state_.position[1] = arm_.qd(1);
    joint_state_.position[2] = arm_.qd(2);
    joint_state_.position[3] = arm_.qd(3);
    joint_state_.position[4] = arm_.qd(4);
    joint_state_.position[5] = arm_.qd(5);
    joint_state_.header.stamp = this->now();
    pub_->publish(joint_state_);

    // 🔇 기존 상세 디버그 출력은 유지하되 주석 처리
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //   "step %zu/%zu | elapsed %.2f ms | dt %.2f ms | q = [%.3f %.3f %.3f %.3f %.3f %.3f]",
    //   idx_, poses_.size(), elapsed_ms, delta_ms,
    //   joint_state_.position[0], joint_state_.position[1], joint_state_.position[2],
    //   joint_state_.position[3], joint_state_.position[4], joint_state_.position[5]);
  }
};

int main(int argc, char* argv[]) {
  // 실시간/메모리 잠금 (가능하면)
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "mlockall failed");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Memory locked.");
  }

  struct sched_param sp{ .sched_priority = 80 };
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to set real-time scheduling");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TxtIKExecutor>());
  rclcpp::shutdown();
  return 0;
}
