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

#include "Arm_class.h"
#include "Kinematics.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

// ============== 데이터 구조 ==============
struct PoseRPY {
  size_t line_no;     // 원본 파일의 행 번호
  double x, y, z;
  double r, p, yaw;
};

// ============== 노드 ==============
class TxtIKExecutor : public rclcpp::Node {
public:
  TxtIKExecutor()
  : Node("txt_ik_executor")
  {
    // --- Parameters ---
    txt_path_       = this->declare_parameter<std::string>(
                        "txt_path",
                        "/home/eunseop/nrs_ws/src/nrs_path2/data/visual_final_waypoints.txt");
    hz_             = this->declare_parameter<double>("hz", 100.0);
    use_degrees_    = this->declare_parameter<bool>("use_degrees", false);
    tool_z_         = this->declare_parameter<double>("tool_z", 0.239); // ✅ EE +Z 기준 TCP 오프셋(m)

    move_home_s_     = this->declare_parameter<double>("move_home_s", 2.0);
    home_hold_s_     = this->declare_parameter<double>("home_hold_s", 3.0); // ✅ 요구 3초
    move_to_first_s_ = this->declare_parameter<double>("move_to_first_s", 3.0);
    return_home_s_   = this->declare_parameter<double>("return_home_s", 2.0);

    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/isaac_joint_commands", rclcpp::QoS(10).reliable());

    joint_state_.name = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    joint_state_.position.assign(6, 0.0);

    // CArm / Kinematics 초기화
    init_arm_state_();

    // Trajectory 로드
    if (!load_txt(txt_path_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load txt: %s", txt_path_.c_str());
      throw std::runtime_error("txt load failed");
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu poses from %s", poses_.size(), txt_path_.c_str());

    // 홈자세(rad): (0, -90, -90, -90, 90, 0)°
    home_q_ << 0.0, -M_PI/2, -M_PI/2, -M_PI/2, +M_PI/2, 0.0;

    // 시작 상태: 현재 명령값 = (0,...,0)에서 홈으로 이동 시작
    current_cmd_.setZero();
    start_joint_trajectory(current_cmd_, home_q_, move_home_s_);
    phase_ = Phase::MOVE_HOME;

    // 타이머 시작
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TxtIKExecutor::on_timer, this)
    );

    start_time_ = last_time_ = std::chrono::high_resolution_clock::now();
  }

private:
  // ===== 파라미터/상수 =====
  std::string txt_path_;
  double hz_;
  bool use_degrees_;
  double tool_z_;

  double move_home_s_;
  double home_hold_s_;
  double move_to_first_s_;
  double return_home_s_;

  // ===== ROS =====
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_state_;

  // ===== 텍스트 궤적 =====
  std::vector<PoseRPY> poses_;
  size_t idx_{0}; // RUN_PATH에서 사용할 인덱스

  // ===== 시간 =====
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_, last_time_;

  // ===== 기구학 =====
  Kinematic_func kin_;
  CArm arm_;

  // ===== 상태기계 =====
  enum class Phase { MOVE_HOME, HOLD_HOME, MOVE_TO_FIRST, RUN_PATH, RETURN_HOME, DONE };
  Phase phase_{Phase::MOVE_HOME};

  // ===== 관절 보간 =====
  Eigen::Matrix<double,6,1> home_q_;
  Eigen::Matrix<double,6,1> current_cmd_;
  Eigen::Matrix<double,6,1> traj_q_start_, traj_q_goal_;
  int traj_steps_total_{0};
  int traj_step_i_{0};

  int hold_steps_left_{0}; // HOLD_HOME에서만 사용

  // ========== 초기화 ==========
  void init_arm_state_() {
    arm_.qc = Eigen::Vector<double, 6>::Zero();
    arm_.qd = Eigen::Vector<double, 6>::Zero();
    arm_.q  = Eigen::Vector<double, 6*8>::Zero(); // 해 8개까지 담는 공간
    arm_.Td = Matrix4d::Identity();
    arm_.Tc = Matrix4d::Identity();
    arm_.R2E_init_flag = false;
    arm_.pre_thc = Eigen::Vector3d::Zero();
    arm_.thc     = Eigen::Vector3d::Zero();
  }

  // ========== 파일 로드 ==========
  bool load_txt(const std::string& path) {
    std::ifstream infile(path);
    if (!infile.is_open()) return false;

    std::string line;
    size_t line_no = 0;
    poses_.clear();
    poses_.reserve(10000);

    while (std::getline(infile, line)) {
      ++line_no;
      if (line.empty()) continue;
      if (line[0] == '#') continue;

      std::istringstream iss(line);
      std::vector<double> cols{ std::istream_iterator<double>(iss), std::istream_iterator<double>() };
      if (cols.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "Line %zu has <6 columns, skipped.", line_no);
        continue;
      }

      PoseRPY p;
      p.line_no = line_no;
      p.x   = cols[0];
      p.y   = cols[1];
      p.z   = cols[2];
      p.r   = cols[3];
      p.p   = cols[4];
      p.yaw = cols[5];

      if (use_degrees_) {
        constexpr double D2R = M_PI/180.0;
        p.r   *= D2R;
        p.p   *= D2R;
        p.yaw *= D2R;
      }
      poses_.push_back(p);
    }
    return !poses_.empty();
  }

  // ========== 보간 시작 ==========
  void start_joint_trajectory(const Eigen::Matrix<double,6,1>& q_start,
                              const Eigen::Matrix<double,6,1>& q_goal,
                              double duration_sec)
  {
    traj_q_start_ = q_start;
    traj_q_goal_  = q_goal;
    traj_step_i_ = 0;
    traj_steps_total_ = std::max(1, (int)std::round(duration_sec * hz_));
  }

  // ========== 타이머 콜백 ==========
  void on_timer() {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - start_time_).count();
    double delta_ms   = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - last_time_).count();
    last_time_ = now;

    switch (phase_) {
      case Phase::MOVE_HOME: {
        // 선형 보간
        double t = (double)(traj_step_i_ + 1) / (double)traj_steps_total_;
        if (t > 1.0) t = 1.0;
        current_cmd_ = (1.0 - t) * traj_q_start_ + t * traj_q_goal_;

        publish_q(current_cmd_);

        if (++traj_step_i_ >= traj_steps_total_) {
          // 완료 → 3초 정지
          hold_steps_left_ = std::max(1, (int)std::round(home_hold_s_ * hz_));
          phase_ = Phase::HOLD_HOME;
          RCLCPP_INFO(this->get_logger(), "Reached HOME. Hold for %.2f s.", home_hold_s_);
        }
        break;
      }

      case Phase::HOLD_HOME: {
        publish_q(current_cmd_);
        if (--hold_steps_left_ <= 0) {
          // 홈에서 첫 포즈로 이동 준비
          Eigen::Matrix<double,6,1> q_first;
          if (!compute_ik_for_tcp_pose(poses_[0], q_first)) {
            RCLCPP_WARN(this->get_logger(), "IK failed for first pose. Staying at home.");
            phase_ = Phase::RUN_PATH; // 그래도 진행
            break;
          }
          start_joint_trajectory(current_cmd_, q_first, move_to_first_s_);
          phase_ = Phase::MOVE_TO_FIRST;
          RCLCPP_INFO(this->get_logger(), "Moving HOME -> FIRST over %.2f s.", move_to_first_s_);
        }
        break;
      }

      case Phase::MOVE_TO_FIRST: {
        double t = (double)(traj_step_i_ + 1) / (double)traj_steps_total_;
        if (t > 1.0) t = 1.0;
        current_cmd_ = (1.0 - t) * traj_q_start_ + t * traj_q_goal_;
        publish_q(current_cmd_);

        if (++traj_step_i_ >= traj_steps_total_) {
          // 첫 포즈 도달 → 텍스트 궤적 재생(현재 idx_는 0부터)
          phase_ = Phase::RUN_PATH;
          RCLCPP_INFO(this->get_logger(), "Reached FIRST pose. Start running TXT path.");
        }
        break;
      }

      case Phase::RUN_PATH: {
        if (idx_ >= poses_.size()) {
          // 끝 → 홈으로 복귀
          start_joint_trajectory(current_cmd_, home_q_, return_home_s_);
          phase_ = Phase::RETURN_HOME;
          RCLCPP_INFO(this->get_logger(), "TXT done. Returning HOME over %.2f s.", return_home_s_);
          break;
        }

        const auto& P = poses_[idx_++];

        // 진행 중 디버그: 현재 줄 번호 + x y z r p y
        RCLCPP_INFO(this->get_logger(),
          "[line %zu] x=%.6f y=%.6f z=%.6f r=%.6f p=%.6f y=%.6f",
          P.line_no, P.x, P.y, P.z, P.r, P.p, P.yaw);

        Eigen::Matrix<double,6,1> q_sol;
        if (!compute_ik_for_tcp_pose(P, q_sol)) {
          RCLCPP_WARN(this->get_logger(), "[line %zu] IK failed. Keep last q.", P.line_no);
          // 실패 시: current_cmd_ 유지
        } else {
          current_cmd_ = q_sol;
        }
        publish_q(current_cmd_);
        break;
      }

      case Phase::RETURN_HOME: {
        double t = (double)(traj_step_i_ + 1) / (double)traj_steps_total_;
        if (t > 1.0) t = 1.0;
        current_cmd_ = (1.0 - t) * traj_q_start_ + t * traj_q_goal_;
        publish_q(current_cmd_);

        if (++traj_step_i_ >= traj_steps_total_) {
          phase_ = Phase::DONE;
          RCLCPP_INFO(this->get_logger(), "All done. At HOME.");
        }
        break;
      }

      case Phase::DONE: {
        publish_q(current_cmd_);
        // 타이머 유지(정지 원하면 cancel())
        break;
      }
    }

    (void)elapsed_ms; (void)delta_ms; // 필요시 사용
  }

  // ========== TCP 포즈 → IK ==========
  bool compute_ik_for_tcp_pose(const PoseRPY& P, Eigen::Matrix<double,6,1>& q_out) {
    // 1) RPY -> R
    Vector3d th; th << P.r, P.p, P.yaw;
    Matrix3d R;
    kin_.EulerAngle2Rotation(R, th);

    // 2) TCP 목표 Td_tcp
    Matrix4d Td_tcp = Matrix4d::Identity();
    Td_tcp.block<3,3>(0,0) = R;
    Td_tcp(0,3) = P.x;
    Td_tcp(1,3) = P.y;
    Td_tcp(2,3) = P.z;

    // 3) EE→TCP (EE 프레임 +Z로 tool_z_)
    Matrix4d EE2TCP = Matrix4d::Identity();
    EE2TCP(2,3) = tool_z_;

    // 4) IK는 EE 기준 → Td_ee
    arm_.Td = Td_tcp * EE2TCP.inverse();

    // 5) 근접해 선택
    arm_.qc = current_cmd_; // 현재 명령값 근처 해
    int nsol = kin_.InverseK_min(&arm_);
    if (nsol <= 0) return false;

    for (int i = 0; i < 6; ++i) q_out(i) = arm_.qd(i);
    return true;
  }

  // ========== 퍼블리시 ==========
  void publish_q(const Eigen::Matrix<double,6,1>& q) {
    joint_state_.position[0] = q(0);
    joint_state_.position[1] = q(1);
    joint_state_.position[2] = q(2);
    joint_state_.position[3] = q(3);
    joint_state_.position[4] = q(4);
    joint_state_.position[5] = q(5);
    joint_state_.header.stamp = this->now();
    pub_->publish(joint_state_);
  }
};

// ============== main ==============
int main(int argc, char* argv[]) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "mlockall failed");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Memory locked.");
  }

  struct sched_param sp;
  sp.sched_priority = 80;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to set real-time scheduling");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TxtIKExecutor>());
  rclcpp::shutdown();
  return 0;
}
