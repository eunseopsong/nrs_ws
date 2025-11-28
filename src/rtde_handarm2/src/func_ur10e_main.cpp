// func_ur10e_main.cpp
//
// JointControl.cpp 에서 빠져나온 공용 유틸 + JointControl 멤버 함수 구현부 모음:
//  - trim_path
//  - yaml_get_path
//  - ensure_parent_dir
//  - readDoublesFromStdin
//  - JointControl::InitMove
//  - JointControl::runCartesianForceChain
//  - JointControl::PathFollow
//  - JointControl::ReturnHomePose
//
// 주의:
//  - yaml_get_path / readDoublesFromStdin 은 전역 함수 (static 아님)
//  - JointControl 멤버 함수는 반드시 JointControl:: 스코프로 구현

#include "func_ur10e_main.h"
#include "JointControl.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>

#include <geometry_msgs/msg/wrench.hpp>

constexpr int DOF = 6;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// -----------------------------------------------------------------------------
// 문자열 앞뒤 공백/개행 제거 (전역 함수, static 금지)
// -----------------------------------------------------------------------------
std::string trim_path(std::string s) {
  auto notspace = [](unsigned char c){ return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
  while (!s.empty() &&
         (s.back() == '\r' || s.back() == '\n' ||
          std::isspace(static_cast<unsigned char>(s.back())))) {
    s.pop_back();
  }
  return s;
}

// -----------------------------------------------------------------------------
// YAML에서 파일 경로 안전 취득 (전역 함수, static 금지)
//  - 헤더 func_ur10e_main.h 와 시그니처 100% 일치해야 함
// -----------------------------------------------------------------------------
std::string yaml_get_path(const YAML::Node& root,
                          const char* key,
                          const rclcpp::Logger& logger)
{
  try {
    if (!root || !root.IsMap()) {
      RCLCPP_ERROR(logger, "NRS_recording is not a map (key='%s')", key);
      return "";
    }
    YAML::Node n = root[key];
    if (!n || !n.IsDefined()) {
      RCLCPP_ERROR(logger, "YAML key '%s' is missing/undefined.", key);
      return "";
    }

    if (n.IsScalar()) {
      return trim_path(n.as<std::string>());
    }

    if (n.IsSequence()) {
      std::filesystem::path p;
      for (std::size_t i = 0; i < n.size(); ++i) {
        if (!n[i].IsScalar()) {
          RCLCPP_ERROR(logger,
                       "YAML key '%s' has non-scalar element in sequence.",
                       key);
          return "";
        }
        p /= n[i].as<std::string>();
      }
      return trim_path(p.string());
    }

    RCLCPP_ERROR(logger, "YAML key '%s' must be a scalar or sequence.", key);
    return "";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "YAML get path error for key '%s': %s", key, e.what());
    return "";
  }
}

// -----------------------------------------------------------------------------
// 파일 부모 디렉토리 생성 보장 (전역 함수)
// -----------------------------------------------------------------------------
void ensure_parent_dir(const std::string& filepath,
                       const rclcpp::Logger& logger)
{
  if (filepath.empty()) return;

  std::error_code ec;
  auto parent = std::filesystem::path(filepath).parent_path();
  if (!parent.empty() && !std::filesystem::exists(parent)) {
    if (!std::filesystem::create_directories(parent, ec)) {
      if (ec) {
        RCLCPP_WARN(logger, "Failed to create parent dir '%s': %s",
                    parent.string().c_str(), ec.message().c_str());
      }
    }
  }
}

// -----------------------------------------------------------------------------
// 터미널에서 double n개 입력 받기 (전역 함수, static 금지)
//  - JointControl.cpp 에서 free 함수로 사용
// -----------------------------------------------------------------------------
bool readDoublesFromStdin(const char* prompt, int n, double* out)
{
  std::cout << prompt << std::flush;
  for (int i = 0; i < n; ++i) {
    if (!(std::cin >> out[i])) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      return false;
    }
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return true;
}

// ============================================================================
// JointControl 멤버 함수 구현부
//  - JointControl.h 에 선언만 있고, 구현은 여기로 분리
// ============================================================================

// ===================== InitMove() =====================
// 역할: 재생 시작 시 현재 위치→TXT 첫 포즈로 시간기반 보간 이동(SLERP+선형)
bool JointControl::InitMove(double dt_s)
{
    static constexpr double LIN_VEL   = 0.20;  // m/s
    static constexpr double ANG_VEL   = 1.0;   // rad/s
    static constexpr double MIN_DUR   = 0.8;   // s
    static constexpr double MAX_DUR   = 4.0;   // s

    static bool active = false, finished = false;
    static double elapsed = 0.0, duration = 0.0;
    static Eigen::Vector3d start_xyz, goal_xyz;
    static Eigen::Matrix3d start_rot, goal_rot;
    static FILE* last_handle = nullptr;

    // 재생 파일 핸들이 바뀌면 상태 리셋
    if (Hand_G_playback != last_handle) {
        active = false; finished = false; elapsed = 0.0; duration = 0.0;
        last_handle = Hand_G_playback;
    }

    // 초기화 단계
    if (!active && !finished) {
        if (!Hand_G_playback) return false;

        float x, y, z, r, p, yw, fx, fy, fz;
        char buf[2048];
        bool valid = false;

        std::rewind(Hand_G_playback);
        while (std::fgets(buf, sizeof(buf), Hand_G_playback)) {
            if (buf[0] == '#') continue;
            int n = std::sscanf(buf, "%f %f %f %f %f %f %f %f %f",
                                &x,&y,&z,&r,&p,&yw,&fx,&fy,&fz);
            if (n == 9) { valid = true; break; }
        }
        if (!valid) {
            RCLCPP_ERROR(node_->get_logger(), "[InitMove] no valid first line in TXT.");
            return false;
        }

        // 현재 위치(EE/TCP 기준)
        start_xyz = RArm.xc;
        // TXT 파일에서 읽은 값도 EE/TCP 기준
        goal_xyz  = Eigen::Vector3d(x, y, z);

        start_rot = RArm.Tc.block<3,3>(0,0);
        Eigen::Vector3d goal_rpy(r,p,yw);
        AKin.EulerAngle2Rotation(goal_rot, goal_rpy);

        const double lin_dist = (goal_xyz - start_xyz).norm();
        const Eigen::Quaterniond q0(start_rot), q1(goal_rot);
        const double ang_dist = std::acos(std::clamp(
                                   q0.normalized().dot(q1.normalized()),
                                   -1.0, 1.0)) * 2.0;

        double t_lin = (LIN_VEL > 1e-6) ? lin_dist / LIN_VEL : 0.0;
        double t_ang = (ANG_VEL > 1e-6) ? ang_dist / ANG_VEL : 0.0;

        duration = std::max(MIN_DUR,
                            std::min(MAX_DUR, std::max(t_lin, t_ang)));

        elapsed = 0.0;
        active = true;
        finished = false;

        printf("[InitMove] dist=%.3f ang=%.3f rad -> dur=%.2fs\n",
               lin_dist, ang_dist, duration);
    }

    if (!active) return finished;

    elapsed += dt_s;
    const double alpha =
        std::clamp(elapsed / std::max(1e-6, duration), 0.0, 1.0);

    // EE/TCP 기준에서 보간
    const Eigen::Vector3d xyz_interp =
        (1.0 - alpha) * start_xyz + alpha * goal_xyz;
    Desired_XYZ = xyz_interp;   // Desired (TCP)

    Eigen::Quaterniond q0(start_rot), q1(goal_rot);
    if (q0.dot(q1) < 0.0) q1.coeffs() *= -1.0;
    const Eigen::Quaterniond q_interp = q0.slerp(alpha, q1).normalized();
    const Eigen::Matrix3d R_interp    = q_interp.toRotationMatrix();
    Desired_RPY = R_interp.eulerAngles(0,1,2);

    // IK용 Td (EE/TCP 기준 직접 사용)
    RArm.Td <<
        R_interp(0,0),R_interp(0,1),R_interp(0,2),Desired_XYZ(0),
        R_interp(1,0),R_interp(1,1),R_interp(1,2),Desired_XYZ(1),
        R_interp(2,0),R_interp(2,1),R_interp(2,2),Desired_XYZ(2),
        0,0,0,1;
#if TCP_standard == 0
    AKin.InverseK_min(&RArm);
#else
    AKin.Ycontact_InverseK_min(&RArm);
#endif

    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < DOF; ++i) joint_state_.position[i] = RArm.qd(i);
    joint_commands_pub_->publish(joint_state_);

    if (alpha >= 1.0 - 1e-6) {
        active   = false;
        finished = true;
        std::rewind(Hand_G_playback);
        printf("[InitMove] completed.\n");
    }
    return finished;
}

// ============================================================================
// PathFollow – TXT → (Xd,RPYd,Fd)만 만들고 공통 체인 호출
// ============================================================================
bool JointControl::PathFollow(double dt_s)
{
    static bool  active      = true;
    static FILE* last_handle = nullptr;
    if (Hand_G_playback != last_handle) {
        active      = true;
        last_handle = Hand_G_playback;
    }
    if (!active) {
        return false;
    }

    if (!Hand_G_playback) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[PB] playback file closed unexpectedly.");
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, "Playback file closed");
        return false;
    }

    float des_x, des_y, des_z;
    float des_r, des_p, des_yaw;
    float des_fx, des_fy, des_fz;

    int reti = std::fscanf(
        Hand_G_playback,
        "%f %f %f %f %f %f %f %f %f",
        &des_x, &des_y, &des_z,
        &des_r, &des_p, &des_yaw,
        &des_fx, &des_fy, &des_fz
    );

    if (reti != 9) {
        std::fclose(Hand_G_playback);
        Hand_G_playback = nullptr;
        active = false;

        // Return-to-home 초기화
        return_active_   = true;
        return_elapsed_  = 0.0;
        return_duration_ = 4.0;
        for (int i = 0; i < DOF; ++i) {
            return_start_q_(i) = RArm.qc(i);
        }

        printf("[PB] End of file. Start return-to-home.\n");
        return false;
    }

    Eigen::Vector3d Xd(des_x, des_y, des_z);
    Eigen::Vector3d RPYd(des_r, des_p, des_yaw);
    Eigen::Vector3d Fd(des_fx, des_fy, des_fz);

    Desired_XYZ = Xd;
    Desired_RPY = RPYd;

    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(9);
        dbg.data[0] = Xd(0);
        dbg.data[1] = Xd(1);
        dbg.data[2] = Xd(2);
        dbg.data[3] = RPYd(0);
        dbg.data[4] = RPYd(1);
        dbg.data[5] = RPYd(2);
        dbg.data[6] = Fd(0);
        dbg.data[7] = Fd(1);
        dbg.data[8] = Fd(2);
        debug_step1_pub_->publish(dbg);
    }

    runCartesianForceChain(Xd, RPYd, Fd, dt_s);
    return true;
}

// ============================================================================
// ReturnHomePose – PathFollow 끝나고 홈자세로 부드럽게 복귀
// ============================================================================
bool JointControl::ReturnHomePose(double dt_s)
{
    static bool     active    = false;
    static double   elapsed   = 0.0;
    static double   duration  = 10.0;
    static Vector6d start_q;

    static const Vector6d HOME_Q = (Vector6d() <<
        0.0,
        -M_PI / 2.0,
        -M_PI / 2.0,
        -M_PI / 2.0,
        +M_PI / 2.0,
        0.0).finished();

    if (return_active_ && !active) {
        active    = true;
        elapsed   = 0.0;
        duration  = 10.0;
        start_q   = return_start_q_;
    }

    if (active) {
        double dt_step = dt_s;
        if (dt_step <= 0.0 || dt_step > 0.05) {
            dt_step = 0.001;
        }

        elapsed += dt_step;

        double s_raw = elapsed / std::max(1e-6, duration);
        if (s_raw < 0.0) s_raw = 0.0;
        if (s_raw > 1.0) s_raw = 1.0;

        double s = (3.0 * s_raw * s_raw) - (2.0 * s_raw * s_raw * s_raw);

        Vector6d q_cmd;
        for (int i = 0; i < DOF; ++i) {
            q_cmd(i) = (1.0 - s) * start_q(i) + s * HOME_Q(i);
        }

        for (int i = 0; i < DOF; ++i) {
            RArm.qd(i) = q_cmd(i);
        }

        joint_state_.header.stamp = node_->now();
        for (int i = 0; i < DOF; ++i) {
            joint_state_.position[i] = RArm.qd(i);
        }
        joint_commands_pub_->publish(joint_state_);

        if (s_raw >= 1.0 - 1e-6) {
            active          = false;
            return_active_  = false;

            printf("[PB] Return-to-home done (10s smooth ramp).\n");

            ctrl.store(0, std::memory_order_release);
            set_status(message_status, "Playback finished");
        }

        return true;
    }

    return false;
}
