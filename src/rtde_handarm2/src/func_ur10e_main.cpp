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

// ===================== runCartesianForceChain() =====================
void JointControl::runCartesianForceChain(
    const Eigen::Vector3d& Xd,
    const Eigen::Vector3d& RPYd,
    const Eigen::Vector3d& Fd,
    double dt_s)
{
    // =========================================================================
    // STEP 2) 외력 추정 F_ext (LPF + saturation)
    // =========================================================================
    Eigen::Matrix4d T_base_TCP_cur = RArm.Tc; // 현재 EE/TCP pose
    Eigen::Matrix3d R_base_TCP     = T_base_TCP_cur.block<3,3>(0,0);
    Eigen::Matrix3d R_TCP_base     = R_base_TCP.transpose();

    // contact_force 는 TCP z 로 들어온 값이라고 가정
    Eigen::Vector3d F_TCP(0.0, 0.0, -contact_force);
    Eigen::Vector3d F_base = R_TCP_base * F_TCP;
    Eigen::Vector3d F_ext  = F_base;

    // LPF
    {
        static Eigen::Vector3d F_lp = Eigen::Vector3d::Zero();
        static bool first_f = true;

        const double fc = 15.0;                                // Hz
        const double Ts = (dt_s > 0.0 ? dt_s : 0.001);
        const double alpha = (2.0 * M_PI * fc * Ts) /
                             (1.0 + 2.0 * M_PI * fc * Ts);

        if (first_f) {
            F_lp    = F_ext;
            first_f = false;
        } else {
            F_lp = F_lp + alpha * (F_ext - F_lp);
        }
        F_ext = F_lp;
    }

    // Saturation
    {
        const double FEXT_SAT = 30.0; // N
        for (int k = 0; k < 3; ++k) {
            if (F_ext(k) >  FEXT_SAT) F_ext(k) =  FEXT_SAT;
            if (F_ext(k) < -FEXT_SAT) F_ext(k) = -FEXT_SAT;
        }
    }

    // 현재 실제 EE 위치 (EE/TCP 기준)
    Eigen::Vector3d X_act = RArm.xc;

    // debug step2
    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(7);
        dbg.data[0] = contact_force;
        dbg.data[1] = F_ext(0);
        dbg.data[2] = F_ext(1);
        dbg.data[3] = F_ext(2);
        dbg.data[4] = X_act(0);
        dbg.data[5] = X_act(1);
        dbg.data[6] = X_act(2);
        debug_step2_pub_->publish(dbg);
    }

    // =========================================================================
    // STEP 3) RPYd → 회전행렬 → axis-angle
    // =========================================================================
    auto rotFromRPY = [](const Eigen::Vector3d &rpy)->Eigen::Matrix3d {
        const double cr = std::cos(rpy(0));
        const double sr = std::sin(rpy(0));
        const double cp = std::cos(rpy(1));
        const double sp = std::sin(rpy(1));
        const double cy = std::cos(rpy(2));
        const double sy = std::sin(rpy(2));

        Eigen::Matrix3d Rz;
        Rz << cy,-sy,0,
              sy, cy,0,
              0 , 0 ,1;
        Eigen::Matrix3d Ry;
        Ry << cp,0,sp,
              0 ,1,0 ,
             -sp,0,cp;
        Eigen::Matrix3d Rx;
        Rx << 1,0 ,0 ,
              0,cr,-sr,
              0,sr, cr;
        return Rz * Ry * Rx;
    };

    auto rotLog = [](const Eigen::Matrix3d &R)->Eigen::Vector3d {
        double cos_theta = (R.trace() - 1.0) * 0.5;
        cos_theta = std::clamp(cos_theta, -1.0, 1.0);
        double theta = std::acos(cos_theta);
        if (theta < 1e-9) {
            return Eigen::Vector3d::Zero();
        }
        Eigen::Vector3d omega;
        omega << R(2,1) - R(1,2),
                 R(0,2) - R(2,0),
                 R(1,0) - R(0,1);
        omega *= 0.5 / std::sin(theta);
        return theta * omega;
    };

    Eigen::Matrix3d Rd_R = rotFromRPY(RPYd);
    Eigen::Vector3d Wd   = rotLog(Rd_R);

    // debug step3
    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(7);
        dbg.data[0] = RPYd(0);
        dbg.data[1] = RPYd(1);
        dbg.data[2] = RPYd(2);
        dbg.data[3] = Wd(0);
        dbg.data[4] = Wd(1);
        dbg.data[5] = Wd(2);
        dbg.data[6] = Wd.norm();
        debug_step3_pub_->publish(dbg);
    }

    // =========================================================================
    // STEP 4) 어드미턴스 + FAAC
    // =========================================================================
    static bool fc_init = false;
    static Yadmittance_control AControl[6] = {
        Yadmittance_control(0.001), Yadmittance_control(0.001),
        Yadmittance_control(0.001), Yadmittance_control(0.001),
        Yadmittance_control(0.001), Yadmittance_control(0.001)
    };
    static std::unique_ptr<Nrs3StepFAAC> FAAC3step[3];
    static bool   FAAC_flag[3] = {false,false,false};
    static double AC_pose_pos[3] = {0.0,0.0,0.0};
    static double AC_pose_ori[3] = {0.0,0.0,0.0};
    static double FC_MASS[6]      = {1.0,   1.0,   1.0,   0.05, 0.05, 0.05};
    static double FC_DAMPER[6]    = {6000., 6000., 6000., 10.0, 10.0, 10.0};
    static double FC_STIFFNESS[6] = {2000., 2000., 2000., 20.0, 20.0, 20.0};
    static Eigen::Vector3d Fd_cmd = Eigen::Vector3d::Zero();

    // 원하는 힘을 부드럽게 램프
    {
        const double alpha_up   = 0.02;
        const double alpha_down = 0.20;
        for (int k = 0; k < 3; ++k) {
            double alpha =
                (std::fabs(Fd(k)) > std::fabs(Fd_cmd(k))) ?
                alpha_up : alpha_down;
            Fd_cmd(k) += alpha * (Fd(k) - Fd_cmd(k));
        }
        const double FDES_SAT = 30.0;
        for (int k=0; k<3; ++k) {
            if (Fd_cmd(k) >  FDES_SAT) Fd_cmd(k) =  FDES_SAT;
            if (Fd_cmd(k) < -FDES_SAT) Fd_cmd(k) = -FDES_SAT;
        }
    }

    if (!fc_init) {
        // admittance 초기화
        for (int i = 0; i < 6; ++i) {
            AControl[i].adm_1D_MDK(
                FC_MASS[i],
                FC_DAMPER[i],
                FC_STIFFNESS[i]
            );
        }
        // FAAC 초기화
        std::vector<double> proc_noise = {0.1,0.1,0.1};
        std::vector<double> meas_noise = {10.0,10.0,10.0};
        double dt_for_faac = (dt_s > 0.0 ? dt_s : 0.001);
        for (int ax = 0; ax < 3; ++ax) {
            FAAC3step[ax] = std::make_unique<Nrs3StepFAAC>(
                FC_MASS[ax],
                FC_DAMPER[ax],
                FC_STIFFNESS[ax],
                dt_for_faac,
                proc_noise,
                meas_noise
            );
            FAAC_flag[ax] = false;
        }
        // 초기 기준
        AC_pose_pos[0] = Xd(0);
        AC_pose_pos[1] = Xd(1);
        AC_pose_pos[2] = Xd(2);
        AC_pose_ori[0] = Wd(0);
        AC_pose_ori[1] = Wd(1);
        AC_pose_ori[2] = Wd(2);
        fc_init = true;
    }

    Eigen::Vector3d Xc_cmd = Xd;
    Eigen::Vector3d Wc_cmd = Wd;
    const double Tank_energy = 5.0;

    // 바닥부 근처면 접촉으로 보고 K=0
    bool contact_on = (Xd(2) <= 0.1);

    for (int ax = 0; ax < 3; ++ax) {
        if (std::fabs(Fd_cmd(ax)) > 0.01 || FAAC_flag[ax])
            FAAC_flag[ax] = true;

        if (FAAC_flag[ax] && FAAC3step[ax]) {
            auto faac_mdk = FAAC3step[ax]->FAAC_MDKob_RUN(
                Tank_energy,
                F_ext(ax),
                Fd_cmd(ax),
                AC_pose_pos[ax],
                X_act(ax)
            );

            double used_K = contact_on ? 0.0 : faac_mdk.Stiffness;

            AControl[ax].adm_1D_MDK(
                faac_mdk.Mass,
                faac_mdk.Damping,
                used_K
            );
        }

        double next_pos = AControl[ax].adm_1D_control(
            Xd(ax),
            Fd_cmd(ax),
            F_ext(ax)
        );

        Xc_cmd(ax) = next_pos;
    }

    // 위치 안정화
    {
        static Eigen::Vector3d Xc_prev = Xd;
        const double max_offset = 0.010;
        const double max_step_each[3] = {0.001, 0.001, 0.0003};

        for (int ax=0; ax<3; ++ax) {
            double lo = Xd(ax) - max_offset;
            double hi = Xd(ax) + max_offset;
            if (Xc_cmd(ax) < lo) Xc_cmd(ax) = lo;
            if (Xc_cmd(ax) > hi) Xc_cmd(ax) = hi;

            double d = Xc_cmd(ax) - Xc_prev(ax);
            double max_step = max_step_each[ax];
            d = std::clamp(d, -max_step, max_step);
            Xc_cmd(ax) = Xc_prev(ax) + d;
        }
        Xc_prev = Xc_cmd;

        AC_pose_pos[0] = Xc_cmd(0);
        AC_pose_pos[1] = Xc_cmd(1);
        AC_pose_pos[2] = Xc_cmd(2);
    }

    AC_pose_ori[0] = Wc_cmd(0);
    AC_pose_ori[1] = Wc_cmd(1);
    AC_pose_ori[2] = Wc_cmd(2);

    // debug step4
    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data.resize(13);
        dbg.data[0]  = F_ext(0);
        dbg.data[1]  = F_ext(1);
        dbg.data[2]  = F_ext(2);
        dbg.data[3]  = Fd_cmd(0);
        dbg.data[4]  = Fd_cmd(1);
        dbg.data[5]  = Fd_cmd(2);
        dbg.data[6]  = X_act(0);
        dbg.data[7]  = X_act(1);
        dbg.data[8]  = X_act(2);
        dbg.data[9]  = Xc_cmd(0);
        dbg.data[10] = Xc_cmd(1);
        dbg.data[11] = Xc_cmd(2);
        dbg.data[12] = static_cast<double>(contact_on);
        debug_step4_pub_->publish(dbg);
    }

    // =========================================================================
    // STEP 5) IK용 pose 만들기 (EE/TCP 기준)
    // =========================================================================
    Eigen::Vector3d rpy_copy = RPYd;   // <- 비-const 참조 요구 때문에 복사
    Eigen::Matrix3d Rd_R_again;
    AKin.EulerAngle2Rotation(Rd_R_again, rpy_copy);

    RArm.Td <<
        Rd_R_again(0,0), Rd_R_again(0,1), Rd_R_again(0,2), Xc_cmd(0),
        Rd_R_again(1,0), Rd_R_again(1,1), Rd_R_again(1,2), Xc_cmd(1),
        Rd_R_again(2,0), Rd_R_again(2,1), Rd_R_again(2,2), Xc_cmd(2),
        0,               0,               0,               1;

#if TCP_standard == 0
    AKin.InverseK_min(&RArm);
#else
    AKin.Ycontact_InverseK_min(&RArm);
#endif

    // =========================================================================
    // STEP 6) 조인트 publish
    // =========================================================================
    joint_state_.header.stamp = node_->now();
    for (int i = 0; i < 6; ++i) {
        joint_state_.position[i] = RArm.qd(i);
    }
    joint_commands_pub_->publish(joint_state_);

    // =========================================================================
    // STEP 7) 외력 publish
    // =========================================================================
    {
        std_msgs::msg::Float64MultiArray force_msg;
        force_msg.data.resize(3);
        force_msg.data[0] = F_ext(0);
        force_msg.data[1] = F_ext(1);
        force_msg.data[2] = F_ext(2);
        force_ext_base_pub_->publish(force_msg);
    }
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
