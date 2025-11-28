// force_control_ur10e_main.cpp
// - UR10e EE/TCP 기준 Admittance + FAAC 기반 힘제어 체인
// - M/D/K 파라미터는 yaml/ur10e_spindle_parameter.yaml 의 force_control 섹션에서 로드
//
//   yaml 예시:
//   force_control:
//     mass:      [1.0, 1.0, 1.0, 0.05, 0.05, 0.05]
//     damping:   [6000.0, 6000.0, 6000.0, 10.0, 10.0, 10.0]
//     stiffness: [2000.0, 2000.0, 2000.0, 20.0, 20.0, 20.0]

#include "func_ur10e_main.h"
#include "JointControl.h"

#include <algorithm>   // std::clamp
#include <vector>
#include <memory>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ============================================================================
//  YAML 로부터 force_control.{mass,damping,stiffness} 로드
// ============================================================================
namespace {

void load_force_control_params(
    rclcpp::Logger logger,
    double FC_MASS[6],
    double FC_DAMPER[6],
    double FC_STIFFNESS[6])
{
    // 1) 기본값 (기존 하드코딩 값)
    double default_mass[6]      = {1.0,   1.0,   1.0,   0.05, 0.05, 0.05};
    double default_damper[6]    = {6000., 6000., 6000., 10.0, 10.0, 10.0};
    double default_stiffness[6] = {2000., 2000., 2000., 20.0, 20.0, 20.0};

    for (int i = 0; i < 6; ++i) {
        FC_MASS[i]      = default_mass[i];
        FC_DAMPER[i]    = default_damper[i];
        FC_STIFFNESS[i] = default_stiffness[i];
    }

    try {
        // 패키지 share 디렉토리 기준 경로:
        //   share/rtde_handarm2/yaml/ur10e_spindle_parameter.yaml
        std::string pkg_share =
            ament_index_cpp::get_package_share_directory("rtde_handarm2");
        std::string yaml_path = pkg_share + "/yaml/ur10e_spindle_parameter.yaml";

        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root["force_control"]) {
            RCLCPP_WARN(logger,
                "[force_control] YAML node not found in %s, using defaults.",
                yaml_path.c_str());
            return;
        }
        YAML::Node fc_node = root["force_control"];

        auto load_vec = [&](const char* key, double out[6]){
            if (!fc_node[key]) {
                RCLCPP_WARN(logger,
                    "[force_control.%s] not found in %s, keep defaults.",
                    key, yaml_path.c_str());
                return;
            }
            auto node = fc_node[key];
            if (!node.IsSequence() || node.size() != 6) {
                RCLCPP_WARN(logger,
                    "[force_control.%s] must be length-6 sequence in %s, keep defaults.",
                    key, yaml_path.c_str());
                return;
            }
            for (std::size_t i = 0; i < 6; ++i) {
                out[i] = node[i].as<double>();
            }
        };

        load_vec("mass",      FC_MASS);
        load_vec("damping",   FC_DAMPER);
        load_vec("stiffness", FC_STIFFNESS);

        RCLCPP_INFO(logger,
            "Loaded force_control params from %s", yaml_path.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger,
            "Failed to load force_control params from YAML: %s. Using defaults.",
            e.what());
    }
}

} // namespace

// ============================================================================
//  JointControl::runCartesianForceChain
// ============================================================================
void JointControl::runCartesianForceChain(
    const Eigen::Vector3d& Xd,
    const Eigen::Vector3d& RPYd,
    const Eigen::Vector3d& Fd,
    double dt_s)
{
    // =========================================================================
    // STEP 2) 외력 추정 F_ext (TCP z-force → Base frame 변환만 사용)
    // =========================================================================
    Eigen::Matrix4d T_base_TCP_cur = RArm.Tc; // 현재 EE/TCP pose
    Eigen::Matrix3d R_base_TCP     = T_base_TCP_cur.block<3,3>(0,0);
    Eigen::Matrix3d R_TCP_base     = R_base_TCP.transpose();

    // contact_force 는 TCP z 로 들어온 값이라고 가정
    // (부호는 기존 코드처럼 -contact_force 유지)
    Eigen::Vector3d F_TCP(0.0, 0.0, -contact_force);
    Eigen::Vector3d F_base = R_TCP_base * F_TCP;
    Eigen::Vector3d F_ext  = F_base;   // ⬅ 필터/램프 없이 바로 사용

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
    // STEP 3) RPYd → 회전행렬 → axis-angle (Wd)
    // =========================================================================
    auto rotFromRPY = [](const Eigen::Vector3d &rpy)->Eigen::Matrix3d {
        const double cr = std::cos(rpy(0));
        const double sr = std::sin(rpy(0));
        const double cp = std::cos(rpy(1));
        const double sp = std::sin(rpy(1));
        const double cy = std::cos(rpy(2));
        const double sy = std::sin(rpy(2));  // ★ BUGFIX: sin 사용

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
    // STEP 4) 어드미턴스 + FAAC (Fd, F_ext 직접 사용)
    // =========================================================================
    static bool fc_init         = false;
    static bool fc_param_loaded = false;

    static Yadmittance_control AControl[6] = {
        Yadmittance_control(0.001), Yadmittance_control(0.001),
        Yadmittance_control(0.001), Yadmittance_control(0.001),
        Yadmittance_control(0.001), Yadmittance_control(0.001)
    };
    static std::unique_ptr<Nrs3StepFAAC> FAAC3step[3];
    static bool   FAAC_flag[3] = {false,false,false};
    static double AC_pose_pos[3] = {0.0,0.0,0.0};
    static double AC_pose_ori[3] = {0.0,0.0,0.0};

    static double FC_MASS[6];
    static double FC_DAMPER[6];
    static double FC_STIFFNESS[6];

    // --- 4-0) YAML에서 M/D/K 한 번만 로드 ---
    if (!fc_param_loaded) {
        load_force_control_params(node_->get_logger(),
                                  FC_MASS, FC_DAMPER, FC_STIFFNESS);
        fc_param_loaded = true;
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
        std::vector<double> proc_noise = {0.1, 0.1, 0.1};
        std::vector<double> meas_noise = {10.0, 10.0, 10.0};
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
    // bool contact_on = (Xd(2) <= 0.1);
    bool contact_on = (Xd(2) <= 100);

    for (int ax = 0; ax < 3; ++ax) {
        if (std::fabs(Fd(ax)) > 0.01 || FAAC_flag[ax])
            FAAC_flag[ax] = true;

        if (FAAC_flag[ax] && FAAC3step[ax]) {
            auto faac_mdk = FAAC3step[ax]->FAAC_MDKob_RUN(
                Tank_energy,
                F_ext(ax),
                Fd(ax),          // ⬅ 원하는 힘 Fd
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

        // 순수 어드미턴스: (Xd, Fd, F_ext) 로부터 next_pos 계산
        double next_pos = AControl[ax].adm_1D_control(
            Xd(ax),
            Fd(ax),
            F_ext(ax)
        );

        Xc_cmd(ax) = next_pos;
    }

    // 위치 안정화 (offset/step 제한은 그대로 두되, 순수 위치 제한 역할만 함)
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
        dbg.data[3]  = Fd(0);
        dbg.data[4]  = Fd(1);
        dbg.data[5]  = Fd(2);
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
    // STEP 7) 외력 publish (Base frame 기준 F_ext)
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
