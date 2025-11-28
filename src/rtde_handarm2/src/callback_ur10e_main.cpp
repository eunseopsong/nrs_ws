// callback_ur10e_main.cpp
// - JointControl ROS2 콜백 함수들만 분리 구현
// - JointControl.h 에 선언된 멤버 함수들의 정의만 따로 두는 파일

#include "JointControl.h"
#include "func_ur10e_main.h"   // set_status, yaml_get_path, etc.

#include <cstdio>
#include <cstring>
#include <iostream>
#include <filesystem>
#include <limits>

// 필요하면 여기서도 사용
// #include <geometry_msgs/msg/wrench.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>
// #include <std_msgs/msg/float64.hpp>
// #include <std_msgs/msg/uint16.hpp>

constexpr int DOF = 6;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// 모드 변경 보호용 (이제 이 파일 안에서만 사용)
static std::mutex g_cmdmode_mtx;

// ============================================================================
// 콜백들 구현부
// ============================================================================

void JointControl::cmdModeCallback(const std_msgs::msg::UInt16::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(g_cmdmode_mtx);
  try {
    mode_cmd = msg->data;
    printf("[DEBUG] cmdModeCallback called. mode_cmd=%u\n", mode_cmd);


    // ===================== FK Control Mode wo/ Force Control ===================== //
    if (mode_cmd == FK_control_mode_cmd) {
      ctrl.store(1, std::memory_order_release);
    }

    // ===================== IK Control Mode wo/ Force Control ===================== //
    else if (mode_cmd == IK_control_mode_cmd) {
      ctrl.store(2, std::memory_order_release);
    }

    // ===================== Playback Mode for Path(.txt) Execution ===================== //
    else if (mode_cmd == Playback_mode_cmd) {
      auto hand_path = yaml_get_path(NRS_recording, "hand_g_recording", node_->get_logger());
      if (hand_path.empty() || !std::filesystem::exists(hand_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory file not found: '%s'", hand_path.c_str());
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, Motion_stop_mode);
        return;
      }
      if (Hand_G_playback) { std::fclose(Hand_G_playback); Hand_G_playback = nullptr; }
      Hand_G_playback = std::fopen(hand_path.c_str(), "rt");
      if (!Hand_G_playback) {
        RCLCPP_ERROR(node_->get_logger(), "open for read failed: '%s' (%s)",
                     hand_path.c_str(), std::strerror(errno));
        ctrl.store(0, std::memory_order_release);
        set_status(message_status, Motion_stop_mode);
        return;
      }
      set_status(message_status, ST_path_gen_done);
      ctrl.store(3, std::memory_order_release);
      pre_ctrl.store(0, std::memory_order_relaxed); // 다음 사이클에서 init 감지되도록
    }

    // ======================== VR Teleop Mode w/ Force Control ======================== //
    else if (mode_cmd == VR_teleop_start) {
      ctrl.store(4, std::memory_order_release);
      set_status(message_status, Data_recording_on);
    }
    else if (mode_cmd == VR_teleop_end) {
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, Data_recording_off);
    }

    // ===================== Keyboard Teleop Mode w/ Force Control ===================== //
    else if (mode_cmd == Keyboard_teleop_start) {
      ctrl.store(5, std::memory_order_release);
    }

    // =============================== Stop Mode =============================== //
    else if (mode_cmd == Motion_stop_cmd) {
      ctrl.store(0, std::memory_order_release);
      set_status(message_status, Motion_stop_mode);
      if (Hand_G_playback)     { std::fclose(Hand_G_playback);     Hand_G_playback     = nullptr; }
      if (hand_g_recording)    { std::fclose(hand_g_recording);    hand_g_recording    = nullptr; }
      if (path_recording_pos)  { std::fclose(path_recording_pos);  path_recording_pos  = nullptr; }
    }

  } catch (const std::exception& e) {
    RCLCPP_FATAL(node_->get_logger(), "cmdModeCallback exception: %s", e.what());
  }
}

void JointControl::PbIterCallback(std_msgs::msg::UInt16::SharedPtr msg) {
  PB_iter_cmd = msg->data;
  PB_iter_cur = 1; // 1 is right
}

void JointControl::JointCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  mjoint_cmd = msg->data;
  printf("\nSelected joint: %1.0f, Target relative joint angle: %4f \n",
         mjoint_cmd[0], mjoint_cmd[1]);

  double Tar_pos[]      = { std::fabs(mjoint_cmd[1]) };
  double Tar_vel[]      = { (mjoint_cmd[1] >= 0) ? 0.1 : -0.1 };
  double Waiting_time[] = {0,0}; // s

  Joint_path_start << RArm.qc(0),RArm.qc(1),RArm.qc(2),
                      RArm.qc(3),RArm.qc(4),RArm.qc(5);

  Path_point_num =
      J_single.Single_blended_path(Tar_pos, Tar_vel, Waiting_time,
                                   (int)(sizeof(Tar_pos)/sizeof(*Tar_pos)));

  if (Path_point_num != -1) {
    ctrl.store(1, std::memory_order_release);
    set_status(message_status, path_gen_done);
    path_done_flag = true;
  }
}

void JointControl::getActualQ(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (int i = 0; i < DOF && i < (int)msg->position.size(); ++i){
    RArm.qc[i] = msg->position[i];
  }
}

void JointControl::FtCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  contact_force = msg->data;

  // TCP 기준 힘(스칼라만 올 때의 가정: [0,0,Fz])
  const Eigen::Vector3d F_TCP(0.0, 0.0, contact_force);

  // RArm.Tc : Base->TCP 변환(회전)으로 가정
  const Eigen::Matrix3d R_base_TCP = RArm.Tc.block<3,3>(0,0);
  const Eigen::Vector3d F_base     = R_base_TCP * F_TCP;
  (void)F_base; // 실제 제어는 runCartesianForceChain 내부에서 처리
}

void JointControl::calibratedPoseCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 6) return;

  teleop_xyz_ <<
      msg->data[0],
      msg->data[1],
      msg->data[2];

  teleop_rpy_ <<
      wrapToPi(msg->data[3]),
      wrapToPi(msg->data[4]),
      wrapToPi(msg->data[5]);

  teleop_pose_valid_ = true;
}

void JointControl::ftSensorCallback(
    const geometry_msgs::msg::Wrench::SharedPtr msg)
{
  teleop_force_ <<
      msg->force.x,
      msg->force.y,
      msg->force.z;
  teleop_force_valid_ = true;
}
