#ifndef FUNC_H
#define FUNC_H

/*---------------------------- Header files ---------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <time.h>
#include <sensor_msgs/msg/joint_state.hpp>

/* Custom library headers */
#include "Kinematics.h"
#include "Armtraj.h"
#include "ArmGuide.h"
#include "Yoon_path.h"
#include "Yoon_force_control.h"
#include "Yoon_filters.h"
#include "Yoon_UR10e_cmd.h"
#include "nrs_msgmonitoring2/msg_monitoring.hpp"

/* Custom Package headers */
#include "nrs_forcecon2/nrs_3step_faac.h"

/* MSG headers */
//// #include "rtde_handarm2/msg/ArmMsg.hpp"
//// #include "rtde_handarm2/msg/FTsensorMsg.msg"

#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//// #include "rtde_handarm/msg/vr_pos_rt_msg_rpy.hpp"
//// #include "rtde_handarm/msg/vr_pos_rt_msg_qua.hpp"
#include <tf2_ros/transform_broadcaster.h>

/* Signal handler headers */
#include <errno.h>
#include <sys/mman.h>
#include <signal.h>
/*---------------------------- URrtde---------------------------------*/
//// #include <ur_rtde/rtde_control_interface.h>
//// #include <ur_rtde/rtde_receive_interface.h>
//// #include <ur_rtde/rtde_io_interface.h>
#include <thread>
#include <csignal>
#include <cmath>
/*---------------------------- Yaml file headers ---------------------------------*/
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "NRS_yaml_location.h"

/* ---------------------------- Macro setting ---------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define pi 3.14159265358979323846

/**** Modes selection ****/
#define Actual_mode 1       // 0: Test mode, 1: Actual_mode

#define Adm_mode 1          // 0: defualt mode, 1: tustin mode, 2: time integral
#define TCP_standard 1      // 0: End-effector based, 1: TCP based
#define Hand_spring_mode 0  // 0: guiding mode, 1: Spring mode (절대 "1" 금지 !!)
#define Playback_mode 1     // 0: Position playback mode, 1: Power(Force + Position) playback mode
#define RT_printing 1       // 0: RT_printing off, 1: RT_printing on

/*---------------------------- Namespace setting & instance generation ---------------------------------*/
//// using namespace ur_rtde;
using namespace std::chrono;
using namespace std;
using nrs_msgmonitoring2::MsgMonitoring; // Add on 2025.06.10 17:23

extern AKfun AKin;
extern CArm RArm;
extern Armtraj A_Traj;
extern ArmGuide A_Guide;

extern FILE *fp_ur_record, *fp_servo, *fp_record, *fp_replay;

/*---------------------------- Yaml file load ---------------------------------*/
extern std::ifstream fin1;
extern std::ifstream fin2;
extern std::ifstream fin3;
extern std::ifstream fin4;
extern YAML::Node NRS_IP;
extern YAML::Node NRS_recording;
extern YAML::Node NRS_Fcon_setting;
extern YAML::Node NRS_VR_setting;

/*---------------------------- System setting ---------------------------------*/











// using namespace std;
// using namespace Eigen;

// // variable declaration
// extern array<float, 6> body_pose;
// extern array<float, 12> joint_pos;
// extern array<float, 12> joint_vel;
// extern array<float, 3> body_pos;
// extern array<float, 3> body_vel;
// extern array<float, 9> imu;
// extern array<float, 4> contact;
// extern array<float, 3> command;
// extern array<float, 4> dist;
// extern array<float, 21> link_force;

// // Function set for FootstepTrajectroy generation
// void solve(double d, double e, double f, double T, double singularity, double B_val[], double arr[6]);
// double CalculateXValues(double l, double v, double t);
// double CalculateValues(double S[], double t, double T, int cases);
// void SplineTrajectory(double t, double T, double vel_of_body, double &xVal, double &zVal);

// // Function set for calculating InverseKineamatics
// double InverseKinematics2D(double xVal, double zVal, int cases);
// void InverseKinematics3D(double px, double py, double pz, double d1, double l2, double l3, double* th);

// // Function set for calculating torque of each joint
// double PDControl(double Kp, double Kd, double target_pos, double current_pos, double current_vel);
// double FFControl(double Kp, double Kd, double th[3], int case_, int cri);
// // double runMPC(double th[3]);

// // Function set for delivering the torque to control node
// void CalculateTorqueStanding(double* output_torque, const std::array<double, 3>& Kp, const std::array<double, 3>& Kd, double t);
// void CalculateTorqueRunning(double* output_torque, const double* target_pos, const std::array<double, 3>& Kp, const std::array<double, 3>& Kd);




#endif // FUNC_H
