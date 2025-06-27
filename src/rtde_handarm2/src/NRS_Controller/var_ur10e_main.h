#ifndef VAR_UR10E_MAIN_H
#define VAR_UR10E_MAIN_H

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
extern float SwitchS[5];
extern float SwitchPre[5];
extern int PointTraj[30]; //Point or Trajectory, 0=none, 1=Path point, 2=Trajectory
extern char filename[100];
extern int SEQ_switch,pointnum,SEQ_play,stop_flag,stop_flag2;
extern int SEQ_record,SEQ_point,SEQ_traj;

/* UR Setup parameters */
extern std::string robot_ip;
//// double rtde_frequency = 500.0; // Hz 500
extern double dt; // 2ms
//// uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
extern int ur_cap_port;

/* ur_rtde realtime priorities */
//// int rt_receive_priority = 90;
//// int rt_control_priority = 85;

//// RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
//// RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);
// RTDEIOInterface rtde_io(NRS_IP["UR10IP"].as<std::string>());
//// RTDEIOInterface rtde_io(robot_ip);

/*---------------------------- Parameters ----------------------------*/
/* Motion parameters */
extern double vel;
extern double acc;
extern double velocity;
extern double acceleration;

/* Servo control parameters */
extern double lookahead_time;
extern double gain;

/* flag */
extern int ctrl; // control mode status
extern int pre_ctrl; // previous control mode status
extern char message_status[32]; // message status;
extern int pause_cnt;
extern int speedmode;
extern int pre_speedmode;
extern char key_MODE;
extern int printer_counter;
extern int print_period; // counter per print


extern std::vector<double> joint_q; // servoJ cmd angle
extern std::vector<double> tcp_pose; // actual tcp pose
extern steady_clock::time_point t_start; // control rt time setting (Do not change)

/* For cartesian motion command */
extern Vector3d Desired_XYZ, Desired_RPY;
extern Matrix3d Desired_rot;

/* [Force / torque] data */
extern Eigen::VectorXd ftS1; // Handle sensor
extern Eigen::VectorXd ftS2; // Contact sensor

/* Hand-guding admittance control parameters */
extern Eigen::VectorXd Hadm_pos_cmd;
extern Eigen::VectorXd Hadm_past_pos_cmd;
extern Eigen::VectorXd Hadm_vel_cmd;
extern Eigen::VectorXd Hadm_past_vel_cmd;

extern Eigen::VectorXd Hadm_pos_act;
extern Eigen::VectorXd Hadm_past_pos_act;
extern Eigen::VectorXd Hadm_2past_pos_act;
extern Eigen::VectorXd Hadm_vel_act;
extern Eigen::VectorXd Hadm_past_vel_act;

/* Teaching force shaping method */
extern Eigen::MatrixXd Shaped_Force;
extern double shaped_F_criteria, shaped_F_dot;
extern double shaped_F_alpha; // rad
extern double shaped_F_offset; // N
extern Eigen::MatrixXd shaped_F_compen;

extern Eigen::Vector3d Handle_Rot_force, Handle_Rot_moment;
extern Eigen::Vector3d Contact_Rot_force, Contact_Rot_moment;

extern Eigen::VectorXd Hadm_FT_data;

extern Eigen::VectorXd Hadmit_M;
extern Eigen::VectorXd Hadmit_D;
extern Eigen::VectorXd Hadmit_K;

extern Eigen::VectorXd Cadmit_PB_M;
extern Eigen::VectorXd Cadmit_PB_D;
extern Eigen::VectorXd Cadmit_PB_K;

extern Yoon_force_control Hadmit_force[6];
extern Yoon_force_control Cadmit_playback[6];

/* For Energy tank hand-guiding */
extern Eigen::VectorXd PHadmit_M;
extern Eigen::VectorXd PHadmit_D;

/* For energy tank */
extern Eigen::VectorXf et_car_vel;
extern Eigen::VectorXf et_pcar_vel;
extern Eigen::VectorXf et_an_vel;
extern double et_norm[6];
extern double et_norm_threshold[6]; // Algorithm execution threshold
extern Yoon_filters et_HFT_HPF[6]; // High pass filter for hand-guiding energy tank

extern double admit_FT_in[6];
extern double admit_vel_in[6];

extern Etank_HG AD_HG_et[6]; // Admittance control hand-guding energy tank instance
extern Etank_HG et_adap; // energy tank paramter structure for adaptation

extern double Md_ti[6]; // Mass at the ti
extern double Md_tf[6]; // Mass at the tf
extern double xm_dot[6]; // Velocity limit(m/s & rad/s)
extern double M_divi_lim[6]; // Mass diviation limit
extern double Dd_tf[6]; // Damping at the tf
extern double deltaT; // update time interval (ms)
extern double et_wait_counter[6]; // update time counter
extern bool par_adap_flag[6];
extern int par_adap_stCounter;
extern int et_decre_counter[6];
extern double init_divi_M[6];
extern double et_decre_par[6]; // energy tank parameter decrement level (low value -> slow)

/*---------------------------- Callback functions ----------------------------*/
extern Eigen::Vector3d Rot_force1,Rot_force2,Rot_moment1,Rot_moment2;
extern Eigen::Vector3d Rot_Cforce1,Rot_Cforce2,Rot_Cmoment1,Rot_Cmoment2;
extern Eigen::Matrix3d HTM_URRot, HTM_FT2UR, HTM_CFT2UR;
extern double Handle_weight; //unit : N (must be plus)
extern double Tool_weight; //set the correct value!!! (Spindle : 16 N)

extern uint16_t mode_cmd;
extern Yoon_path path_planning,PB_PL_X,PB_PL_Y,PB_PL_Z,PB_PL_RX,PB_PL_RY,PB_PL_RZ; // path planning instance
extern Yoon_path J_single; // Single joint path planning
extern Eigen::VectorXd TCP_path_start;
extern Eigen::VectorXd Joint_path_start;

/* Status message start */
extern char path_gen_done[32];
extern char ST_path_gen_done[32];
extern char Hand_guiding_mode[32];
extern char Motion_stop_mode[32];
extern char Data_recording_on[32];
extern char Data_recording_off[32];
extern char Saved_way_point[32];
extern char Playback_terminated[32];
extern char Playback_iteration[32];

/* Status message end */
extern int Path_point_num;
extern bool path_done_flag;
extern bool PB_starting_path_done_flag;
extern bool path_recording_flag;
extern FILE *path_recording_pos, *path_recording_joint, *Hand_G_recording, *Hand_G_playback;
extern FILE *Descre_P_recording, *EXPdata1, *VRCali_UR10CB_EE, *VRCali_UR10CB_VR;
extern int EXPdata1_switch;
extern float LD_X,LD_Y,LD_Z,LD_Roll,LD_Pitch,LD_Yaw,LD_resi,LD_CFx,LD_CFy,LD_CFz; // Loaded XYZRPY for playback
extern double PB_RCF_norm; // For contact detection on playback control with recorded contact force
extern double PB_des_CF[3]; // Desired contact force at power playback
extern std::vector<double> mjoint_cmd;
extern Yoon_path Posture_PB; // Posture playback instance
extern DS_power_playback Power_PB; // Power(posture & force) playback instance
extern DS_power_PB_RTinput PPB_RTinput; // Power playback real-time input instance
extern double Norm_LD_CF;
extern double CF_gen_threshold; // Contact force generation threshold(unit: N)
extern double PPB_surfN_Fext;
extern uint32_t PB_iter_cmd; // number of playback iteration
extern uint32_t PB_iter_cur; // current itereation status

/* descrete points path parms */
extern Eigen::MatrixXd Decr_RD_points;
extern Eigen::MatrixXd Inst_RD_points;
extern int Num_RD_points;

extern Eigen::MatrixXd Decr_EE_points;
extern Eigen::MatrixXd Inst_EE_points;
extern int Num_EE_points;
extern Eigen::MatrixXd Decr_VR_points; // Quaternion (7 cols.)
extern Eigen::MatrixXd Inst_VR_points; // Quaternion (7 cols.)
extern int Num_VR_points;
extern Yoon_path Descr_RD_blending1,Descr_RD_blending2,Descr_RD_blending3;
extern bool Descr_RD_blending_done;

/* Variable stiffness control (Kd to Zero) */
extern bool KdToZero_flag, ZeroToKd_flag, KTZ_Fd_flag;
extern double Kd_change_dist; // Kd changed distance, Unit: m
extern double KTZ_Kd_init; // Save the initial contact stiffness
extern double KTZ_update_par; // Kd to Z, Z to Kd upadte parameter (half-Ts is proper)
extern double KTZ_Z_init, KTZ_Z_end;
extern double KTZ_Kd_threshold;
extern double KTZ_Kd_h;
extern double CR_startZP,CR_endZP;
extern Vector3d CR_start, CR_start_dist;

/* Dynamical system based adaptive variable damping admittance law */
extern int Contact_Fcon_mode;
extern double DB_AVA_Rd; // Desired update ratio, 0~1, (0.625) -> to make the sigma 0.5
extern double DB_AVA_sigma; // Calculated update rate
extern double DB_AVA_phi ;
extern double DB_AVA_Dd_init;
extern double DB_AVA_Xc, DB_AVA_Xc_pre, DB_AVA_Xc_dot;
extern double DB_AVA_Dsature[2]; // Damping saturation MAX, MIN value

/* Dynamical system based adaptive variable stiffness admittance law */
extern double DB_AVA_Kd_init;
extern double DB_AVA_Xr, DB_AVA_X;
extern double DB_AVA_Ksature[2]; // Damping saturation MAX, MIN value

/* Fuzzy based adaptive variable stiffness admittance law */
extern double Fuzzy_F_error, Fuzzy_F_Perror, Fuzzy_F_error_dot;
extern Fuzzy_adaptive_k FAK;

/* Fuzzy based adaptive variable mass & damping admittance law */
extern double Fuzzy_mass_var, Fuzzy_md_ratio;
extern double Fuzzy_mass_limit[2]; // {0.5, 5}
// Fuzzy_adaptive_md FAMD("cu");
extern double FAAC_HPF_cf; // Hz
extern double FAAC_HPF_threshold; // N
// extern Fuzzy_adaptive_md FAMD,
// NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3"].as<double>(),
// NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>(), dt, FAAC_HPF_cf, FAAC_HPF_threshold);

/* Three-step FAAC */
extern std::vector<double> process_noise;
extern std::vector<double> measurement_noise;
extern Nrs3StepFAAC FAAC3step;
// extern Nrs3StepFAAC FAAC3step,
// NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3"].as<double>(),
// NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>(), dt, process_noise, measurement_noise);


/* VR parameters */
extern bool VR_yaml_loader; // VR yaml calibration matrix load flag
extern double VR_pose[7]; // VR pose (Position-3, Orientation-quaternion)
extern double VR_cal_pose[7];

extern Eigen::MatrixXd VR_Q2Rot;
extern Eigen::MatrixXd VR_PoseM;
extern Eigen::MatrixXd VR_CalPoseM;
extern Eigen::VectorXd VR_CalRPY;
extern Eigen::VectorXd VR_CalPoseRPY;
extern Eigen::MatrixXd VR_Cali_TAD;
extern Eigen::MatrixXd VR_Cali_TBC;
extern Eigen::MatrixXd VR_Cali_TBC_inv;
extern Eigen::MatrixXd VR_Cali_TBC_PB;
extern Eigen::MatrixXd VR_Cali_TCE;
extern Eigen::MatrixXd VR_Cali_RAdj;
extern Eigen::MatrixXd VR_Cali_TAdj;

#endif // VAR_UR10E_MAIN_H
