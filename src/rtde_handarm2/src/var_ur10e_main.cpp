#include "var_ur10e_main.h"

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
// using namespace ur_rtde;
using namespace std::chrono;
using namespace std;
AKfun AKin;
CArm RArm;
Armtraj A_Traj;
ArmGuide A_Guide;

FILE *fp_ur_record, *fp_servo, *fp_record, *fp_replay;

/*---------------------------- Yaml file load ---------------------------------*/
std::ifstream fin1(NRS_UR10_IP_loc);
std::ifstream fin2(NRS_Record_Printing_loc);
std::ifstream fin3(NRS_Fcon_setting_loc);
std::ifstream fin4(NRS_VR_setting_loc);
YAML::Node NRS_IP = YAML::Load(fin1);
YAML::Node NRS_recording = YAML::Load(fin2);
YAML::Node NRS_Fcon_setting = YAML::Load(fin3);
YAML::Node NRS_VR_setting = YAML::Load(fin4);

/*---------------------------- System setting ---------------------------------*/
float SwitchS[5];
float SwitchPre[5];
int PointTraj[30]; //Point or Trajectory, 0=none, 1=Path point, 2=Trajectory
char filename[100];
int SEQ_switch=0,pointnum=0,SEQ_play=0,stop_flag=0,stop_flag2=0;
int SEQ_record=0,SEQ_point=0,SEQ_traj=0;

/* UR Setup parameters */
std::string robot_ip = NRS_IP["UR10IP"].as<std::string>();
//// double rtde_frequency = 500.0; // Hz 500
double dt = 0.002; // 2ms
//// uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
int ur_cap_port = 50002;

/* ur_rtde realtime priorities */
//// int rt_receive_priority = 90;
//// int rt_control_priority = 85;

//// RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
//// RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);
//// RTDEIOInterface rtde_io(NRS_IP["UR10IP"].as<std::string>());
//// RTDEIOInterface rtde_io(robot_ip);

/*---------------------------- Parameters ----------------------------*/
/* Motion parameters */
double vel = 0.5;
double acc = 10.0;
double velocity = 0;
double acceleration = 0;

/* Servo control parameters */
double lookahead_time = 0.1;
double gain = 600;

/* flag */
std::atomic<int> ctrl{0};       //// int ctrl=0;     // control mode status
std::atomic<int> pre_ctrl{0};   //// int pre_ctrl=0; // previous control mode status
char message_status[32] = "Motion stop"; // message status;
int pause_cnt=0;
int speedmode = 0;
int pre_speedmode = 0;
char key_MODE='0';
int printer_counter = 0;
int print_period = 20; // counter per print

std::vector<double> joint_q;      // servoJ cmd angle
std::vector<double> tcp_pose;     // actual tcp pose
steady_clock::time_point t_start; // control rt time setting (Do not change)

/* For cartesian motion command */
Vector3d Desired_XYZ, Desired_RPY;
Matrix3d Desired_rot;

/* [Force / torque] data */
Eigen::VectorXd ftS1 = Eigen::VectorXd::Zero(6); // Handle sensor
Eigen::VectorXd ftS2 = Eigen::VectorXd::Zero(6); // Contact sensor

/* Hand-guding admittance control parameters */
Eigen::VectorXd Hadm_pos_cmd = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadm_past_pos_cmd = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadm_vel_cmd = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadm_past_vel_cmd = Eigen::VectorXd::Zero(6);

Eigen::VectorXd Hadm_pos_act = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadm_past_pos_act = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadm_2past_pos_act = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadm_vel_act = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadm_past_vel_act = Eigen::VectorXd::Zero(6);

/* Teaching force shaping method */
Eigen::MatrixXd Shaped_Force = Eigen::MatrixXd::Zero(3,1);
double shaped_F_criteria, shaped_F_dot;
double shaped_F_alpha = 0.05; // rad
double shaped_F_offset = 1;   // N
Eigen::MatrixXd shaped_F_compen = Eigen::MatrixXd::Zero(3,1);

Eigen::Vector3d Handle_Rot_force(3), Handle_Rot_moment(3);
Eigen::Vector3d Contact_Rot_force(3), Contact_Rot_moment(3);

Eigen::VectorXd Hadm_FT_data = Eigen::VectorXd::Zero(6);

Eigen::VectorXd Hadmit_M = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadmit_D = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadmit_K = Eigen::VectorXd::Zero(6);

Eigen::VectorXd Cadmit_PB_M = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Cadmit_PB_D = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Cadmit_PB_K = Eigen::VectorXd::Zero(6);

Yoon_force_control Hadmit_force[6];
Yoon_force_control Cadmit_playback[6];

/* For Energy tank hand-guiding */
Eigen::VectorXd PHadmit_M = Eigen::VectorXd::Zero(6);
Eigen::VectorXd PHadmit_D = Eigen::VectorXd::Zero(6);

/* For energy tank */
Eigen::VectorXf et_car_vel = Eigen::VectorXf::Zero(6);
Eigen::VectorXf et_pcar_vel = Eigen::VectorXf::Zero(6);
Eigen::VectorXf et_an_vel = Eigen::VectorXf::Zero(6);
double et_norm[6] = {0,};
double et_norm_threshold[6] = {0.8,0.8,0.8,0.08,0.08,0.08}; // Algorithm execution threshold
Yoon_filters et_HFT_HPF[6]; // High pass filter for hand-guiding energy tank

double admit_FT_in[6] = {0,};
double admit_vel_in[6] = {0,};

Etank_HG AD_HG_et[6]; // Admittance control hand-guding energy tank instance
Etank_HG et_adap; // energy tank paramter structure for adaptation

double Md_ti[6] = {0,}; // Mass at the ti
double Md_tf[6] = {0,}; // Mass at the tf
double xm_dot[6] = {1,1,1,0.05,0.05,0.05}; // Velocity limit(m/s & rad/s)
double M_divi_lim[6] = {3,3,3,0.3,0.3,0.3}; // Mass diviation limit
double Dd_tf[6] = {0,}; // Damping at the tf
double deltaT = 0.2; // update time interval (ms)
double et_wait_counter[6] = {0,}; // update time counter
bool par_adap_flag[6] = {false,};
int par_adap_stCounter = 0;
int et_decre_counter[6] = {0,};
double init_divi_M[6] = {0,};
double et_decre_par[6] = {0.0004,0.0004,0.0004,0.0004,0.0004,0.0004}; // energy tank parameter decrement level (low value -> slow)

/*---------------------------- Callback functions ----------------------------*/
Eigen::Vector3d Rot_force1,Rot_force2,Rot_moment1,Rot_moment2;
Eigen::Vector3d Rot_Cforce1,Rot_Cforce2,Rot_Cmoment1,Rot_Cmoment2;
Eigen::Matrix3d HTM_URRot, HTM_FT2UR, HTM_CFT2UR;
double Handle_weight = 3.0; //unit : N (must be plus)
double Tool_weight = 16.0; //set the correct value!!! (Spindle : 16 N)

uint16_t mode_cmd = 0;
Yoon_path path_planning,PB_PL_X,PB_PL_Y,PB_PL_Z,PB_PL_RX,PB_PL_RY,PB_PL_RZ; // path planning instance
Yoon_path J_single; // Single joint path planning
Eigen::VectorXd TCP_path_start = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Joint_path_start = Eigen::VectorXd::Zero(6);

/* Status message start */
char path_gen_done[32]    = "Path generation done";
char ST_path_gen_done[32] = "Starting path generation done";
char Hand_guiding_mode[32] = "Hand guiding control mode";
char Motion_stop_mode[32] = "Motion stop";
char Data_recording_on[32] = "Data recording on";
char Data_recording_off[32] = "Data recording off";
char Saved_way_point[32] = "Saved way points: ";
char Playback_terminated[32] = "Payback was terminated";
char Playback_iteration[32] = "Payback: ";

/* Status message end */
int Path_point_num = -1;
bool path_done_flag = false;
bool PB_starting_path_done_flag = false;
bool path_recording_flag = false;
FILE *path_recording_pos, *path_recording_joint, *hand_g_recording, *Hand_G_playback, *Discre_P_recording,
*EXPdata1, *VRCali_UR10CB_EE, *VRCali_UR10CB_VR;
int EXPdata1_switch = NRS_recording["EXPdata1_switch"].as<int>();
float LD_X,LD_Y,LD_Z,LD_Roll,LD_Pitch,LD_Yaw,LD_resi,LD_CFx,LD_CFy,LD_CFz; // Loaded XYZRPY for playback
double PB_RCF_norm = 0; // For contact detection on playback control with recorded contact force
double PB_des_CF[3] = {0,}; // Desired contact force at power playback
std::vector<double> mjoint_cmd;
Yoon_path Posture_PB; // Posture playback instance
DS_power_playback Power_PB; // Power(posture & force) playback instance
DS_power_PB_RTinput PPB_RTinput; // Power playback real-time input instance
double Norm_LD_CF = 0;
double CF_gen_threshold = 0; // Contact force generation threshold(unit: N)
double PPB_surfN_Fext = 0;
uint32_t PB_iter_cmd = 0; // number of playback iteration
uint32_t PB_iter_cur = 0; // current itereation status

/* Discrete points path parms */
Eigen::MatrixXd Decr_RD_points = Eigen::MatrixXd::Zero(1,6);
Eigen::MatrixXd Inst_RD_points = Eigen::MatrixXd::Zero(1,6);
int Num_RD_points = 0;

Eigen::MatrixXd Decr_EE_points = Eigen::MatrixXd::Zero(1,12);
Eigen::MatrixXd Inst_EE_points = Eigen::MatrixXd::Zero(1,12);
int Num_EE_points = 0;
Eigen::MatrixXd Decr_VR_points = Eigen::MatrixXd::Zero(1,7); // Quaternion (7 cols.)
Eigen::MatrixXd Inst_VR_points = Eigen::MatrixXd::Zero(1,7); // Quaternion (7 cols.)
int Num_VR_points = 0;
Yoon_path Descr_RD_blending1,Descr_RD_blending2,Descr_RD_blending3;
bool Descr_RD_blending_done = false;

/* Variable stiffness control (Kd to Zero) */
bool KdToZero_flag, ZeroToKd_flag, KTZ_Fd_flag;
double Kd_change_dist; // Kd changed distance, Unit: m
double KTZ_Kd_init; // Save the initial contact stiffness
double KTZ_update_par; // Kd to Z, Z to Kd upadte parameter (half-Ts is proper)
double KTZ_Z_init, KTZ_Z_end;
double KTZ_Kd_threshold;
double KTZ_Kd_h;
double CR_startZP,CR_endZP;
Vector3d CR_start, CR_start_dist;

/* Dynamical system based adaptive variable damping admittance law */
int Contact_Fcon_mode = NRS_Fcon_setting["Contact_Fcon_mode"].as<int>();
double DB_AVA_Rd = NRS_Fcon_setting["AAC_update_ratio"].as<double>(); // Desired update ratio, 0~1, (0.625) -> to make the sigma 0.5
double DB_AVA_sigma; // Calculated update rate
double DB_AVA_phi = 0;
double DB_AVA_Dd_init;
double DB_AVA_Xc, DB_AVA_Xc_pre, DB_AVA_Xc_dot;
double DB_AVA_Dsature[2] = {NRS_Fcon_setting["AAC_Dsature"]["Lower_limit"].as<double>(),
NRS_Fcon_setting["AAC_Dsature"]["Upper_limit"].as<double>()}; // Damping saturation MAX, MIN value

/* Dynamical system based adaptive variable stiffness admittance law */
double DB_AVA_Kd_init;
double DB_AVA_Xr, DB_AVA_X;
double DB_AVA_Ksature[2] = {NRS_Fcon_setting["AAC_Ksature"]["Lower_limit"].as<double>(),
NRS_Fcon_setting["AAC_Ksature"]["Upper_limit"].as<double>()}; // Damping saturation MAX, MIN value

/* Fuzzy based adaptive variable stiffness admittance law */
double Fuzzy_F_error, Fuzzy_F_Perror, Fuzzy_F_error_dot;
Fuzzy_adaptive_k FAK("Fs_Cf");

/* Fuzzy based adaptive variable mass & damping admittance law */
double Fuzzy_mass_var, Fuzzy_md_ratio;
double Fuzzy_mass_limit[2] = {0.5, 5}; // {0.5, 5}
// Fuzzy_adaptive_md FAMD("cu");
double FAAC_HPF_cf = 15; // Hz
double FAAC_HPF_threshold = 1; // N
Fuzzy_adaptive_md FAMD("cu",NRS_Fcon_setting["ContactDesiredMass"]["LamdaM3"].as<double>(),
NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3"].as<double>(),
NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>(), dt, FAAC_HPF_cf, FAAC_HPF_threshold);

/* Three-step FAAC */
std::vector<double> process_noise    = {0.1, 0.1, 0.1};
std::vector<double> measurement_noise = {10, 10, 10};

Nrs3StepFAAC FAAC3step(
  NRS_Fcon_setting["ContactDesiredMass"]["LamdaM3"].as<double>(),
  NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3"].as<double>(),
  NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>(),
  dt,
  process_noise,
  measurement_noise
);


/* VR parameters */
bool VR_yaml_loader = false; // VR yaml calibration matrix load flag
double VR_pose[7] = {0,}; // VR pose (Position-3, Orientation-quaternion)
double VR_cal_pose[7] = {0,};

Eigen::MatrixXd VR_Q2Rot = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd VR_PoseM = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_CalPoseM = Eigen::MatrixXd::Zero(4,4);
Eigen::VectorXd VR_CalRPY = Eigen::VectorXd::Zero(3);
Eigen::VectorXd VR_CalPoseRPY = Eigen::VectorXd::Zero(6);
Eigen::MatrixXd VR_Cali_TAD = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_TBC = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_TBC_inv = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_TBC_PB = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_TCE = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_RAdj = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd VR_Cali_TAdj = Eigen::MatrixXd::Zero(4,4);
