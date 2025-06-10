// src/globals.cpp

#include "Yoon_UR10e_main.h"  // extern 선언만 남겨둔 헤더

// — YAML 파일 읽기 스트림 —
// (헤더에서 extern 으로 선언하지 않고, 여기서만 정의/초기화)
namespace {
  std::ifstream fin1(NRS_UR10_IP_loc);
  std::ifstream fin2(NRS_Record_Printing_loc);
  std::ifstream fin3(NRS_Fcon_setting_loc);
  std::ifstream fin4(NRS_VR_setting_loc);
}

// — YAML 노드 정의 —
// (헤더에는 extern 선언만, 여기서만 Load)
YAML::Node NRS_IP           = YAML::Load(fin1);
YAML::Node NRS_recording    = YAML::Load(fin2);
YAML::Node NRS_Fcon_setting = YAML::Load(fin3);
YAML::Node NRS_VR_setting   = YAML::Load(fin4);

// — 전역 객체 정의 —
AKfun    AKin;
CArm     RArm;
Armtraj  A_Traj;
ArmGuide A_Guide;

// — 시스템 파라미터 —
double dt          = 0.002;
int    ur_cap_port = 50002;

// — 상태/플래그/버퍼 변수 —
// 콜백에서 쓰는 변수들
uint16_t               mode_cmd       = 0;
std::vector<double>    mjoint_cmd;

// Admittance 제어 관련
Eigen::VectorXd Hadmit_M = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadmit_D = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Hadmit_K = Eigen::VectorXd::Zero(6);

Eigen::VectorXd Cadmit_PB_M = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Cadmit_PB_D = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Cadmit_PB_K = Eigen::VectorXd::Zero(6);

Yoon_force_control Hadmit_force[6];
Yoon_force_control Cadmit_playback[6];

Eigen::VectorXf et_car_vel  = Eigen::VectorXf::Zero(6);
Eigen::VectorXf et_pcar_vel = Eigen::VectorXf::Zero(6);
Eigen::VectorXf et_an_vel   = Eigen::VectorXf::Zero(6);

Yoon_filters et_HFT_HPF[6];

Etank_HG AD_HG_et[6];
Etank_HG et_adap;

// 경로 플래닝
Yoon_path path_planning;
Yoon_path PB_PL_X, PB_PL_Y, PB_PL_Z, PB_PL_RX, PB_PL_RY, PB_PL_RZ;
Eigen::VectorXd TCP_path_start   = Eigen::VectorXd::Zero(6);
Eigen::VectorXd Joint_path_start = Eigen::VectorXd::Zero(6);

// Power playback
int    EXPdata1_switch = NRS_recording["EXPdata1_switch"].as<int>();
DS_power_playback   Power_PB;
DS_power_PB_RTinput PPB_RTinput;

// Descrete & Euler point 저장
Eigen::MatrixXd Decr_RD_points   = Eigen::MatrixXd::Zero(1,6);
Eigen::MatrixXd Inst_RD_points   = Eigen::MatrixXd::Zero(1,6);
Eigen::MatrixXd Decr_EE_points   = Eigen::MatrixXd::Zero(1,12);
Eigen::MatrixXd Inst_EE_points   = Eigen::MatrixXd::Zero(1,12);
Eigen::MatrixXd Decr_VR_points   = Eigen::MatrixXd::Zero(1,7);
Eigen::MatrixXd Inst_VR_points   = Eigen::MatrixXd::Zero(1,7);

// Descrete blending
Yoon_path Descr_RD_blending1, Descr_RD_blending2, Descr_RD_blending3;

// Fuzzy/FAAC 등
int    Contact_Fcon_mode = NRS_Fcon_setting["Contact_Fcon_mode"].as<int>();
double DB_AVA_Rd         = NRS_Fcon_setting["AAC_update_ratio"].as<double>();
double DB_AVA_Dsature[2] = {
  NRS_Fcon_setting["AAC_Dsature"]["Lower_limit"].as<double>(),
  NRS_Fcon_setting["AAC_Dsature"]["Upper_limit"].as<double>()
};
double DB_AVA_Ksature[2] = {
  NRS_Fcon_setting["AAC_Ksature"]["Lower_limit"].as<double>(),
  NRS_Fcon_setting["AAC_Ksature"]["Upper_limit"].as<double>()
};
Fuzzy_adaptive_k FAK("Fs_Cf");

double FAAC_HPF_cf        = 15;
double FAAC_HPF_threshold = 1;
Fuzzy_adaptive_md FAMD(
  "cu",
  NRS_Fcon_setting["ContactDesiredMass"]["LamdaM3"].as<double>(),
  NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3"].as<double>(),
  NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>(),
  dt,
  FAAC_HPF_cf,
  FAAC_HPF_threshold
);

std::vector<double> process_noise     = {0.1,0.1,0.1};
std::vector<double> measurement_noise = {10,10,10};
Nrs3StepFAAC FAAC3step(
  NRS_Fcon_setting["ContactDesiredMass"]["LamdaM3"].as<double>(),
  NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3"].as<double>(),
  NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>(),
  dt,
  process_noise,
  measurement_noise
);

// VR 데이터
bool             VR_yaml_loader = false;
Eigen::MatrixXd  VR_Q2Rot        = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd  VR_PoseM        = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd  VR_CalPoseM     = Eigen::MatrixXd::Zero(4,4);
Eigen::VectorXd  VR_CalRPY       = Eigen::VectorXd::Zero(3);
Eigen::VectorXd  VR_CalPoseRPY   = Eigen::VectorXd::Zero(6);
Eigen::MatrixXd  VR_Cali_TAD     = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd  VR_Cali_TBC     = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd  VR_Cali_TBC_inv = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd  VR_Cali_TBC_PB  = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd  VR_Cali_TCE     = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd  VR_Cali_RAdj    = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd  VR_Cali_TAdj    = Eigen::MatrixXd::Zero(4,4);

// 전역 파일 핸들러
FILE *fp_ur_record = nullptr, *fp_servo = nullptr, *fp_record = nullptr, *fp_replay = nullptr;
bool path_recording_flag = false;

// 상태 메시지
char path_gen_done[32]     = "Path generation done";
char ST_path_gen_done[32]  = "Starting path generation done";
char Hand_guiding_mode[32] = "Hand guiding control mode";
char Motion_stop_mode[32]  = "Motion stop";
char Data_recording_on[32] = "Data recording on";
char Data_recording_off[32]= "Data recording off";
char Saved_way_point[32]   = "Saved way points: ";
