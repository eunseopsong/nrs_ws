// src/globals.cpp

#include "Yoon_UR10e_main.h"
#include <fstream>

namespace {
// YAML 파일 로드를 위한 내부 스트림
std::ifstream fin1(NRS_UR10_IP_loc);
std::ifstream fin2(NRS_Record_Printing_loc);
std::ifstream fin3(NRS_Fcon_setting_loc);
std::ifstream fin4(NRS_VR_setting_loc);
}

// YAML 노드 정의
YAML::Node NRS_IP            = YAML::Load(fin1);
YAML::Node NRS_recording     = YAML::Load(fin2);
YAML::Node NRS_Fcon_setting  = YAML::Load(fin3);
YAML::Node NRS_VR_setting    = YAML::Load(fin4);

// 객체 정의
AKfun    AKin;
CArm     RArm;
Armtraj  A_Traj;
ArmGuide A_Guide;

// 시스템 파라미터
double    dt             = 0.002;
int       ur_cap_port    = 50002;

// 상태 변수
uint16_t                mode_cmd         = 0;
std::vector<double>     mjoint_cmd;
bool                    VR_yaml_loader   = false;
int                     Contact_Fcon_mode = NRS_Fcon_setting["Contact_Fcon_mode"].as<int>();

// Adaptive parameters
double DB_AVA_Rd                    = NRS_Fcon_setting["AAC_update_ratio"].as<double>();
double DB_AVA_Dsature[2]            = {
  NRS_Fcon_setting["AAC_Dsature"]["Lower_limit"].as<double>(),
  NRS_Fcon_setting["AAC_Dsature"]["Upper_limit"].as<double>()
};
double DB_AVA_Ksature[2]            = {
  NRS_Fcon_setting["AAC_Ksature"]["Lower_limit"].as<double>(),
  NRS_Fcon_setting["AAC_Ksature"]["Upper_limit"].as<double>()
};

Fuzzy_adaptive_k FAK("Fs_Cf");

double FAAC_HPF_threshold = 1.0;
double FAAC_HPF_cf        = 15.0;

Fuzzy_adaptive_md FAMD(
  "cu",
  NRS_Fcon_setting["ContactDesiredMass"]["LamdaM3"   ].as<double>(),
  NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3" ].as<double>(),
  NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>(),
  dt,
  FAAC_HPF_cf,
  FAAC_HPF_threshold
);

std::vector<double> process_noise     = {0.1,0.1,0.1};
std::vector<double> measurement_noise = {10,10,10};
Nrs3StepFAAC FAAC3step(
  NRS_Fcon_setting["ContactDesiredMass"]["LamdaM3"   ].as<double>(),
  NRS_Fcon_setting["ContactDesiredDamper"]["LamdaD3" ].as<double>(),
  NRS_Fcon_setting["ContactDesiredSpring"]["LamdaK3"].as<double>(),
  dt,
  process_noise,
  measurement_noise
);

// VR 변수
Eigen::MatrixXd VR_Q2Rot        = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd VR_PoseM        = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_CalPoseM     = Eigen::MatrixXd::Zero(4,4);
Eigen::VectorXd VR_CalRPY       = Eigen::VectorXd::Zero(3);
Eigen::VectorXd VR_CalPoseRPY   = Eigen::VectorXd::Zero(6);
Eigen::MatrixXd VR_Cali_TAD     = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_TBC     = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_TBC_inv = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_TBC_PB  = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_TCE     = Eigen::MatrixXd::Zero(4,4);
Eigen::MatrixXd VR_Cali_RAdj    = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd VR_Cali_TAdj    = Eigen::MatrixXd::Zero(4,4);

// 상태 메시지
char path_gen_done[32]       = "Path generation done";
char ST_path_gen_done[32]    = "Starting path generation done";
char Hand_guiding_mode[32]   = "Hand guiding control mode";
char Motion_stop_mode[32]    = "Motion stop";
char Data_recording_on[32]   = "Data recording on";
char Data_recording_off[32]  = "Data recording off";
char Saved_way_point[32]     = "Saved way points: ";

// 파일 핸들러
FILE *fp_ur_record = nullptr, *fp_servo = nullptr, *fp_record = nullptr, *fp_replay = nullptr;

// 기타 상태
bool path_recording_flag = false;
int  Num_RD_points       = 0;
int  Num_EE_points       = 0;
int  Num_VR_points       = 0;

// 추가적으로, 헤더에 extern 선언된 다른 전역 변수들도
// 모두 이곳에 “타입 이름 = 초기값;” 으로 한 번만 정의해주세요.
