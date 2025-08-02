// #include <stdio.h>
// #include "Yoon_FT_sensor.h"
// #include "Yoon_filters.h"
// #include "ros/ros.h"
// #include "rtde_handarm/ftsensorMsg.h"
// #include <signal.h>
// #include <ctime>
// #include <std_srvs/Empty.h>
// #include <fstream>
// #include <yaml-cpp/yaml.h>
// #include <iostream>
// #include "NRS_yaml_location.h"

// #define H_Mov_OnOff 1
// #define C_Mov_OnOff 0
// #define H_LPF_OnOff 1
// #define C_LPF_OnOff 0

// class YoonFTAcquisition {
// private:
//     ros::NodeHandle nh;
//     ros::Publisher ft_pub;
//     ros::ServiceServer sensor_srv;
//     Yoon_FT_sensor FT_sensor;
//     rtde_handarm::ftsensorMsg msg;
//     FILE *Data1_txt;
//     double time_counter = 0;
//     double time_step = 0.02;
//     char key_MODE = '0';

//     #if H_Mov_OnOff
//     Yoon_filters MV_filter_Fx, MV_filter_Fy, MV_filter_Fz;
//     Yoon_filters MV_filter_Mx, MV_filter_My, MV_filter_Mz;
//     #endif

//     #if C_Mov_OnOff
//     Yoon_filters MV_filter_CFx, MV_filter_CFy, MV_filter_CFz;
//     Yoon_filters MV_filter_CMx, MV_filter_CMy, MV_filter_CMz;
//     #endif

//     #if H_LPF_OnOff
//     Yoon_filters LPF_Fx, LPF_Fy, LPF_Fz;
//     Yoon_filters LPF_Mx, LPF_My, LPF_Mz;
//     #endif

//     #if C_LPF_OnOff
//     Yoon_filters LPF_CFx, LPF_CFy, LPF_CFz;
//     Yoon_filters LPF_CMx, LPF_CMy, LPF_CMz;
//     #endif

//     double HSam_freq = 50;
//     double CSam_freq = 500;
//     int HMov_num = 3;
//     int CMov_num = 3;
//     double HLPF_CF = 2;
//     double CLPF_CF = 10;

//     std::ifstream fin1{NRS_UR10_IP_loc};
//     std::ifstream fin2{NRS_Record_Printing_loc};
//     YAML::Node NRS_IP = YAML::Load(fin1);
//     YAML::Node NRS_recording = YAML::Load(fin2);

//     static void signalHandler(int sig) {
//         printf("FT_acquisit was terminated\n");
//         exit(1);
//     }

//     bool sensorZeroSetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
//         FT_sensor.sensor_init_counter = 0;
// 		time_counter =0;
//         return true;
//     }

//     void setupFilters() {
//         #if H_Mov_OnOff
//         MV_filter_Fx.MV_par.mv_num = HMov_num;
//         MV_filter_Fy.MV_par.mv_num = HMov_num;
//         MV_filter_Fz.MV_par.mv_num = HMov_num;

//         MV_filter_Mx.MV_par.mv_num = HMov_num;
//         MV_filter_My.MV_par.mv_num = HMov_num;
//         MV_filter_Mz.MV_par.mv_num = HMov_num;
//         #endif

//         #if C_Mov_OnOff
//         MV_filter_CFx.MV_par.mv_num = CMov_num;
//         MV_filter_CFy.MV_par.mv_num = CMov_num;
//         MV_filter_CFz.MV_par.mv_num = CMov_num;

//         MV_filter_CMx.MV_par.mv_num = CMov_num;
//         MV_filter_CMy.MV_par.mv_num = CMov_num;
//         MV_filter_CMz.MV_par.mv_num = CMov_num;
//         #endif

//         #if H_LPF_OnOff
//         LPF_Fx.LPF_par.SamplingFrequency = HSam_freq;
//         LPF_Fy.LPF_par.SamplingFrequency = HSam_freq;
//         LPF_Fz.LPF_par.SamplingFrequency = HSam_freq;

//         LPF_Mx.LPF_par.SamplingFrequency = HSam_freq;
//         LPF_My.LPF_par.SamplingFrequency = HSam_freq;
//         LPF_Mz.LPF_par.SamplingFrequency = HSam_freq;

//         LPF_Fx.LPF_par.CutOffFrequency = HLPF_CF;
//         LPF_Fy.LPF_par.CutOffFrequency = HLPF_CF;
//         LPF_Fz.LPF_par.CutOffFrequency = HLPF_CF;

//         LPF_Mx.LPF_par.CutOffFrequency = HLPF_CF;
//         LPF_My.LPF_par.CutOffFrequency = HLPF_CF;
//         LPF_Mz.LPF_par.CutOffFrequency = HLPF_CF;
//         #endif

//         #if C_LPF_OnOff
//         LPF_CFx.LPF_par.SamplingFrequency = CSam_freq;
//         LPF_CFy.LPF_par.SamplingFrequency = CSam_freq;
//         LPF_CFz.LPF_par.SamplingFrequency = CSam_freq;

//         LPF_CMx.LPF_par.SamplingFrequency = CSam_freq;
//         LPF_CMy.LPF_par.SamplingFrequency = CSam_freq;
//         LPF_CMz.LPF_par.SamplingFrequency = CSam_freq;

//         LPF_CFx.LPF_par.CutOffFrequency = CLPF_CF;
//         LPF_CFy.LPF_par.CutOffFrequency = CLPF_CF;
//         LPF_CFz.LPF_par.CutOffFrequency = CLPF_CF;

//         LPF_CMx.LPF_par.CutOffFrequency = CLPF_CF;
//         LPF_CMy.LPF_par.CutOffFrequency = CLPF_CF;
//         LPF_CMz.LPF_par.CutOffFrequency = CLPF_CF;
//         #endif
//     }

// public:
//     YoonFTAcquisition() {
//         ft_pub = nh.advertise<rtde_handarm::ftsensorMsg>("/ftsensor", 10);
//         sensor_srv = nh.advertiseService("sensor_zeroset", &YoonFTAcquisition::sensorZeroSetCallback, this);
//         signal(SIGTERM, YoonFTAcquisition::signalHandler);
//         signal(SIGINT, YoonFTAcquisition::signalHandler);

//         auto YamlString_IP = NRS_IP["AFT80IP"].as<std::string>();
//         char* AFT80_IP = const_cast<char*>(YamlString_IP.c_str());
//         FT_sensor.TCP_init(AFT80_IP, 4001);

//         auto YamlData1_path = NRS_recording["Data1_path"].as<std::string>();
//         Data1_txt = fopen(YamlData1_path.c_str(), "wt");
//         setupFilters();
//     }

//     void run() {
//         printf("MODE SELECT: printing mode\n");
//         printf("1 : Printing FT data,  2 : Non-Printing FT data\n");
//         key_MODE = getchar();

// 		while (ros::ok()) {
// 			if (FT_sensor.TCP_start() != 0) {
// 				if (time_counter >= 200) {
// 					FT_sensor.Sensor_value_init();
// 				}

// 				/* Handle sensor proccesing */
// 				#if H_Mov_OnOff
// 				FT_sensor.Force_val[0] = MV_filter_Fx.MovingAvgFilter(FT_sensor.Force_val[0]);
// 				FT_sensor.Force_val[1] = MV_filter_Fy.MovingAvgFilter(FT_sensor.Force_val[1]);
// 				FT_sensor.Force_val[2] = MV_filter_Fz.MovingAvgFilter(FT_sensor.Force_val[2]);

// 				FT_sensor.Moment_val[0] = MV_filter_Mx.MovingAvgFilter(FT_sensor.Moment_val[0]);
// 				FT_sensor.Moment_val[1] = MV_filter_My.MovingAvgFilter(FT_sensor.Moment_val[1]);
// 				FT_sensor.Moment_val[2] = MV_filter_Mz.MovingAvgFilter(FT_sensor.Moment_val[2]);
// 				#endif

// 				#if H_LPF_OnOff
// 				FT_sensor.Force_val[0] = LPF_Fx.LowPassFilter(FT_sensor.Force_val[0]);
// 				FT_sensor.Force_val[1] = LPF_Fy.LowPassFilter(FT_sensor.Force_val[1]);
// 				FT_sensor.Force_val[2] = LPF_Fz.LowPassFilter(FT_sensor.Force_val[2]);

// 				FT_sensor.Moment_val[0] = LPF_Mx.LowPassFilter(FT_sensor.Moment_val[0]);
// 				FT_sensor.Moment_val[1] = LPF_My.LowPassFilter(FT_sensor.Moment_val[1]);
// 				FT_sensor.Moment_val[2] = LPF_Mz.LowPassFilter(FT_sensor.Moment_val[2]);
// 				#endif

// 				/* Contact sensor proccesing */
// 				#if C_Mov_OnOff
// 				FT_sensor.Contact_Force_val[0] = MV_filter_CFx.MovingAvgFilter(FT_sensor.Contact_Force_val[0]);
// 				FT_sensor.Contact_Force_val[1] = MV_filter_CFy.MovingAvgFilter(FT_sensor.Contact_Force_val[1]);
// 				FT_sensor.Contact_Force_val[2] = MV_filter_CFz.MovingAvgFilter(FT_sensor.Contact_Force_val[2]);

// 				FT_sensor.Contact_Moment_val[0] = MV_filter_CMx.MovingAvgFilter(FT_sensor.Contact_Moment_val[0]);
// 				FT_sensor.Contact_Moment_val[1] = MV_filter_CMy.MovingAvgFilter(FT_sensor.Contact_Moment_val[1]);
// 				FT_sensor.Contact_Moment_val[2] = MV_filter_CMz.MovingAvgFilter(FT_sensor.Contact_Moment_val[2]);
// 				#endif

// 				#if C_LPF_OnOff
// 				FT_sensor.Contact_Force_val[0] = LPF_CFx.LowPassFilter(FT_sensor.Contact_Force_val[0]);
// 				FT_sensor.Contact_Force_val[1] = LPF_CFy.LowPassFilter(FT_sensor.Contact_Force_val[1]);
// 				FT_sensor.Contact_Force_val[2] = LPF_CFz.LowPassFilter(FT_sensor.Contact_Force_val[2]);

// 				FT_sensor.Contact_Moment_val[0] = LPF_CMx.LowPassFilter(FT_sensor.Contact_Moment_val[0]);
// 				FT_sensor.Contact_Moment_val[1] = LPF_CMy.LowPassFilter(FT_sensor.Contact_Moment_val[1]);
// 				FT_sensor.Contact_Moment_val[2] = LPF_CMz.LowPassFilter(FT_sensor.Contact_Moment_val[2]);
// 				#endif

// 				if (key_MODE == '1') {
// 					printf("Fx: %10f,  Fy: %10f,  Fz: %10f\n", FT_sensor.Force_val[0], FT_sensor.Force_val[1], FT_sensor.Force_val[2]);
// 					printf("Mx: %10f,  My: %10f,  Mz: %10f\n", FT_sensor.Moment_val[0], FT_sensor.Moment_val[1], FT_sensor.Moment_val[2]);
// 					printf("CFx: %10f, CFy: %10f, CFz: %10f\n", FT_sensor.Contact_Force_val[0], FT_sensor.Contact_Force_val[1], FT_sensor.Contact_Force_val[2]);
// 					printf("CMx: %10f, CMy: %10f, CMz: %10f\n", FT_sensor.Contact_Moment_val[0], FT_sensor.Contact_Moment_val[1], FT_sensor.Contact_Moment_val[2]);
// 				}

// 				msg.Fx = FT_sensor.Force_val[0];
// 				msg.Fy = FT_sensor.Force_val[1];
// 				msg.Fz = FT_sensor.Force_val[2];

// 				msg.Mx = FT_sensor.Moment_val[0];
// 				msg.My = FT_sensor.Moment_val[1];
// 				msg.Mz = FT_sensor.Moment_val[2];

// 				msg.CFx = FT_sensor.Contact_Force_val[0];
// 				msg.CFy = FT_sensor.Contact_Force_val[1];
// 				msg.CFz = FT_sensor.Contact_Force_val[2];

// 				msg.CMx = FT_sensor.Contact_Moment_val[0];
// 				msg.CMy = FT_sensor.Contact_Moment_val[1];
// 				msg.CMz = FT_sensor.Contact_Moment_val[2];

// 				ft_pub.publish(msg);
// 				time_counter += 1.0;
// 				ros::spinOnce();
// 			}
// 		}
//         fclose(Data1_txt);
//     }
// };

// int main(int argc, char* argv[]) {
//     ros::init(argc, argv, "Yoon_FT_acquisit");
//     YoonFTAcquisition node;
//     node.run();
//     return 0;
// }



