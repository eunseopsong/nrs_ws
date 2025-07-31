#ifndef FT_PROCESSING_H
#define FT_PROCESSING_H

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Wrench.h>
#include "NRS_FT_AQ/vive_ft_msg.h"
#include <std_srvs/Empty.h> // For ROS service - AIDIN GUI SERVER
#include "Gen_filter.hpp"
#include "CAN_reader.hpp"

class FT_processing : public NRS_FTSensor
{
    public:
        FT_processing(ros::NodeHandle nh, double Ts, unsigned char& HandleID_, unsigned char& ContactID_, bool HaccSwitch_, bool CaccSwitch_);
        ~FT_processing();
        
        void FT_init(int sen_init_num);
        void FT_filtering();

        void FT_publish();
        void FT_print();
        void FT_record();

        /* Service for sensor init */
        bool SRV5_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        void FT_run();

    private:

        /* Common parameters */
        double Ts_;
        double time_counter = 0;
        FILE *Data1_txt;
        bool runnning = true;

        /* Yaml */
        std::string YamlString_IP, YamlData1_path;
        int YamlData1_switch, YamlPrint_switch;
        bool Hmov_switch, HLPF_switch, HBSF_switch, Cmov_switch, CLPF_switch, CBSF_switch;
        bool HaccSwitch = false;
        bool CaccSwitch = false;
    
        /* ROS */
        ros::NodeHandle nh_;
        ros::Publisher ftsensor_pub_, Cftsensor_pub_; // Handle data, Contact data
        ros::Publisher Vive_Force_pub, Vive_Moment_pub, Vive_Acc_pub; 

        ros::ServiceServer Aidin_gui_srv5;

        ros::Publisher aidinGui_statePub;
        std_msgs::String aidinGui_stateMsg;

        geometry_msgs::Wrench pub_data, Cpub_data; // Pub. structure of Handle and Contact 
        NRS_FT_AQ::vive_ft_msg Vive_Force_data;
        NRS_FT_AQ::vive_ft_msg Vive_Moment_data;
        std_msgs::Float64MultiArray Vive_Acc_data;

        /* Mov Filter */
        int Mov_num = 30;
        std::vector<NRS_MovFilter> movF, movM, movCF, movCM; 

        /* LPF Filter */
        double LPF_cutF = 2; // Handle sensor cut-off frequency
        double CLPF_cutF = 10; // Contact sensor cut-off frequency
        std::vector<NRS_FreqFilter> LPF_F, LPF_M, LPF_CF, LPF_CM;

        /* BSF Filter */
        double BSF_cutF = 15; // cut-off frequency, stop frequency(Hz)
        double BSF_BW = 5; // stop frequency width(Hz)
        double CBSF_cutF = 15; // cut-off frequency, stop frequency(Hz)
        double CBSF_BW = 5; // stop frequency width(Hz)
        std::vector<NRS_FreqFilter> BSF_F, BSF_M, BSF_CF, BSF_CM;
        
    
};

#endif