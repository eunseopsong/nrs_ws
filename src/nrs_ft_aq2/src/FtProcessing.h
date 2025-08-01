#ifndef FTPROCESSING_H_
#define FTPROCESSING_H_

//// #include <stdio.h>
//// #include <ros/ros.h>
//// #include <std_msgs/String.h>
//// #include <std_msgs/Float64MultiArray.h>
//// #include <geometry_msgs/Wrench.h>
//// #include "NRS_FT_AQ/vive_ft_msg.h"
//// #include <std_srvs/Empty.h> // For ROS service - AIDIN GUI SERVER
//// #include "Gen_filter.hpp"
//// #include "CAN_reader.hpp"

#include <cstdio>  // C 스타일 입출력
#include <memory>  // std::shared_ptr 등 스마트 포인터

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/wrench.hpp"

// #include "nrs_ft_aq2/msg/vive_ft_msg.hpp" // 패키지명/msg/메시지명.hpp

#include "std_srvs/srv/empty.hpp"  // 서비스는 srv 디렉토리로

#include "nrs_gen_filter2/Gen_filter.hpp"
#include "nrs_ft_aq2/CAN_reader.hpp"

class FtProcessing : public rclcpp::Node, public NrsFtSensor
{
public:
    //// FTProcessing(ros::NodeHandle nh, double Ts, unsigned char& HandleID_, unsigned char& ContactID_, bool HaccSwitch_, bool CaccSwitch_);
    //// ~FTProcessing();
    FtProcessing(double Ts, unsigned char& HandleID_, unsigned char& ContactID_, bool HaccSwitch_, bool CaccSwitch_);
    ~FtProcessing();

    void FT_init(int sen_init_num);
    void FT_filtering();
    void FT_publish();
    void FT_print();
    void FT_record();
    void FT_run();

private:
    /* Service for sensor init */
    //// bool SRV5_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void SRV5_Handle(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    /* Common parameters */
    double Ts_;
    double time_counter = 0.0;
    FILE* Data1_txt = nullptr;  //// FILE *Data1_txt;
    bool runnning = true;

    /* Yaml */
    std::string YamlString_IP, YamlData1_path;
    //// int YamlData1_switch, YamlPrint_switch;
    int YamlData1_switch = 0;
    int YamlPrint_switch = 0;
    //// bool Hmov_switch, HLPF_switch, HBSF_switch, Cmov_switch, CLPF_switch, CBSF_switch;
    bool Hmov_switch = false, HLPF_switch = false, HBSF_switch = false;
    bool Cmov_switch = false, CLPF_switch = false, CBSF_switch = false;
    bool HaccSwitch = false;
    bool CaccSwitch = false;

    /* ROS2 Publisher*/
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr ftsensor_pub_;  // Handle data
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr Cftsensor_pub_; // Contact data
    rclcpp::Publisher<nrs_ft_aq2::msg::ViveFTMsg>::SharedPtr Vive_Force_pub;
    rclcpp::Publisher<nrs_ft_aq2::msg::ViveFTMsg>::SharedPtr Vive_Moment_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Vive_Acc_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aidinGui_statePub;

    /* ROS2 Service*/
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv5;

    // 메시지 버퍼
    std_msgs::msg::String aidinGui_stateMsg;
    geometry_msgs::msg::Wrench pub_data, Cpub_data;
    nrs_ft_aq2::msg::ViveFTMsg Vive_Force_data;
    nrs_ft_aq2::msg::ViveFTMsg Vive_Moment_data;
    std_msgs::msg::Float64MultiArray Vive_Acc_data;

    //// geometry_msgs::Wrench pub_data, Cpub_data; // Pub. structure of Handle and Contact 
    //// NRS_FT_AQ::vive_ft_msg Vive_Force_data;
    //// NRS_FT_AQ::vive_ft_msg Vive_Moment_data;
    //// std_msgs::Float64MultiArray Vive_Acc_data;

    /* Mov Filter */
    int Mov_num = 30;
    std::vector<NRS_MovFilter> movF, movM, movCF, movCM;

    /* LPF Filter */
    double LPF_cutF = 2.0;
    double CLPF_cutF = 10.0;
    std::vector<NRS_FreqFilter> LPF_F, LPF_M, LPF_CF, LPF_CM;

    /* BSF Filter */
    double BSF_cutF = 15.0;
    double BSF_BW = 5.0;
    double CBSF_cutF = 15.0;
    double CBSF_BW = 5.0;
    std::vector<NRS_FreqFilter> BSF_F, BSF_M, BSF_CF, BSF_CM;

    // 센서값 버퍼
    double Force_val[3]{};
    double Moment_val[3]{};
    double Contact_Force_val[3]{};
    double Contact_Moment_val[3]{};
    double Pos_acc_val[3]{};
    double Ang_acc_val[3]{};
    double Ang_vel_val[3]{};
    double CPos_acc_val[3]{};
    double CAng_acc_val[3]{};
    double CAng_vel_val[3]{};

    // 센서 제어용 변수
    int init_average_num = 0;
    int sensor_init_counter = 0;
    double CAN_sampling = 0.0;
};

#endif  // FTPROCESSING_H_