#ifndef FT_PROCESSING_H
#define FT_PROCESSING_H

#include <stdio.h>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_srvs/srv/empty.hpp>  // For ROS service - AIDIN GUI SERVER

#include "Gen_filter.hpp"
#include "CAN_reader.hpp"
#include "nrs_ft_aq2/msg/vive_ft_msg.hpp"   // 패키지 이름을 nrs_ft_aq2 로 쓴다고 가정

// ROS1: class FT_processing : public NRS_FTSensor
// ROS2에서도 동일하게 NRS_FTSensor 를 상속
class FT_processing : public NRS_FTSensor
{
public:
    // ROS1: FT_processing(ros::NodeHandle nh, ...)
    // ROS2에서는 node 쉐어드포인터를 받아서 그걸로 publisher/service 만들게 한다
    FT_processing(std::shared_ptr<rclcpp::Node> node,
                  double Ts,
                  unsigned char& HandleID_,
                  unsigned char& ContactID_,
                  bool HaccSwitch_,
                  bool CaccSwitch_);
    ~FT_processing();

    void FT_init(int sen_init_num);
    void FT_filtering();

    void FT_publish();
    void FT_print();
    void FT_record();

    /* Service for sensor init */
    bool SRV5_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                     std::shared_ptr<std_srvs::srv::Empty::Response> res);

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
    
    /* ROS2 */
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr ftsensor_pub_;   // Handle data
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr Cftsensor_pub_;  // Contact data
    rclcpp::Publisher<nrs_ft_aq2::msg::ViveFtMsg>::SharedPtr Vive_Force_pub;
    rclcpp::Publisher<nrs_ft_aq2::msg::ViveFtMsg>::SharedPtr Vive_Moment_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Vive_Acc_pub;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv5;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aidinGui_statePub;
    std_msgs::msg::String aidinGui_stateMsg;

    geometry_msgs::msg::Wrench pub_data, Cpub_data;              // Pub. structure of Handle and Contact 
    nrs_ft_aq2::msg::ViveFtMsg Vive_Force_data;
    nrs_ft_aq2::msg::ViveFtMsg Vive_Moment_data;
    std_msgs::msg::Float64MultiArray Vive_Acc_data;

    /* Mov Filter */
    int Mov_num = 30;
    std::vector<NRS_MovFilter> movF, movM, movCF, movCM; 

    /* LPF Filter */
    double LPF_cutF = 2;     // Handle sensor cut-off frequency
    double CLPF_cutF = 10;   // Contact sensor cut-off frequency
    std::vector<NRS_FreqFilter> LPF_F, LPF_M, LPF_CF, LPF_CM;

    /* BSF Filter */
    double BSF_cutF = 15;    // cut-off frequency, stop frequency(Hz)
    double BSF_BW = 5;       // stop frequency width(Hz)
    double CBSF_cutF = 15;   // cut-off frequency, stop frequency(Hz)
    double CBSF_BW = 5;      // stop frequency width(Hz)
    std::vector<NRS_FreqFilter> BSF_F, BSF_M, BSF_CF, BSF_CM;
};

#endif
