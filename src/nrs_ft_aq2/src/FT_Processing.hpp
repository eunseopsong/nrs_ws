#ifndef FT_PROCESSING_H
#define FT_PROCESSING_H

#include <stdio.h>
#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_srvs/srv/empty.hpp>

#include "nrs_filter_core.hpp"
#include "nrs_filter_applied.hpp"

#include "CAN_reader.hpp"

class FT_processing : public NRS_FTSensor
{
public:
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

    bool SRV5_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                     std::shared_ptr<std_srvs::srv::Empty::Response> res);

    void FT_run();

private:
    std::shared_ptr<rclcpp::Node> node_;
    double Ts_;
    double time_counter = 0;
    FILE *Data1_txt = nullptr;
    bool runnning = true;

    std::string YamlString_IP, YamlData1_path;
    int YamlData1_switch = 0;
    int YamlPrint_switch = 0;
    bool Hmov_switch = false, HLPF_switch = false, HBSF_switch = false;
    bool Cmov_switch = false, CLPF_switch = false, CBSF_switch = false;
    bool HaccSwitch = false;
    bool CaccSwitch = false;

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr ftsensor_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr Cftsensor_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vive_force_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vive_moment_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vive_acc_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv5;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aidinGui_statePub;
    std_msgs::msg::String aidinGui_stateMsg;

    geometry_msgs::msg::Wrench pub_data, Cpub_data;

    int Mov_num = 30;
    std::vector<NRS_MovFilter> movF, movM, movCF, movCM;

    double LPF_cutF = 2;
    double CLPF_cutF = 10;
    std::vector<NRS_FreqFilter> LPF_F, LPF_M, LPF_CF, LPF_CM;

    double BSF_cutF = 15;
    double BSF_BW = 5;
    double CBSF_cutF = 15;
    double CBSF_BW = 5;
    std::vector<NRS_FreqFilter> BSF_F, BSF_M, BSF_CF, BSF_CM;
};

#endif
