#ifndef NRS_UR10E_CONTROL_H
#define NRS_UR10E_CONTROL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "rtde_handarm/ftsensorMsg.h"
#include "geometry_msgs/PoseStamped.h"
#include <yaml-cpp/yaml.h>

class NRSUR10eControl {
public:
    NRSUR10eControl(double rtde_frequency, int rt_control_priority, int rt_receive_priority);
    ~NRSUR10eControl(); // Destructor to clean up dynamic memory
    void initialize(const std::string& robot_ip);
    void loadYamlConfigurations(const std::string& config_file);
    void runControlLoop();
    void shutdown();
    void getActualQ();
    void handleForceControl();
    void ftDataCallback(const rtde_handarm::ftsensorMsg::ConstPtr& msg);
    void vrDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void modeCmdCallback(const std_msgs::UInt16::ConstPtr& msg);
    void cmdModeCallback(const std_msgs::UInt16::ConstPtr& msg);
    void jointCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void pbIterCallback(const std_msgs::UInt16::ConstPtr& msg);

private:
    ur_rtde::RTDEControlInterface* rtde_control;
    ur_rtde::RTDEReceiveInterface* rtde_receive;
    std::vector<double> joint_q;
    Eigen::VectorXd ftS1, ftS2;
    Eigen::MatrixXd VR_Cali_TAD, VR_Cali_TBC, VR_Cali_TBC_inv, VR_Cali_TBC_PB, VR_Cali_TCE, VR_Cali_RAdj, VR_Cali_TAdj;
    double dt, velocity, acceleration, lookahead_time, gain;
    bool running, stop_flag;
    int mode_cmd;
    ros::Publisher joint_angle_pub, pose_pub;
    ros::Subscriber ft_sub, vr_sub, cmd_mode_sub, joint_cmd_sub, pb_iter_sub;

    // YAML 변수들
    YAML::Node NRS_VR_setting;
    YAML::Node NRS_Fcon_setting;
    YAML::Node NRS_recording;

    void controlRobot();
    void handleForceControlLogic(); // Force compensation logic
};

#endif // NRS_UR10E_CONTROL_H
