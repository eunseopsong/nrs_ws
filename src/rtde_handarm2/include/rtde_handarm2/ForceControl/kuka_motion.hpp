#pragma once

#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include "Y2Matrix/YMatrix.hpp"
#include "Y2Kinematics/KinematicsKUKAiiwa.hpp"

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <cstdio>
#include "Y2ForceCon/admittance_control.hpp"
#include "Y2ForceCon/nrs_3step_faac.hpp"

/* include fri for session state */ 
#include "friClientIf.h"

/* include lbr_fri_idl */
#include "lbr_fri_idl/msg/lbr_joint_position_command.hpp"

#define M_PI 3.141592
#define DegreeToRadian(degree) ((degree) * M_PI / 180.0)
#define RadianToDegree(radian) ((radian) * 180.0 / M_PI)


class kuka_motion: public KinematicsKUKAiiwa
{
    public:
        kuka_motion(rclcpp::Node::SharedPtr node, const std::string& RB_name, 
            double Control_period, int numOfJoint, const YMatrix& HTMEE2TCP);

        bool jointsReceived() const {
            return current_angles_received;
        }
        void start(bool monitoring_flag_ = true){
            start_flag = true;
            monitoring_flag = monitoring_flag_;
        }

        /* Robot states */
        std::vector<double> current_angles, pre_current_angles, current_angvel;
        std::vector<double> target_angles, pre_target_angles, target_angvel;
        std::vector<double> current_pose, pre_current_pose; // x,y,z,wx,wy,wz
        std::vector<double> current_carvel;
        std::vector<double> target_pose, pre_target_pose; // x,y,z,wx,wy,wz
        std::vector<double> target_carvel;
        std::vector<double> AC_pose; // x,y,z,wx,wy,wz
        YMatrix target_HTM;

        /* FT Sensor States */
        std::vector<double> ft1data;

        /* Control mode */
        std::string control_mode; // Idling, Position, Custom
        std::string pre_control_mode; // Previous control mode for comparison

        /* Force control mode */
        std::string force_con_mode;

        /* Robot name */
        std::string robot_name;

        /* Number of joints */
        unsigned int numOfJoints;

    private:
        /* Node pointer instance */
        rclcpp::Node::SharedPtr node_;

        /* Basic parameters */
        double Control_period_ = 0.008;

        /* Monitoring flag */
        bool monitoring_flag = false;

        /* Start flag */
        bool start_flag = false;

        /* Init flags */
        bool current_angles_received = false;

        /* Mimic mode */
        std::string Mimic_mode; // Master, Slave, None

        /* Admittance control object */
        Yadmittance_control AControl[6];
        std::vector<double> HG_AC_desX; // Hand-guiding Desired Pose
        std::vector<double> FC_AC_desX; // Force-control Desired Pose
        std::vector<double> FC_MASS, FC_DAMPER, FC_STIFFNESS;

        /* Fuzzy Adaptive Admittance Control (Jay's Controller) */
        // Nrs3StepFAAC FAAC3step[3];
        std::unique_ptr<Nrs3StepFAAC> FAAC3step[3];
        bool FAAC_flag[3] = {false,};

        /* Timer callback */
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr minitoring_timer_;
        
        /* ROS message parts */
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
        currentJ_pub, currentP_pub, currentF_pub, targetJ_pub, targetP_pub, targetF_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ctlMode_pub;
        rclcpp::Publisher<lbr_fri_idl::msg::LBRJointPositionCommand>::SharedPtr kuka_targetJ_pub;

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmdMotion_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmdMode_sub;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr JointState_sub;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ftsensor_sub;
        
        std_msgs::msg::Float64MultiArray currentJ_msg;
        std_msgs::msg::Float64MultiArray currentp_msg;
        std_msgs::msg::Float64MultiArray currentF_msg;
        std_msgs::msg::Float64MultiArray targetJ_msg;
        std_msgs::msg::Float64MultiArray targetP_msg;
        std_msgs::msg::Float64MultiArray targetF_msg;

        std_msgs::msg::String ctlMode_msg;
        lbr_fri_idl::msg::LBRJointPositionCommand kuka_targetJ_msg;

        /* Callback function */
        void cmdMotionCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void cmdModeCB(const std_msgs::msg::String::SharedPtr msg);
        void JointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg);
        void ftsensorCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

        /* Joint_state: mapping generation */
        std::vector<std::string> joint_names = {"lbr_A1","lbr_A2","lbr_A3","lbr_A4","lbr_A5","lbr_A6","lbr_A7"};
        std::vector<int> joint_mapping_; // mapping table
        bool mapping_initialized_ = false;
        void initializeJointMapping(const sensor_msgs::msg::JointState::SharedPtr msg);

        /* Control functions */
        void control_idling();
        void control_position();
        void control_guiding();
        void control_force();

        /* Main control loop */
        void main_control();

        void state_update();

        void state_monitoring();

        void state_publisher();
};