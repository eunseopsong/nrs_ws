#pragma once
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include "Y2Matrix/YMatrix.hpp"
#include "Y2Trajectory/MotionBlender6D.hpp"
#include "Y2Trajectory/MotionBlender9D.hpp"

#define M_PI 3.141592
#define DegreeToRadian(degree) ((degree) * M_PI / 180.0)
#define RadianToDegree(radian) ((radian) * 180.0 / M_PI)

struct PathGenParam {
    double defualt_travelTime = 5.0;        // seconds
    double initialTransferSpeed = 5.0;      // mm/s
    double angularVelocityLimit = 5.0;      // degrees/s
    double accelerationTime = 1.0;          // seconds
    double startingTime = 2.0;              // seconds
    double lastRestingTime = 2.0;           // seconds
    double ptp_target_velocity = 0.5;       // degrees/s
    std::string loadFileType = "cmd_9D";    // cmd_6D, cmd_9D, cmd_continue6D, cmd_continue9D
};

class robot_command
{
    public: 
        robot_command(rclcpp::Node* node, const std::string& robot_name, double control_period, PathGenParam& pg_param_);
        
        // Main functions
        void sendCommand(const std::string& command, const YMatrix& loaded_motion);
        YMatrix PTP_command_input();
        void PTP_command_gen(const YMatrix& loaded_motion);
        void TxtLoad_command_gen(const YMatrix& loaded_motion);
        
        // Public members
        std::vector<double> current_position;
        bool robot_init = false;
        PathGenParam pg_param;
        std::string robot_name_;

    private:

        /* Generate hip instance of node (To share the node pointer) */
        rclcpp::Node* node_;

        // Node parameters
        double control_period_;
        
        // Publishers and Subscribers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_motion_pub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;
        
        // Messages
        std_msgs::msg::String cmd_msg_;
        std_msgs::msg::Float64MultiArray cmd_motion_msg_;
        
        // Timer for control loop
        rclcpp::TimerBase::SharedPtr timer_;
        
        // Callbacks
        void currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

};


