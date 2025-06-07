#ifndef NRS_MSGMONITORING2__MSG_MONITORING_HPP_
#define NRS_MSGMONITORING2__MSG_MONITORING_HPP_

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace nrs_msgmonitoring2
{

class MsgMonitoring
{
public:
  MsgMonitoring(const std::shared_ptr<rclcpp::Node>& node, const std::string& topic_name_);
  ~MsgMonitoring();

  std::shared_ptr<rclcpp::Node> node_;

  std::string topic_name;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mon1_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mon2_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mon3_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mon4_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mon5_pub;

  bool mon1_description_flag = false;
  bool mon2_description_flag = false;
  bool mon3_description_flag = false;
  bool mon4_description_flag = false;
  bool mon5_description_flag = false;

  void Mon1_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);
  void Mon2_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);
  void Mon3_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);
  void Mon4_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);
  void Mon5_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);

  FILE *Mon1_file,*Mon2_file,*Mon3_file,*Mon4_file,*Mon5_file, *Data_description_file;

private:
  // private methods (if any)
};

}  // namespace nrs_msgmonitoring2

#endif  // NRS_MSGMONITORING2__MSG_MONITORING_HPP_













// #include <ros/ros.h>
// #include <iostream>
// #include <std_msgs/Float64MultiArray.h>
// #include <ros/package.h>


// class MsgMonitoring
// {
//     public:
//         MsgMonitoring(ros::NodeHandle &nh, std::string& topic_name_);
//         ~MsgMonitoring();

//         ros::NodeHandle nh;

//         std::string topic_name;

//         ros::Publisher mon1_pub;
//         ros::Publisher mon2_pub;
//         ros::Publisher mon3_pub;
//         ros::Publisher mon4_pub;
//         ros::Publisher mon5_pub;

//         bool mon1_description_flag = false;
//         bool mon2_description_flag = false;
//         bool mon3_description_flag = false;
//         bool mon4_description_flag = false;
//         bool mon5_description_flag = false;

//         void Mon1_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);
//         void Mon2_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);
//         void Mon3_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);
//         void Mon4_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);
//         void Mon5_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile = false);

//         FILE *Mon1_file,*Mon2_file,*Mon3_file,*Mon4_file,*Mon5_file, *Data_description_file;


//     private:

// };