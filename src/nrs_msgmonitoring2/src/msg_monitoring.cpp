#include "nrs_msgmonitoring2/msg_monitoring.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <filesystem>
#include <cstdlib>  // system()
#include <fstream>

namespace fs = std::filesystem;

namespace nrs_msgmonitoring2
{

MsgMonitoring::MsgMonitoring(const std::shared_ptr<rclcpp::Node>& node, const std::string& topic_name_)
: node_(node), topic_name(topic_name_)
{
  mon1_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name + "/mon1", 10);
  mon2_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name + "/mon2", 10);
  mon3_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name + "/mon3", 10);
  mon4_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name + "/mon4", 10);
  mon5_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name + "/mon5", 10);

  std::string package_path = std::string(std::getenv("HOME")) + "/nrs_ws/src/nrs_msgmonitoring2";  // 경로 임시 지정

  std::string clean_topic = topic_name;
  if (!clean_topic.empty() && clean_topic[0] == '/') {
    clean_topic = clean_topic.substr(1);
  }

  std::string dir_path = package_path + "/recording/" + clean_topic;
  fs::create_directories(dir_path);

  Mon1_file = fopen((dir_path + "/mon1.txt").c_str(), "w");
  Mon2_file = fopen((dir_path + "/mon2.txt").c_str(), "w");
  Mon3_file = fopen((dir_path + "/mon3.txt").c_str(), "w");
  Mon4_file = fopen((dir_path + "/mon4.txt").c_str(), "w");
  Mon5_file = fopen((dir_path + "/mon5.txt").c_str(), "w");
  Data_description_file = fopen((dir_path + "/Data_descriptions.txt").c_str(), "w");
}

MsgMonitoring::~MsgMonitoring()
{
  fclose(Mon1_file);
  fclose(Mon2_file);
  fclose(Mon3_file);
  fclose(Mon4_file);
  fclose(Mon5_file);
  fclose(Data_description_file);
}

void MsgMonitoring::Mon1_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.clear();

  if (!mon1_description_flag) {
    fprintf(Data_description_file, "Mon1: %s\n", data_description.c_str());
    mon1_description_flag = true;
  }

  for (const auto& data : input_data) {
    msg.data.push_back(data);
    if (writeToFile) fprintf(Mon1_file, "%f ", data);
  }
  if (writeToFile) fprintf(Mon1_file, "\n");

  mon1_pub->publish(msg);
}

void MsgMonitoring::Mon2_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.clear();

  if (!mon2_description_flag) {
    fprintf(Data_description_file, "Mon2: %s\n", data_description.c_str());
    mon2_description_flag = true;
  }

  for (const auto& data : input_data) {
    msg.data.push_back(data);
    if (writeToFile) fprintf(Mon2_file, "%f ", data);
  }
  if (writeToFile) fprintf(Mon2_file, "\n");

  mon2_pub->publish(msg);
}

void MsgMonitoring::Mon3_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.clear();

  if (!mon3_description_flag) {
    fprintf(Data_description_file, "Mon3: %s\n", data_description.c_str());
    mon3_description_flag = true;
  }

  for (const auto& data : input_data) {
    msg.data.push_back(data);
    if (writeToFile) fprintf(Mon3_file, "%f ", data);
  }
  if (writeToFile) fprintf(Mon3_file, "\n");

  mon3_pub->publish(msg);
}

void MsgMonitoring::Mon4_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.clear();

  if (!mon4_description_flag) {
    fprintf(Data_description_file, "Mon4: %s\n", data_description.c_str());
    mon4_description_flag = true;
  }

  for (const auto& data : input_data) {
    msg.data.push_back(data);
    if (writeToFile) fprintf(Mon4_file, "%f ", data);
  }
  if (writeToFile) fprintf(Mon4_file, "\n");

  mon4_pub->publish(msg);
}

void MsgMonitoring::Mon5_publish(const std::vector<double>& input_data, std::string& data_description, const bool writeToFile)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.clear();

  if (!mon5_description_flag) {
    fprintf(Data_description_file, "Mon5: %s\n", data_description.c_str());
    mon5_description_flag = true;
  }

  for (const auto& data : input_data) {
    msg.data.push_back(data);
    if (writeToFile) fprintf(Mon5_file, "%f ", data);
  }
  if (writeToFile) fprintf(Mon5_file, "\n");

  mon5_pub->publish(msg);
}

}  // namespace nrs_msgmonitoring2
