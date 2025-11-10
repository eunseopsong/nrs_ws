#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <hdf5.h>
#include <H5public.h>

#include <array>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <filesystem>
#include <ctime>
#include <iostream>

namespace fs = std::filesystem;

class ActDataRecorder : public rclcpp::Node
{
public:
  ActDataRecorder()
  : Node("act_data_recorder"),
    recording_(false),
    waiting_for_save_decision_(false)
  {
    // 1) subscribers
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/isaac_joint_states",
      10,
      std::bind(&ActDataRecorder::jointCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/rgb",
      10,
      std::bind(&ActDataRecorder::imageCallback, this, std::placeholders::_1));

    // ★ 여기! 퍼블리셔가 내보내는 이름과 동일하게, 소문자 v
    ft_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "/ftsensor/measured_Cvalue",
      10,
      std::bind(&ActDataRecorder::ftCallback, this, std::placeholders::_1));

    ur10e_joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    // 경로: /home/eunseop/nrs_ws/src/rtde_handarm2/data/{mmdd_HHMM}/act_data.hdf5
    auto t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%m%d_%H%M", &tm);
    base_dir_ = "/home/eunseop/nrs_ws/src/rtde_handarm2/data/" + std::string(buf);
    fs::create_directories(base_dir_);
    hdf5_path_ = base_dir_ + "/act_data.hdf5";

    RCLCPP_INFO(this->get_logger(), "Saving to: %s", hdf5_path_.c_str());

    // 키보드 스레드
    keyboard_thread_ = std::thread(&ActDataRecorder::keyboardLoop, this);

    // 20Hz 루프
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ActDataRecorder::mainLoop, this));
  }

  ~ActDataRecorder()
  {
    run_keyboard_loop_ = false;
    if (keyboard_thread_.joinable())
      keyboard_thread_.join();
  }

private:
  // ===== callbacks =====
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_joint_.fill(0.0);

    for (std::size_t i = 0; i < ur10e_joint_names_.size(); ++i) {
      for (std::size_t j = 0; j < msg->name.size(); ++j) {
        if (msg->name[j] == ur10e_joint_names_[i]) {
          latest_joint_[i] = msg->position[j];
          break;
        }
      }
    }
    joint_received_ = true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_image_ = cv_ptr->image.clone();
      image_received_ = true;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
  }

  void ftCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_ft_[0] = static_cast<float>(msg->force.x);
    latest_ft_[1] = static_cast<float>(msg->force.y);
    latest_ft_[2] = static_cast<float>(msg->force.z);
    ft_received_ = true;
  }

  // ===== main loop =====
  void mainLoop()
  {
    if (!recording_) return;

    std::lock_guard<std::mutex> lock(data_mutex_);

    // joints
    if (joint_received_) {
      buffer_joints_.push_back(latest_joint_);
    } else {
      buffer_joints_.push_back(std::array<double,6>{0,0,0,0,0,0});
    }

    // ft
    if (ft_received_) {
      buffer_fts_.push_back(latest_ft_);
    } else {
      buffer_fts_.push_back(std::array<float,3>{0.f,0.f,0.f});
    }

    // image
    if (image_received_) {
      buffer_images_.push_back(latest_image_.clone());
    }
  }

  // ===== keyboard =====
  void keyboardLoop()
  {
    while (run_keyboard_loop_) {
      char c;
      std::cin >> c;
      if (!run_keyboard_loop_) break;

      if (c == 's') {
        if (!waiting_for_save_decision_) startRecording();
      } else if (c == 'q') {
        if (recording_ && !waiting_for_save_decision_) stopRecordingAndAskSave();
      } else if (c == 't') {
        rclcpp::shutdown();
        break;
      }
    }
  }

  void startRecording()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    recording_ = true;
    waiting_for_save_decision_ = false;
    buffer_joints_.clear();
    buffer_fts_.clear();
    buffer_images_.clear();
    RCLCPP_INFO(this->get_logger(), "Recording started.");
  }

  void stopRecordingAndAskSave()
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      recording_ = false;
      waiting_for_save_decision_ = true;
    }
    RCLCPP_INFO(this->get_logger(), "Stop. Save? (y/n)");
    char ans;
    std::cin >> ans;
    if (ans == 'y' || ans == 'Y') {
      saveToHDF5();
    } else {
      RCLCPP_INFO(this->get_logger(), "Data discarded.");
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      waiting_for_save_decision_ = false;
      buffer_joints_.clear();
      buffer_fts_.clear();
      buffer_images_.clear();
    }
  }

  // ===== HDF5 저장 =====
  void saveToHDF5()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (buffer_joints_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No data to save.");
      return;
    }

    hid_t file_id;
    if (fs::exists(hdf5_path_)) {
      file_id = H5Fopen(hdf5_path_.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    } else {
      file_id = H5Fcreate(hdf5_path_.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    }
    if (file_id < 0) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open/create HDF5");
      return;
    }

    hid_t data_group_id;
    if (H5Lexists(file_id, "/data", H5P_DEFAULT) > 0) {
      data_group_id = H5Gopen(file_id, "/data", H5P_DEFAULT);
    } else {
      data_group_id = H5Gcreate(file_id, "/data", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    }

    hsize_t nobj;
    H5Gget_num_objs(data_group_id, &nobj);
    std::string demo_name = "demo_" + std::to_string(nobj);
    hid_t demo_group_id = H5Gcreate(data_group_id, demo_name.c_str(),
                                    H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    // joints
    {
      hsize_t N = buffer_joints_.size();
      hsize_t dims[2] = {N, 6};
      hid_t space_id = H5Screate_simple(2, dims, nullptr);
      hid_t dset_id = H5Dcreate(demo_group_id, "joints", H5T_NATIVE_DOUBLE, space_id,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      std::vector<double> flat;
      flat.reserve(N * 6);
      for (auto &a : buffer_joints_) {
        for (double v : a) flat.push_back(v);
      }

      H5Dwrite(dset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
      H5Dclose(dset_id);
      H5Sclose(space_id);
    }

    // ft
    {
      hsize_t N = buffer_fts_.size();
      hsize_t dims[2] = {N, 3};
      hid_t space_id = H5Screate_simple(2, dims, nullptr);
      hid_t dset_id = H5Dcreate(demo_group_id, "ft", H5T_NATIVE_FLOAT, space_id,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      std::vector<float> flat;
      flat.reserve(N * 3);
      for (auto &a : buffer_fts_) {
        for (float v : a) flat.push_back(v);
      }

      H5Dwrite(dset_id, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
      H5Dclose(dset_id);
      H5Sclose(space_id);
    }

    // image
    if (!buffer_images_.empty()) {
      int N = static_cast<int>(buffer_images_.size());
      int H = buffer_images_[0].rows;
      int W = buffer_images_[0].cols;
      int C = buffer_images_[0].channels();

      hsize_t dims[4] = {
        static_cast<hsize_t>(N),
        static_cast<hsize_t>(H),
        static_cast<hsize_t>(W),
        static_cast<hsize_t>(C)
      };
      hid_t space_id = H5Screate_simple(4, dims, nullptr);
      hid_t dset_id = H5Dcreate(demo_group_id, "image", H5T_NATIVE_UCHAR, space_id,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      std::vector<unsigned char> flat;
      flat.reserve(N * H * W * C);
      for (auto &img : buffer_images_) {
        flat.insert(flat.end(), img.data, img.data + (H * W * C));
      }

      H5Dwrite(dset_id, H5T_NATIVE_UCHAR, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
      H5Dclose(dset_id);
      H5Sclose(space_id);
    }

    H5Gclose(demo_group_id);
    H5Gclose(data_group_id);
    H5Fclose(file_id);

    RCLCPP_INFO(this->get_logger(),
                "Saved demo to %s (%zu joints, %zu ft, %zu images)",
                hdf5_path_.c_str(),
                buffer_joints_.size(),
                buffer_fts_.size(),
                buffer_images_.size());
  }

private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr   ft_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // latest
  std::array<double, 6> latest_joint_{};
  std::array<float, 3>  latest_ft_{};
  cv::Mat latest_image_;
  bool joint_received_{false};
  bool ft_received_{false};
  bool image_received_{false};

  // buffer
  std::vector<std::array<double,6>> buffer_joints_;
  std::vector<std::array<float,3>>  buffer_fts_;
  std::vector<cv::Mat>              buffer_images_;

  // flags
  bool recording_;
  bool waiting_for_save_decision_;
  std::mutex data_mutex_;

  // keyboard
  std::thread keyboard_thread_;
  bool run_keyboard_loop_{true};

  // joint names
  std::vector<std::string> ur10e_joint_names_;

  // save paths
  std::string base_dir_;
  std::string hdf5_path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActDataRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
