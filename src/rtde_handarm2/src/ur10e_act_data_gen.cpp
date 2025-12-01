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
#include <cmath>
#include <algorithm>

namespace fs = std::filesystem;

class ActDataRecorder : public rclcpp::Node
{
public:
  ActDataRecorder()
  : Node("act_data_recorder"),
    recording_(false),
    saving_(false)
  {
    // 1) subscribers
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/isaac_joint_states",
      10,
      std::bind(&ActDataRecorder::jointCallback, this, std::placeholders::_1));

    // front camera
    image_front_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/front_camera/rgb",
      10,
      std::bind(&ActDataRecorder::frontImageCallback, this, std::placeholders::_1));

    // top camera
    image_top_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/top_camera/rgb",
      10,
      std::bind(&ActDataRecorder::topImageCallback, this, std::placeholders::_1));

    // FT 센서
    ft_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "/ftsensor/measured_Cvalue",
      10,
      std::bind(&ActDataRecorder::ftCallback, this, std::placeholders::_1));

    // UR10e joint names
    ur10e_joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    // 저장 경로: 시간 포함
    auto t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%m%d_%H%M", &tm);
    base_dir_ = "/home/eunseop/nrs_lab2/datasets/ACT/" + std::string(buf);
    fs::create_directories(base_dir_);
    hdf5_path_ = base_dir_ + "/act_data.hdf5";

    RCLCPP_INFO(this->get_logger(), "Saving to: %s", hdf5_path_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Episode rule: start=|fx|>=10, end=|fy|>=10");
    RCLCPP_INFO(this->get_logger(),
                "Shutdown rule: |fx|>=20 && |fy|>=20");
    RCLCPP_INFO(this->get_logger(),
                "Cameras: /front_camera/rgb -> image_front, /top_camera/rgb -> image_top");

    // main loop 20Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ActDataRecorder::mainLoop, this));
  }

private:
  // ================== callbacks ==================
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

  void frontImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_image_front_ = cv_ptr->image.clone();
      image_front_received_ = true;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge (front) error: %s", e.what());
    }
  }

  void topImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_image_top_ = cv_ptr->image.clone();
      image_top_received_ = true;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge (top) error: %s", e.what());
    }
  }

  void ftCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
  {
    double fx = msg->force.x;
    double fy = msg->force.y;
    double fz = msg->force.z;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_ft_raw_[0] = static_cast<float>(fx);
      latest_ft_raw_[1] = static_cast<float>(fy);
      latest_ft_raw_[2] = static_cast<float>(fz);
      ft_received_ = true;
    }

    bool abs_fx_over_start = std::fabs(fx) >= 10.0;
    bool abs_fy_over_end   = std::fabs(fy) >= 10.0;
    bool abs_fx_over_shutdown = std::fabs(fx) >= 20.0;
    bool abs_fy_over_shutdown = std::fabs(fy) >= 20.0;

    // 3) 노드 종료 조건: fx, fy 둘 다 20 이상
    if (abs_fx_over_shutdown && abs_fy_over_shutdown) {
      RCLCPP_WARN(this->get_logger(),
                  "FX=%.3f FY=%.3f -> shutdown condition (20/20) met.", fx, fy);
      if (recording_) {
        stopRecordingAndSave();
      }
      rclcpp::shutdown();
      return;
    }

    // 1) 에피소드 시작: |fx| >= 10, 현재 녹화중 아님
    if (abs_fx_over_start && !recording_) {
      startRecording();
      return;
    }

    // 2) 에피소드 끝: |fy| >= 10, 현재 녹화중
    if (abs_fy_over_end && recording_) {
      stopRecordingAndSave();
      return;
    }
  }

  // ================== main loop ==================
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

    // ft: fx=0, fy=0, fz 그대로
    std::array<float,3> ft_to_save{0.f, 0.f, 0.f};
    if (ft_received_) {
      ft_to_save[2] = latest_ft_raw_[2];
    }
    buffer_fts_.push_back(ft_to_save);

    // front image
    if (image_front_received_) {
      buffer_images_front_.push_back(latest_image_front_.clone());
    }
    // top image
    if (image_top_received_) {
      buffer_images_top_.push_back(latest_image_top_.clone());
    }
  }

  // ================== episode control ==================
  void startRecording()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (recording_) return;
    recording_ = true;
    buffer_joints_.clear();
    buffer_fts_.clear();
    buffer_images_front_.clear();
    buffer_images_top_.clear();
    RCLCPP_INFO(this->get_logger(), "=== EPISODE STARTED (by |fx| >= 10) ===");
  }

  void stopRecordingAndSave()
  {
    if (saving_) return;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!recording_) return;
      recording_ = false;
      saving_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "=== EPISODE ENDED (by |fy| >= 10) ===");
    saveToHDF5();

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      buffer_joints_.clear();
      buffer_fts_.clear();
      buffer_images_front_.clear();
      buffer_images_top_.clear();
      saving_ = false;
    }
  }

  // ================== HDF5 저장 ==================
  void saveToHDF5()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (buffer_joints_.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Not enough joint data to form (t, t+1) pairs.");
      return;
    }

    // 각 버퍼 길이
    std::size_t N_joint = buffer_joints_.size();
    std::size_t N_ft    = buffer_fts_.size();
    std::size_t N_front = buffer_images_front_.size();
    std::size_t N_top   = buffer_images_top_.size();

    std::size_t N_min = std::min(
      std::min(N_joint, N_ft),
      std::min(N_front, N_top)
    );

    if (N_min < 2) {
      RCLCPP_WARN(this->get_logger(),
                  "Not enough synchronized data (min buffer size = %zu).", N_min);
      return;
    }

    // (obs_t, action_t) 쌍 개수: t=0..N_pair-1 에 대해 joints_t / joints_{t+1}
    std::size_t N_pair = N_min - 1;

    // 이미지 크기 (front 기준)
    int H_front = buffer_images_front_[0].rows;
    int W_front = buffer_images_front_[0].cols;
    int C_front = buffer_images_front_[0].channels();

    int H_top = buffer_images_top_[0].rows;
    int W_top = buffer_images_top_[0].cols;
    int C_top = buffer_images_top_[0].channels();

    // 파일 열기/생성
    hid_t file_id;
    if (fs::exists(hdf5_path_)) {
      file_id = H5Fopen(hdf5_path_.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    } else {
      file_id = H5Fcreate(hdf5_path_.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    }
    if (file_id < 0) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open/create HDF5 file");
      return;
    }

    // /data 그룹
    hid_t data_group_id;
    if (H5Lexists(file_id, "/data", H5P_DEFAULT) > 0) {
      data_group_id = H5Gopen(file_id, "/data", H5P_DEFAULT);
    } else {
      data_group_id = H5Gcreate(file_id, "/data", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    }

    // demo 이름
    hsize_t nobj;
    H5Gget_num_objs(data_group_id, &nobj);
    std::string demo_name = "demo_" + std::to_string(nobj);
    hid_t demo_group_id = H5Gcreate(data_group_id, demo_name.c_str(),
                                    H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    // =======================
    // 1) obs_joints: (N_pair, 6)
    //    obs_joints[t] = joints_t
    // =======================
    {
      hsize_t dims[2] = { static_cast<hsize_t>(N_pair), 6 };
      hid_t space_id = H5Screate_simple(2, dims, nullptr);
      hid_t dset_id = H5Dcreate(demo_group_id, "obs_joints", H5T_NATIVE_DOUBLE, space_id,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      std::vector<double> flat;
      flat.reserve(N_pair * 6);
      for (std::size_t t = 0; t < N_pair; ++t) {
        const auto &q_t = buffer_joints_[t];
        for (double v : q_t) flat.push_back(v);
      }

      H5Dwrite(dset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
      H5Dclose(dset_id);
      H5Sclose(space_id);
    }

    // =======================
    // 2) action_joints: (N_pair, 6)
    //    action_joints[t] = joints_{t+1}
    // =======================
    {
      hsize_t dims[2] = { static_cast<hsize_t>(N_pair), 6 };
      hid_t space_id = H5Screate_simple(2, dims, nullptr);
      hid_t dset_id = H5Dcreate(demo_group_id, "action_joints", H5T_NATIVE_DOUBLE, space_id,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      std::vector<double> flat;
      flat.reserve(N_pair * 6);
      for (std::size_t t = 0; t < N_pair; ++t) {
        const auto &q_tp1 = buffer_joints_[t + 1];
        for (double v : q_tp1) flat.push_back(v);
      }

      H5Dwrite(dset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
      H5Dclose(dset_id);
      H5Sclose(space_id);
    }

    // =======================
    // 3) action_ft: (N_pair, 3)
    //    action_ft[t] = [0, 0, fz_{t+1}]
    // =======================
    {
      hsize_t dims[2] = { static_cast<hsize_t>(N_pair), 3 };
      hid_t space_id = H5Screate_simple(2, dims, nullptr);
      hid_t dset_id = H5Dcreate(demo_group_id, "action_ft", H5T_NATIVE_FLOAT, space_id,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      std::vector<float> flat;
      flat.reserve(N_pair * 3);
      for (std::size_t t = 0; t < N_pair; ++t) {
        const auto &ft_tp1 = buffer_fts_[t + 1];  // [0, 0, fz]
        for (float v : ft_tp1) flat.push_back(v);
      }

      H5Dwrite(dset_id, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
      H5Dclose(dset_id);
      H5Sclose(space_id);
    }

    // =======================
    // 4) obs_image_front: (N_pair, H, W, C)
    //    obs_image_front[t] = image_front_t
    // =======================
    if (!buffer_images_front_.empty()) {
      hsize_t dims[4] = {
        static_cast<hsize_t>(N_pair),
        static_cast<hsize_t>(H_front),
        static_cast<hsize_t>(W_front),
        static_cast<hsize_t>(C_front)
      };
      hid_t space_id = H5Screate_simple(4, dims, nullptr);
      hid_t dset_id = H5Dcreate(demo_group_id, "obs_image_front", H5T_NATIVE_UCHAR, space_id,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      std::vector<unsigned char> flat;
      flat.reserve(N_pair * H_front * W_front * C_front);
      for (std::size_t t = 0; t < N_pair; ++t) {
        const auto &img = buffer_images_front_[t];
        flat.insert(flat.end(), img.data, img.data + (H_front * W_front * C_front));
      }

      H5Dwrite(dset_id, H5T_NATIVE_UCHAR, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
      H5Dclose(dset_id);
      H5Sclose(space_id);
    }

    // =======================
    // 5) obs_image_top: (N_pair, H, W, C)
    //    obs_image_top[t] = image_top_t
    // =======================
    if (!buffer_images_top_.empty()) {
      hsize_t dims[4] = {
        static_cast<hsize_t>(N_pair),
        static_cast<hsize_t>(H_top),
        static_cast<hsize_t>(W_top),
        static_cast<hsize_t>(C_top)
      };
      hid_t space_id = H5Screate_simple(4, dims, nullptr);
      hid_t dset_id = H5Dcreate(demo_group_id, "obs_image_top", H5T_NATIVE_UCHAR, space_id,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      std::vector<unsigned char> flat;
      flat.reserve(N_pair * H_top * W_top * C_top);
      for (std::size_t t = 0; t < N_pair; ++t) {
        const auto &img = buffer_images_top_[t];
        flat.insert(flat.end(), img.data, img.data + (H_top * W_top * C_top));
      }

      H5Dwrite(dset_id, H5T_NATIVE_UCHAR, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat.data());
      H5Dclose(dset_id);
      H5Sclose(space_id);
    }

    H5Gclose(demo_group_id);
    H5Gclose(data_group_id);
    H5Fclose(file_id);

    RCLCPP_INFO(this->get_logger(),
                "Saved %s (pairs: %zu, joints_len=%zu, ft_len=%zu, front_len=%zu, top_len=%zu)",
                demo_name.c_str(),
                N_pair,
                N_joint,
                N_ft,
                N_front,
                N_top);
  }

private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_top_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr   ft_sub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  // latest data
  std::array<double, 6> latest_joint_{};
  std::array<float, 3>  latest_ft_raw_{};

  cv::Mat latest_image_front_;
  cv::Mat latest_image_top_;

  bool joint_received_{false};
  bool ft_received_{false};
  bool image_front_received_{false};
  bool image_top_received_{false};

  // buffers
  std::vector<std::array<double,6>> buffer_joints_;
  std::vector<std::array<float,3>>  buffer_fts_;
  std::vector<cv::Mat>              buffer_images_front_;
  std::vector<cv::Mat>              buffer_images_top_;

  // state
  bool recording_;
  bool saving_;
  std::mutex data_mutex_;

  // joint names
  std::vector<std::string> ur10e_joint_names_;

  // save path
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
