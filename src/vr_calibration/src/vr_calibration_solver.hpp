#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <array>
#include <string>
#include <vector>

#include "vr_calibration_common.hpp"

namespace vr_calib {

class VrCalibrationSolver
{
public:
  // Load existing T_CE, T_SA(old) from yaml (with same defaults as original code)
  static void loadExistingYamlConstants(const std::string& calib_yaml_path,
                                       Eigen::Matrix4d& T_CE_out,
                                       Eigen::Matrix4d& T_SA_old_out,
                                       const rclcpp::Logger& logger);

  // Compute T_SA_new using right-multiply logic from original code
  static bool computeTSA_RightMultiply(const std::array<double,6>& cal_pose_xyz_wxyz_rad,
                                       const Eigen::Matrix4d& T_SA_old,
                                       double t_sa_w_des_z_rad,
                                       Eigen::Matrix4d& T_SA_new_out,
                                       const rclcpp::Logger& logger);

  // Finalize: compute T_BC, T_AD_avg and write YAML once (same algorithm/format)
  static void finalizeAndSaveYaml(const std::vector<Eigen::Matrix4d>& T_AB_all,
                                  const std::vector<Eigen::Matrix4d>& T_DC_all,
                                  const Eigen::Matrix3d& R_Adj,
                                  const Eigen::Matrix4d& T_CE,
                                  const Eigen::Matrix4d& T_SA,
                                  const std::string& calib_yaml_path,
                                  double t_sa_w_des_z_rad,
                                  const rclcpp::Logger& logger);

private:
  static bool readMat4(const YAML::Node& n, Eigen::Matrix4d& T);
  static void writeCalibrationYamlAll(const std::string& calib_yaml_path,
                                      double t_sa_w_des_z_rad,
                                      const Eigen::Matrix4d& T_AD,
                                      const Eigen::Matrix4d& T_BC,
                                      const Eigen::Matrix3d& R_Adj,
                                      const Eigen::Matrix4d& T_CE,
                                      const Eigen::Matrix4d& T_SA);
};

} // namespace vr_calib
