#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <vector>

class VrCalibrationSolver
{
public:
  VrCalibrationSolver();

  void setYamlPath(const std::string& path);
  void setTSADesiredZ(double w_des_z);
  void setTSASide(const std::string& side); // "left" or "right"

  void loadExistingYamlConstants();

  void resetSamples();
  void resetRAdj();

  void pushSample(const Eigen::Matrix4d& T_AB, const Eigen::Matrix4d& T_DC);
  void feedVRQuaternionForRAdj(const Eigen::Quaterniond& q_vr);

  bool haveRAdj() const;

  // T_SA: computed ONLY from calibrated_pose rotation R_total (from wx wy wz rotvec(rad))
  Eigen::Matrix4d computeTSAFromCalibratedPoseRotation(const Eigen::Matrix3d& R_total) const;

  // finalize: solve T_BC, T_AD, compute/keep R_Adj, save YAML once
  void finalizeAndSave(bool t_sa_computed, const Eigen::Matrix4d& T_SA_new);

private:
  Eigen::Matrix4d invT(const Eigen::Matrix4d& T) const;
  Eigen::Matrix<double,9,9> kron3(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) const;

  void writeCalibrationYamlAll(const Eigen::Matrix4d& T_AD,
                              const Eigen::Matrix4d& T_BC,
                              const Eigen::Matrix3d& R_Adj,
                              const Eigen::Matrix4d& T_CE,
                              const Eigen::Matrix4d& T_SA);

  // YAML constants
  std::string calib_yaml_path_;
  double t_sa_w_des_z_ = 1.57079632679;
  std::string t_sa_side_ = "right";

  Eigen::Matrix4d T_CE_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_SA_old_ = Eigen::Matrix4d::Identity();

  // samples
  std::vector<Eigen::Matrix4d> T_AB_all_;
  std::vector<Eigen::Matrix4d> T_DC_all_;

  // R_Adj (optional)
  bool have_radj_ = false;
  Eigen::Matrix3d R_adj_ = Eigen::Matrix3d::Identity();
  std::vector<Eigen::Quaterniond> vr_quats_;
};
