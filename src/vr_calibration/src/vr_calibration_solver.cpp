#include "vr_calibration_solver.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

namespace vr_calib {

bool VrCalibrationSolver::readMat4(const YAML::Node& n, Eigen::Matrix4d& T)
{
  if (!n || !n.IsSequence() || n.size() != 4) return false;
  for (int r=0;r<4;r++){
    if (!n[r].IsSequence() || n[r].size() != 4) return false;
    for (int c=0;c<4;c++){
      T(r,c) = n[r][c].as<double>();
    }
  }
  return true;
}

void VrCalibrationSolver::loadExistingYamlConstants(const std::string& calib_yaml_path,
                                                    Eigen::Matrix4d& T_CE_out,
                                                    Eigen::Matrix4d& T_SA_old_out,
                                                    const rclcpp::Logger& logger)
{
  // defaults (same as original)
  T_CE_out = Eigen::Matrix4d::Identity();
  T_CE_out(2,3) = 0.222;

  T_SA_old_out = Eigen::Matrix4d::Identity();

  try {
    YAML::Node existing = YAML::LoadFile(calib_yaml_path);

    if (existing["T_CE"]) {
      Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
      if (readMat4(existing["T_CE"], tmp)) {
        T_CE_out = tmp;
        RCLCPP_INFO(logger, "[YAML] Loaded existing T_CE.");
      }
    }

    if (existing["T_SA"]) {
      Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
      if (readMat4(existing["T_SA"], tmp)) {
        T_SA_old_out = tmp;
        RCLCPP_INFO(logger, "[YAML] Loaded existing T_SA (old).");
      }
    } else {
      RCLCPP_WARN(logger, "[YAML] T_SA not found. Assume Identity (old).");
    }
  } catch (...) {
    RCLCPP_WARN(logger, "[YAML] Cannot load existing yaml. Will create new: %s", calib_yaml_path.c_str());
  }
}

bool VrCalibrationSolver::computeTSA_RightMultiply(const std::array<double,6>& cal_pose_xyz_wxyz_rad,
                                                   const Eigen::Matrix4d& T_SA_old,
                                                   double t_sa_w_des_z_rad,
                                                   Eigen::Matrix4d& T_SA_new_out,
                                                   const rclcpp::Logger& logger)
{
  // w_meas (rad) from /calibrated_pose: [x y z wx wy wz]
  std::array<double,3> w_meas = { cal_pose_xyz_wxyz_rad[3],
                                  cal_pose_xyz_wxyz_rad[4],
                                  cal_pose_xyz_wxyz_rad[5] };

  // R_total from w_meas
  std::array<double,9> Rtot_arr;
  rotvecToRotMatRad(w_meas, Rtot_arr);
  Eigen::Matrix3d R_total;
  R_total << Rtot_arr[0], Rtot_arr[1], Rtot_arr[2],
             Rtot_arr[3], Rtot_arr[4], Rtot_arr[5],
             Rtot_arr[6], Rtot_arr[7], Rtot_arr[8];

  // desired R_des from w_des=[0,0,t_sa_w_des_z]
  std::array<double,3> w_des = {0.0, 0.0, t_sa_w_des_z_rad};
  std::array<double,9> Rdes_arr;
  rotvecToRotMatRad(w_des, Rdes_arr);
  Eigen::Matrix3d R_des;
  R_des << Rdes_arr[0], Rdes_arr[1], Rdes_arr[2],
           Rdes_arr[3], Rdes_arr[4], Rdes_arr[5],
           Rdes_arr[6], Rdes_arr[7], Rdes_arr[8];

  const Eigen::Matrix3d R_SA_old = T_SA_old.block<3,3>(0,0);
  const Eigen::Matrix3d R_SA_new = R_SA_old * R_total.transpose() * R_des;

  T_SA_new_out = Eigen::Matrix4d::Identity();
  T_SA_new_out.block<3,3>(0,0) = R_SA_new;

  RCLCPP_INFO(logger,
    "[T_SA_DONE] Computed T_SA (right-multiply). w_meas(rad)=[%.6f %.6f %.6f]  w_des=[0 0 %.6f]",
    w_meas[0], w_meas[1], w_meas[2], t_sa_w_des_z_rad);

  // sanity: predicted becomes R_des
  Eigen::Matrix3d R_chain = R_total * R_SA_old.transpose();
  Eigen::Matrix3d R_pred  = R_chain * R_SA_new;
  RCLCPP_INFO(logger, "[T_SA_CHECK] trace(R_pred)=%.6f (should be close to trace(R_des)=%.6f)",
              R_pred.trace(), R_des.trace());

  return true;
}

void VrCalibrationSolver::writeCalibrationYamlAll(const std::string& calib_yaml_path,
                                                  double t_sa_w_des_z_rad,
                                                  const Eigen::Matrix4d& T_AD,
                                                  const Eigen::Matrix4d& T_BC,
                                                  const Eigen::Matrix3d& R_Adj,
                                                  const Eigen::Matrix4d& T_CE,
                                                  const Eigen::Matrix4d& T_SA)
{
  std::ofstream ofs(calib_yaml_path, std::ios::out | std::ios::trunc);
  if (!ofs.is_open()) throw std::runtime_error("Failed to open yaml: " + calib_yaml_path);

  const int prec = 12;

  ofs << "# VR calibration matrix setting\n";
  ofs << "# Auto-capture + One-shot YAML update (R_Adj, T_AD, T_BC, T_SA)\n\n";
  ofs << "meta:\n";
  ofs << "  t_sa_w_des_z: " << std::fixed << std::setprecision(prec) << t_sa_w_des_z_rad << "\n";
  ofs << "  note: \"T_SA is right-multiplied in vive_tracker (M_cal = ... @ T_SA)\"\n\n";

  auto writeMat4 = [&](const std::string& key, const Eigen::Matrix4d& T){
    ofs << key << ":\n";
    ofs << std::fixed << std::setprecision(prec);
    for (int r=0;r<4;r++){
      ofs << "  - [";
      for (int c=0;c<4;c++){
        ofs << T(r,c);
        if (c<3) ofs << ", ";
      }
      ofs << "]\n";
    }
    ofs << "\n";
  };

  auto writeMat3 = [&](const std::string& key, const Eigen::Matrix3d& R){
    ofs << key << ":\n";
    ofs << std::fixed << std::setprecision(prec);
    for (int r=0;r<3;r++){
      ofs << "  - [";
      for (int c=0;c<3;c++){
        ofs << R(r,c);
        if (c<2) ofs << ", ";
      }
      ofs << "]\n";
    }
    ofs << "\n";
  };

  writeMat4("T_AD", T_AD);
  writeMat4("T_BC", T_BC);
  writeMat3("R_Adj", R_Adj);

  ofs << "# constant offset: tune here if needed\n";
  writeMat4("T_CE", T_CE);

  ofs << "# spatial-angle frame alignment (right-multiply)\n";
  writeMat4("T_SA", T_SA);

  ofs.flush();
}

void VrCalibrationSolver::finalizeAndSaveYaml(const std::vector<Eigen::Matrix4d>& T_AB_all,
                                              const std::vector<Eigen::Matrix4d>& T_DC_all,
                                              const Eigen::Matrix3d& R_Adj,
                                              const Eigen::Matrix4d& T_CE,
                                              const Eigen::Matrix4d& T_SA,
                                              const std::string& calib_yaml_path,
                                              double t_sa_w_des_z_rad,
                                              const rclcpp::Logger& logger)
{
  const size_t N = T_AB_all.size();
  if (N < 2 || T_DC_all.size() != N) {
    throw std::runtime_error("Not enough samples to compute calibration (need >=2).");
  }

  const size_t K = N - 1;

  Eigen::MatrixXd M(9 * K, 9);
  Eigen::MatrixXd K1(3 * K, 3);
  Eigen::VectorXd K2(3 * K);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  // store for translation solve and T_AD average
  std::vector<Eigen::Vector3d> O_B0B1_list;
  std::vector<Eigen::Vector3d> O_C0C1_list;
  std::vector<Eigen::Matrix4d> T_AB0_list;
  std::vector<Eigen::Matrix4d> T_DC0_list;
  O_B0B1_list.reserve(K);
  O_C0C1_list.reserve(K);
  T_AB0_list.reserve(K);
  T_DC0_list.reserve(K);

  for (size_t i=0; i<K; i++) {
    const Eigen::Matrix4d& T_AB0 = T_AB_all[i];
    const Eigen::Matrix4d& T_AB1 = T_AB_all[i+1];

    const Eigen::Matrix4d& T_DC0 = T_DC_all[i];
    const Eigen::Matrix4d& T_DC1 = T_DC_all[i+1];

    const Eigen::Matrix4d T_B0B1 = invT(T_AB0) * T_AB1;
    const Eigen::Matrix4d T_C0C1 = invT(T_DC0) * T_DC1;

    const Eigen::Matrix3d R_B0B1 = T_B0B1.block<3,3>(0,0);
    const Eigen::Vector3d O_B0B1 = T_B0B1.block<3,1>(0,3);

    const Eigen::Matrix3d R_C0C1 = T_C0C1.block<3,3>(0,0);
    const Eigen::Vector3d O_C0C1 = T_C0C1.block<3,1>(0,3);

    // M block: kron(I,R_B0B1) - kron(R_C0C1^T, I)
    Eigen::Matrix<double,9,9> m = kron3(I, R_B0B1) - kron3(R_C0C1.transpose(), I);
    M.block(9*i, 0, 9, 9) = m;

    // K1 block
    K1.block(3*i, 0, 3, 3) = (I - R_B0B1);

    O_B0B1_list.push_back(O_B0B1);
    O_C0C1_list.push_back(O_C0C1);
    T_AB0_list.push_back(T_AB0);
    T_DC0_list.push_back(T_DC0);
  }

  // Solve for R_BC via smallest eigenvector of X = M^T M
  Eigen::MatrixXd X = M.transpose() * M;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(X);
  if (es.info() != Eigen::Success) throw std::runtime_error("EigenSolver failed on X=M^T*M");

  Eigen::VectorXd vectX = es.eigenvectors().col(0); // smallest eigenvalue
  Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::ColMajor>> R_BC_raw(vectX.data());

  // Orthonormalize with SVD: R = U V^T, det>0
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_BC_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d R_BC = U * V.transpose();
  if (R_BC.determinant() < 0) {
    U.col(2) *= -1.0;
    R_BC = U * V.transpose();
  }

  // Now build K2 and solve O_BC
  for (size_t i=0; i<K; i++) {
    const Eigen::Vector3d& O_B0B1 = O_B0B1_list[i];
    const Eigen::Vector3d& O_C0C1 = O_C0C1_list[i];
    Eigen::Vector3d temp = O_B0B1 - R_BC * O_C0C1;
    K2.segment<3>(3*i) = temp;
  }

  Eigen::Vector3d O_BC = K1.colPivHouseholderQr().solve(K2);
  Eigen::Matrix4d T_BC = makeT(R_BC, O_BC);

  // Compute T_AD_i = T_AB0_i * T_BC * inv(T_DC0_i), i=0..K-1
  std::vector<Eigen::Quaterniond> quats;
  quats.reserve(K);
  Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();

  for (size_t i=0; i<K; i++) {
    const Eigen::Matrix4d& T_AB0 = T_AB0_list[i];
    const Eigen::Matrix4d& T_DC0 = T_DC0_list[i];

    Eigen::Matrix4d T_AD_i = T_AB0 * T_BC * invT(T_DC0);
    Eigen::Matrix3d R = T_AD_i.block<3,3>(0,0);
    Eigen::Vector3d t = T_AD_i.block<3,1>(0,3);

    Eigen::Quaterniond q(R);
    q.normalize();
    quats.push_back(q);
    t_sum += t;
  }

  // Average quaternion with sign alignment (same as original)
  Eigen::Quaterniond q_ref = quats.front();
  Eigen::Vector4d q_sum = Eigen::Vector4d::Zero(); // (x y z w) in coeffs()
  for (auto q : quats) {
    if (q_ref.coeffs().dot(q.coeffs()) < 0) {
      q.coeffs() *= -1.0;
    }
    q_sum += q.coeffs();
  }
  q_sum /= static_cast<double>(quats.size());
  Eigen::Quaterniond q_mean;
  q_mean.coeffs() = q_sum;
  q_mean.normalize();

  Eigen::Vector3d t_mean = t_sum / static_cast<double>(K);
  Eigen::Matrix4d T_AD_avg = makeT(q_mean.toRotationMatrix(), t_mean);

  // One-shot YAML write
  writeCalibrationYamlAll(calib_yaml_path,
                          t_sa_w_des_z_rad,
                          T_AD_avg,
                          T_BC,
                          R_Adj,
                          T_CE,
                          T_SA);

  RCLCPP_INFO(logger, "[YAML_SAVED] T_AD, T_BC, R_Adj, T_CE, T_SA -> %s", calib_yaml_path.c_str());
}

} // namespace vr_calib
