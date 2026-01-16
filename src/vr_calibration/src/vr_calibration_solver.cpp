#include "vr_calibration_solver.hpp"
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>

VrCalibrationSolver::VrCalibrationSolver() {}

void VrCalibrationSolver::setYamlPath(const std::string& path)
{
  calib_yaml_path_ = path;
}

void VrCalibrationSolver::setTSADesiredZ(double w_des_z)
{
  t_sa_w_des_z_ = w_des_z;
}

void VrCalibrationSolver::setTSASide(const std::string& side)
{
  t_sa_side_ = side;
  std::string s = t_sa_side_;
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  if (s != "left" && s != "right") {
    throw std::runtime_error("t_sa_side must be 'left' or 'right'");
  }
  t_sa_side_ = s;
}

static bool readMat4Node(const YAML::Node& n, Eigen::Matrix4d& T)
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

void VrCalibrationSolver::loadExistingYamlConstants()
{
  // defaults
  T_CE_ = Eigen::Matrix4d::Identity();
  T_CE_(2,3) = 0.222;
  T_SA_old_ = Eigen::Matrix4d::Identity();

  try {
    YAML::Node existing = YAML::LoadFile(calib_yaml_path_);

    if (existing["T_CE"]) {
      Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
      if (readMat4Node(existing["T_CE"], tmp)) T_CE_ = tmp;
    }
    if (existing["T_SA"]) {
      Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
      if (readMat4Node(existing["T_SA"], tmp)) T_SA_old_ = tmp;
    }
  } catch (...) {
    // keep defaults
  }
}

void VrCalibrationSolver::resetSamples()
{
  T_AB_all_.clear();
  T_DC_all_.clear();
  vr_quats_.clear();
}

void VrCalibrationSolver::resetRAdj()
{
  have_radj_ = false;
  R_adj_.setIdentity();
}

void VrCalibrationSolver::pushSample(const Eigen::Matrix4d& T_AB, const Eigen::Matrix4d& T_DC)
{
  T_AB_all_.push_back(T_AB);
  T_DC_all_.push_back(T_DC);
}

void VrCalibrationSolver::feedVRQuaternionForRAdj(const Eigen::Quaterniond& q_vr)
{
  Eigen::Quaterniond q = q_vr;
  q.normalize();
  vr_quats_.push_back(q);
}

bool VrCalibrationSolver::haveRAdj() const { return have_radj_; }

Eigen::Matrix4d VrCalibrationSolver::invT(const Eigen::Matrix4d& T) const
{
  Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R = T.block<3,3>(0,0);
  const Eigen::Vector3d p = T.block<3,1>(0,3);
  Ti.block<3,3>(0,0) = R.transpose();
  Ti.block<3,1>(0,3) = -R.transpose()*p;
  return Ti;
}

Eigen::Matrix<double,9,9> VrCalibrationSolver::kron3(const Eigen::Matrix3d& A,
                                                     const Eigen::Matrix3d& B) const
{
  Eigen::Matrix<double,9,9> K;
  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++){
      K.block<3,3>(3*i,3*j) = A(i,j) * B;
    }
  }
  return K;
}

// =====================================================
// T_SA computation rule (axis correction is ONLY here)
// Input: R_total from calibrated_pose rotvec (rad)
// Output: T_SA (4x4) used as right-multiply in vive_tracker: M_cal = ... @ T_SA
//
// We want: R_total * R_SA has desired w_z alignment behavior.
// Here we implement your "side" convention to avoid mirrored axes.
// - keep it contained: changing alignment -> only edit this function.
// =====================================================
Eigen::Matrix4d VrCalibrationSolver::computeTSAFromCalibratedPoseRotation(const Eigen::Matrix3d& R_total) const
{
  // We align the "spatial-angle frame" so that the resulting yaw-like component matches desired sign.
  // Practical: build a rotation around Z by +/- t_sa_w_des_z_ then apply sign depending on side.

  const double sgn = (t_sa_side_ == "left") ? +1.0 : -1.0;
  const double ang = sgn * t_sa_w_des_z_;

  Eigen::Matrix3d R_SA = Eigen::AngleAxisd(ang, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Because vive_tracker right-multiplies, we store pure R_SA here.
  // If you ever want to incorporate R_total-dependent logic, do it here (and only here).
  (void)R_total;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = R_SA;
  return T;
}

void VrCalibrationSolver::finalizeAndSave(bool t_sa_computed, const Eigen::Matrix4d& T_SA_new)
{
  const size_t N = T_AB_all_.size();
  if (N < 2 || T_DC_all_.size() != N) {
    throw std::runtime_error("Not enough samples (need >= 2).");
  }

  const size_t K = N - 1;

  Eigen::MatrixXd M(9*K, 9);
  Eigen::MatrixXd K1(3*K, 3);
  Eigen::VectorXd K2(3*K);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  std::vector<Eigen::Vector3d> O_B0B1_list;
  std::vector<Eigen::Vector3d> O_C0C1_list;
  std::vector<Eigen::Matrix4d> T_AB0_list;
  std::vector<Eigen::Matrix4d> T_DC0_list;

  for (size_t i=0;i<K;i++){
    const Eigen::Matrix4d& T_AB0 = T_AB_all_[i];
    const Eigen::Matrix4d& T_AB1 = T_AB_all_[i+1];
    const Eigen::Matrix4d& T_DC0 = T_DC_all_[i];
    const Eigen::Matrix4d& T_DC1 = T_DC_all_[i+1];

    const Eigen::Matrix4d T_B0B1 = invT(T_AB0) * T_AB1;
    const Eigen::Matrix4d T_C0C1 = invT(T_DC0) * T_DC1;

    const Eigen::Matrix3d R_B0B1 = T_B0B1.block<3,3>(0,0);
    const Eigen::Vector3d O_B0B1 = T_B0B1.block<3,1>(0,3);

    const Eigen::Matrix3d R_C0C1 = T_C0C1.block<3,3>(0,0);
    const Eigen::Vector3d O_C0C1 = T_C0C1.block<3,1>(0,3);

    M.block(9*i,0,9,9) = kron3(I, R_B0B1) - kron3(R_C0C1.transpose(), I);
    K1.block(3*i,0,3,3) = (I - R_B0B1);

    O_B0B1_list.push_back(O_B0B1);
    O_C0C1_list.push_back(O_C0C1);
    T_AB0_list.push_back(T_AB0);
    T_DC0_list.push_back(T_DC0);
  }

  // solve rotation (null space of M)
  Eigen::MatrixXd X = M.transpose() * M;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(X);
  if (es.info() != Eigen::Success) throw std::runtime_error("EigenSolver failed");

  Eigen::VectorXd v = es.eigenvectors().col(0);
  Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::ColMajor>> R_BC_raw(v.data());

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_BC_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d R_BC = U * V.transpose();
  if (R_BC.determinant() < 0) {
    U.col(2) *= -1.0;
    R_BC = U * V.transpose();
  }

  // solve translation
  for (size_t i=0;i<K;i++){
    K2.segment<3>(3*i) = O_B0B1_list[i] - R_BC * O_C0C1_list[i];
  }
  Eigen::Vector3d O_BC = K1.colPivHouseholderQr().solve(K2);

  Eigen::Matrix4d T_BC = Eigen::Matrix4d::Identity();
  T_BC.block<3,3>(0,0) = R_BC;
  T_BC.block<3,1>(0,3) = O_BC;

  // average T_AD
  std::vector<Eigen::Quaterniond> quats;
  quats.reserve(K);
  Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();

  for (size_t i=0;i<K;i++){
    Eigen::Matrix4d T_AD = T_AB0_list[i] * T_BC * invT(T_DC0_list[i]);
    Eigen::Matrix3d R = T_AD.block<3,3>(0,0);
    Eigen::Vector3d t = T_AD.block<3,1>(0,3);

    Eigen::Quaterniond q(R);
    q.normalize();
    quats.push_back(q);
    t_sum += t;
  }

  Eigen::Quaterniond q_ref = quats.front();
  Eigen::Vector4d q_sum = Eigen::Vector4d::Zero();
  for (auto& q : quats) {
    if (q_ref.coeffs().dot(q.coeffs()) < 0) q.coeffs() *= -1.0;
    q_sum += q.coeffs();
  }
  q_sum /= static_cast<double>(quats.size());
  Eigen::Quaterniond q_mean;
  q_mean.coeffs() = q_sum;
  q_mean.normalize();

  Eigen::Vector3d t_mean = t_sum / static_cast<double>(K);

  Eigen::Matrix4d T_AD_avg = Eigen::Matrix4d::Identity();
  T_AD_avg.block<3,3>(0,0) = q_mean.toRotationMatrix();
  T_AD_avg.block<3,1>(0,3) = t_mean;

  // R_Adj (keep as existing style: optional; here we compute a simple quaternion mean about VR)
  // If you already have a specific original R_Adj method, put it here; VrCalibration never changes.
  if (vr_quats_.size() >= 2) {
    Eigen::Quaterniond q0 = vr_quats_.front();
    Eigen::Vector4d qs = Eigen::Vector4d::Zero();
    for (auto q : vr_quats_) {
      if (q0.coeffs().dot(q.coeffs()) < 0) q.coeffs() *= -1.0;
      qs += q.coeffs();
    }
    qs /= static_cast<double>(vr_quats_.size());
    Eigen::Quaterniond qmean;
    qmean.coeffs() = qs;
    qmean.normalize();
    R_adj_ = qmean.toRotationMatrix();
    have_radj_ = true;
  } else {
    have_radj_ = false;
    R_adj_.setIdentity();
  }

  const Eigen::Matrix4d T_SA_to_save = t_sa_computed ? T_SA_new : T_SA_old_;

  writeCalibrationYamlAll(
    T_AD_avg,
    T_BC,
    have_radj_ ? R_adj_ : Eigen::Matrix3d::Identity(),
    T_CE_,
    T_SA_to_save
  );
}

void VrCalibrationSolver::writeCalibrationYamlAll(const Eigen::Matrix4d& T_AD,
                                                  const Eigen::Matrix4d& T_BC,
                                                  const Eigen::Matrix3d& R_Adj,
                                                  const Eigen::Matrix4d& T_CE,
                                                  const Eigen::Matrix4d& T_SA)
{
  std::ofstream ofs(calib_yaml_path_, std::ios::out | std::ios::trunc);
  if (!ofs.is_open()) throw std::runtime_error("Failed to open yaml: " + calib_yaml_path_);

  const int prec = 12;

  ofs << "# VR calibration matrix setting\n";
  ofs << "# Auto-capture + One-shot YAML update (R_Adj, T_AD, T_BC, T_SA)\n\n";
  ofs << "meta:\n";
  ofs << "  t_sa_w_des_z: " << std::fixed << std::setprecision(prec) << t_sa_w_des_z_ << "\n";
  ofs << "  t_sa_side: \"" << t_sa_side_ << "\"\n";
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
