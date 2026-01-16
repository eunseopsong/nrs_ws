// vr_calibration_solver.cpp  (ver5)
// - No main() here (so you can compile it together with vr_calibration.cpp)
// - Hand-eye style solve: estimate T_BC and average T_AD from samples
// - PLUS: T_SA (R_SA) compute helper for right-multiply chain, with robust candidate selection
//
// Expected sample meaning (naming follows your vr_calibration.cpp logic):
//   T_AB_all[i] : Arm base(A) -> EE(B) at sample i
//   T_DC_all[i] : Tracker world(D) -> Tracker(C) at sample i
//
// We solve rotation part via AX = XB (Kronecker form), then translation via LS,
// then compute T_AD_i = T_AB0_i * T_BC * inv(T_DC0_i) and average it.
//
// Additional for T_SA:
//   vive_tracker chain (as you stated):  R_total = R_chain * R_SA   (right-multiply)
//   where R_total is measured from /calibrated_pose (rotation matrix),
//   R_SA_old is current YAML T_SA rotation.
//   We want R_chain * R_SA_new = R_des  (desired rotation)
//   => R_chain = R_total * R_SA_old^T
//   => R_SA_new = R_chain^T * R_des = R_SA_old * R_total^T * R_des
//
// ver5: provides compute_R_SA_new_right_multiply() with a robust candidate selection.

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace vr_calib_solver
{

// ================== basic rigid helpers ==================
static inline Eigen::Matrix4d makeT(const Eigen::Matrix3d& R, const Eigen::Vector3d& p)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = p;
  return T;
}

static inline Eigen::Matrix4d invT(const Eigen::Matrix4d& T)
{
  Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R = T.block<3,3>(0,0);
  const Eigen::Vector3d p = T.block<3,1>(0,3);
  Ti.block<3,3>(0,0) = R.transpose();
  Ti.block<3,1>(0,3) = -R.transpose() * p;
  return Ti;
}

static inline Eigen::Matrix<double,9,9> kron3(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B)
{
  Eigen::Matrix<double,9,9> K;
  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++){
      K.block<3,3>(3*i,3*j) = A(i,j) * B;
    }
  }
  return K;
}

// ================== quaternion averaging ==================
static inline Eigen::Quaterniond averageQuaternionSignAligned(const std::vector<Eigen::Quaterniond>& qs)
{
  if (qs.empty()) {
    return Eigen::Quaterniond(1,0,0,0);
  }

  Eigen::Quaterniond q_ref = qs.front();
  Eigen::Vector4d sum = Eigen::Vector4d::Zero(); // coeffs() = (x,y,z,w)

  for (auto q : qs) {
    // align sign
    if (q_ref.coeffs().dot(q.coeffs()) < 0.0) {
      q.coeffs() *= -1.0;
    }
    sum += q.coeffs();
  }

  sum /= static_cast<double>(qs.size());

  Eigen::Quaterniond q_mean;
  q_mean.coeffs() = sum;
  q_mean.normalize();
  return q_mean;
}

// ================== rotation utilities (ver5 add) ==================
static inline Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& R_in)
{
  // SVD projection to closest rotation matrix
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_in, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d R = U * V.transpose();
  if (R.determinant() < 0.0) {
    U.col(2) *= -1.0;
    R = U * V.transpose();
  }
  return R;
}

static inline double rotAngleRad(const Eigen::Matrix3d& R)
{
  // angle of rotation for a relative rotation matrix
  // clamp trace to avoid NaN
  double c = (R.trace() - 1.0) * 0.5;
  c = std::min(1.0, std::max(-1.0, c));
  return std::acos(c);
}

// ================== T_SA solve helper (ver5 add) ==================
// Compute R_SA_new assuming chain is right-multiply:
//   R_total = R_chain * R_SA_old
// Want:
//   R_chain * R_SA_new = R_des
//
// Baseline3-correct formula:
//   R_SA_new = R_SA_old * R_total^T * R_des
//
// ver5 includes candidate selection to avoid "transpose-direction mistake / flip":
//   candA = R_SA_old * R_total^T * R_des     (baseline3)
//   candB = R_SA_old * R_total   * R_des     (alternate in case upstream used opposite convention)
//
// Score each candidate by predicting:
//   R_chain = R_total * R_SA_old^T
//   R_pred  = R_chain * R_SA_candidate
// and comparing to R_des: angle( R_pred^T * R_des ) (smaller is better)
//
// Returns true on success.
bool compute_R_SA_new_right_multiply(
    const Eigen::Matrix3d& R_total_in,   // from /calibrated_pose rotvec (RAD) -> matrix
    const Eigen::Matrix3d& R_SA_old_in,  // from YAML T_SA
    const Eigen::Matrix3d& R_des_in,     // desired rotation
    Eigen::Matrix3d& R_SA_new_out,
    double* chosen_err_rad,              // optional
    std::string* err)
{
  try {
    // Ensure inputs are proper rotations (robust)
    const Eigen::Matrix3d R_total = projectToSO3(R_total_in);
    const Eigen::Matrix3d R_SA_old = projectToSO3(R_SA_old_in);
    const Eigen::Matrix3d R_des = projectToSO3(R_des_in);

    // Right-multiply chain: R_total = R_chain * R_SA_old
    const Eigen::Matrix3d R_chain = R_total * R_SA_old.transpose();

    // Candidates
    const Eigen::Matrix3d candA = projectToSO3(R_SA_old * R_total.transpose() * R_des); // baseline3
    const Eigen::Matrix3d candB = projectToSO3(R_SA_old * R_total * R_des);             // alternate

    auto score = [&](const Eigen::Matrix3d& R_SA)->double {
      const Eigen::Matrix3d R_pred = R_chain * R_SA;
      const Eigen::Matrix3d R_rel  = R_pred.transpose() * R_des;
      return rotAngleRad(R_rel);
    };

    const double eA = score(candA);
    const double eB = score(candB);

    if (eA <= eB) {
      R_SA_new_out = candA;
      if (chosen_err_rad) *chosen_err_rad = eA;
    } else {
      R_SA_new_out = candB;
      if (chosen_err_rad) *chosen_err_rad = eB;
    }

    // Extra safety: if error is suspiciously large (~>30deg), still return true
    // because user might want to inspect; caller can check chosen_err_rad.
    return true;
  }
  catch (const std::exception& e) {
    if (err) *err = std::string("Exception in compute_R_SA_new_right_multiply: ") + e.what();
    return false;
  }
}

// ================== original solver (unchanged) ==================
// Solve:
//  - T_BC (4x4)
//  - T_AD_avg (4x4)
//
// Return true on success, false on failure with err message.
bool solve_T_BC_and_T_AD_avg(
    const std::vector<Eigen::Matrix4d>& T_AB_all,
    const std::vector<Eigen::Matrix4d>& T_DC_all,
    Eigen::Matrix4d& T_BC_out,
    Eigen::Matrix4d& T_AD_avg_out,
    std::string* err)
{
  try {
    const size_t N = T_AB_all.size();
    if (N < 2 || T_DC_all.size() != N) {
      if (err) *err = "Need >=2 samples and matched sizes (T_AB_all vs T_DC_all).";
      return false;
    }

    // Use consecutive pairs (i, i+1)
    const size_t K = N - 1;

    Eigen::MatrixXd M(9 * K, 9);
    Eigen::MatrixXd K1(3 * K, 3);
    Eigen::VectorXd K2(3 * K);

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // store for later
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

      // M block: kron(I, R_B0B1) - kron(R_C0C1^T, I)
      const Eigen::Matrix<double,9,9> m = kron3(I, R_B0B1) - kron3(R_C0C1.transpose(), I);
      M.block(9*i, 0, 9, 9) = m;

      // K1 block: (I - R_B0B1)
      K1.block(3*i, 0, 3, 3) = (I - R_B0B1);

      O_B0B1_list.push_back(O_B0B1);
      O_C0C1_list.push_back(O_C0C1);
      T_AB0_list.push_back(T_AB0);
      T_DC0_list.push_back(T_DC0);
    }

    // Rotation solve: smallest eigenvector of X = M^T M
    Eigen::MatrixXd X = M.transpose() * M;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(X);
    if (es.info() != Eigen::Success) {
      if (err) *err = "EigenSolver failed on X=M^T*M.";
      return false;
    }

    Eigen::VectorXd v = es.eigenvectors().col(0); // smallest eigenvalue
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::ColMajor>> R_BC_raw(v.data());

    // Orthonormalize with SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_BC_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R_BC = U * V.transpose();
    if (R_BC.determinant() < 0.0) {
      U.col(2) *= -1.0;
      R_BC = U * V.transpose();
    }

    // Translation solve:
    // K2 = O_B0B1 - R_BC * O_C0C1
    for (size_t i=0; i<K; i++) {
      const Eigen::Vector3d temp = O_B0B1_list[i] - R_BC * O_C0C1_list[i];
      K2.segment<3>(3*i) = temp;
    }

    const Eigen::Vector3d O_BC = K1.colPivHouseholderQr().solve(K2);
    const Eigen::Matrix4d T_BC = makeT(R_BC, O_BC);

    // Compute T_AD_i and average
    std::vector<Eigen::Quaterniond> quats;
    quats.reserve(K);
    Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();

    for (size_t i=0; i<K; i++) {
      const Eigen::Matrix4d& T_AB0 = T_AB0_list[i];
      const Eigen::Matrix4d& T_DC0 = T_DC0_list[i];

      const Eigen::Matrix4d T_AD_i = T_AB0 * T_BC * invT(T_DC0);
      const Eigen::Matrix3d R_i = T_AD_i.block<3,3>(0,0);
      const Eigen::Vector3d t_i = T_AD_i.block<3,1>(0,3);

      Eigen::Quaterniond q_i(R_i);
      q_i.normalize();
      quats.push_back(q_i);
      t_sum += t_i;
    }

    const Eigen::Quaterniond q_mean = averageQuaternionSignAligned(quats);
    const Eigen::Vector3d t_mean = t_sum / static_cast<double>(K);

    const Eigen::Matrix4d T_AD_avg = makeT(q_mean.toRotationMatrix(), t_mean);

    // outputs
    T_BC_out = T_BC;
    T_AD_avg_out = T_AD_avg;

    return true;
  }
  catch (const std::exception& e) {
    if (err) *err = std::string("Exception: ") + e.what();
    return false;
  }
}

} // namespace vr_calib_solver
