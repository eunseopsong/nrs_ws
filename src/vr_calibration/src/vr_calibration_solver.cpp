// vr_calibration_solver.cpp
// Offline solver: read ur10_ee.txt + ur10_vr.txt, compute T_BC / T_AD / R_Adj and write calibration_matrix.yaml
// Key change vs legacy: NO manual sign flipping for x or wz.
// Additionally: auto-detect whether VR pose in file is (world<-tracker) or (tracker<-world) by minimizing residual.

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <cstdlib>
#include <stdexcept>
#include <algorithm>
#include <limits>

// -------------------- small helpers --------------------
static constexpr double kPi = 3.14159265358979323846;

static inline double clampd(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static Eigen::Matrix4d makeT(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = p;
  return T;
}

static Eigen::Matrix4d invT(const Eigen::Matrix4d& T) {
  Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R = T.block<3,3>(0,0);
  const Eigen::Vector3d p = T.block<3,1>(0,3);
  Ti.block<3,3>(0,0) = R.transpose();
  Ti.block<3,1>(0,3) = -R.transpose() * p;
  return Ti;
}

static Eigen::Matrix3d orthonormalizeR(const Eigen::Matrix3d& R) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d Rn = U * V.transpose();
  if (Rn.determinant() < 0) {
    U.col(2) *= -1.0;
    Rn = U * V.transpose();
  }
  return Rn;
}

static double rotAngleRad(const Eigen::Matrix3d& R) {
  double cosang = (R.trace() - 1.0) * 0.5;
  cosang = clampd(cosang, -1.0, 1.0);
  return std::acos(cosang);
}

static Eigen::Matrix<double,9,9> kron3(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
  Eigen::Matrix<double,9,9> K;
  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++){
      K.block<3,3>(3*i,3*j) = A(i,j) * B;
    }
  }
  return K;
}

// -------------------- YAML helpers --------------------
static bool readMat4(const YAML::Node& n, Eigen::Matrix4d& T) {
  if (!n || !n.IsSequence() || n.size() != 4) return false;
  for (int r=0;r<4;r++){
    if (!n[r].IsSequence() || n[r].size() != 4) return false;
    for (int c=0;c<4;c++){
      T(r,c) = n[r][c].as<double>();
    }
  }
  return true;
}

static void writeMat4(std::ofstream& ofs, const std::string& key, const Eigen::Matrix4d& T, int prec=12) {
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
}

static void writeMat3(std::ofstream& ofs, const std::string& key, const Eigen::Matrix3d& R, int prec=12) {
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
}

// -------------------- core solve --------------------
struct SolveResult {
  Eigen::Matrix4d T_BC = Eigen::Matrix4d::Identity(); // (B<-C) : EE <- Tracker
  Eigen::Matrix4d T_AD = Eigen::Matrix4d::Identity(); // (A<-D) : RobotBase <- VRWorld
  Eigen::Matrix3d R_Adj = Eigen::Matrix3d::Identity();
  double mean_pos_err_m = std::numeric_limits<double>::infinity();
  double mean_ang_err_deg = std::numeric_limits<double>::infinity();
  bool ok = false;
};

static void computeRAdjFromFirstTwo(const std::vector<Eigen::Matrix4d>& T_DC,
                                   Eigen::Matrix3d& R_adj_out)
{
  // Same as your capture code: R_Adj = R1^T * R2 (right-multiply adjustment style)
  // NOTE: This does NOT apply any sign flips.
  if (T_DC.size() < 2) {
    R_adj_out = Eigen::Matrix3d::Identity();
    return;
  }
  const Eigen::Matrix3d R1 = T_DC[0].block<3,3>(0,0);
  const Eigen::Matrix3d R2 = T_DC[1].block<3,3>(0,0);
  R_adj_out = R1.transpose() * R2;
  R_adj_out = orthonormalizeR(R_adj_out);
}

static SolveResult solveHandEye_AndEvaluate(const std::vector<Eigen::Matrix4d>& T_AB_all,
                                           const std::vector<Eigen::Matrix4d>& T_DC_all)
{
  SolveResult res;
  const size_t N = T_AB_all.size();
  if (N < 2 || T_DC_all.size() != N) return res;

  const size_t K = N - 1;
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd M(9 * K, 9);
  Eigen::MatrixXd K1(3 * K, 3);
  Eigen::VectorXd K2(3 * K);

  std::vector<Eigen::Vector3d> tA_list;
  std::vector<Eigen::Vector3d> tB_list;
  std::vector<Eigen::Matrix4d> T_AB0_list;
  std::vector<Eigen::Matrix4d> T_DC0_list;
  tA_list.reserve(K);
  tB_list.reserve(K);
  T_AB0_list.reserve(K);
  T_DC0_list.reserve(K);

  for (size_t i=0;i<K;i++){
    const Eigen::Matrix4d& T_AB0 = T_AB_all[i];
    const Eigen::Matrix4d& T_AB1 = T_AB_all[i+1];
    const Eigen::Matrix4d& T_DC0 = T_DC_all[i];
    const Eigen::Matrix4d& T_DC1 = T_DC_all[i+1];

    const Eigen::Matrix4d T_B0B1 = invT(T_AB0) * T_AB1;
    const Eigen::Matrix4d T_C0C1 = invT(T_DC0) * T_DC1;

    const Eigen::Matrix3d R_A = T_B0B1.block<3,3>(0,0);
    const Eigen::Matrix3d R_B = T_C0C1.block<3,3>(0,0);
    const Eigen::Vector3d t_A = T_B0B1.block<3,1>(0,3);
    const Eigen::Vector3d t_B = T_C0C1.block<3,1>(0,3);

    // (I ⊗ R_A - R_B^T ⊗ I) vec(R_X) = 0
    Eigen::Matrix<double,9,9> m = kron3(I, R_A) - kron3(R_B.transpose(), I);
    M.block(9*i, 0, 9, 9) = m;

    // (I - R_A) t_X = t_A - R_X t_B
    K1.block(3*i, 0, 3, 3) = (I - R_A);

    tA_list.push_back(t_A);
    tB_list.push_back(t_B);
    T_AB0_list.push_back(T_AB0);
    T_DC0_list.push_back(T_DC0);
  }

  // rotation solve by smallest eigenvector of M^T M
  Eigen::MatrixXd X = M.transpose() * M;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(X);
  if (es.info() != Eigen::Success) return res;

  Eigen::VectorXd v = es.eigenvectors().col(0); // smallest
  Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::ColMajor>> R_raw(v.data());
  Eigen::Matrix3d R_BC = orthonormalizeR(R_raw);

  // translation solve
  for (size_t i=0;i<K;i++){
    K2.segment<3>(3*i) = tA_list[i] - R_BC * tB_list[i];
  }
  Eigen::Vector3d t_BC = K1.colPivHouseholderQr().solve(K2);
  Eigen::Matrix4d T_BC = makeT(R_BC, t_BC);

  // compute T_AD_i = T_AB0_i * T_BC * inv(T_DC0_i), average over i
  std::vector<Eigen::Quaterniond> quats;
  quats.reserve(K);
  Eigen::Vector3d tsum = Eigen::Vector3d::Zero();

  for (size_t i=0;i<K;i++){
    Eigen::Matrix4d T_AD_i = T_AB0_list[i] * T_BC * invT(T_DC0_list[i]);
    Eigen::Matrix3d Ri = orthonormalizeR(T_AD_i.block<3,3>(0,0));
    Eigen::Vector3d ti = T_AD_i.block<3,1>(0,3);

    Eigen::Quaterniond q(Ri);
    q.normalize();
    quats.push_back(q);
    tsum += ti;
  }

  // average quaternion (sign-aligned)
  Eigen::Quaterniond qref = quats.front();
  Eigen::Vector4d qsum = Eigen::Vector4d::Zero(); // coeffs() = (x,y,z,w)
  for (auto q : quats){
    if (qref.coeffs().dot(q.coeffs()) < 0) q.coeffs() *= -1.0;
    qsum += q.coeffs();
  }
  qsum /= static_cast<double>(K);
  Eigen::Quaterniond qmean;
  qmean.coeffs() = qsum;
  qmean.normalize();

  Eigen::Vector3d tmean = tsum / static_cast<double>(K);
  Eigen::Matrix4d T_AD = makeT(qmean.toRotationMatrix(), tmean);

  // compute R_Adj from first two VR poses (optional but kept)
  Eigen::Matrix3d R_Adj;
  computeRAdjFromFirstTwo(T_DC_all, R_Adj);

  // evaluate consistency:
  // predicted robot EE pose: T_AB_pred_i = T_AD * T_DC_i * inv(T_BC)
  double pos_sum = 0.0;
  double ang_sum = 0.0;
  for (size_t i=0;i<N;i++){
    Eigen::Matrix4d T_AB_pred = T_AD * T_DC_all[i] * invT(T_BC);
    const Eigen::Vector3d t_meas = T_AB_all[i].block<3,1>(0,3);
    const Eigen::Vector3d t_pred = T_AB_pred.block<3,1>(0,3);
    pos_sum += (t_pred - t_meas).norm();

    Eigen::Matrix3d R_meas = T_AB_all[i].block<3,3>(0,0);
    Eigen::Matrix3d R_pred = T_AB_pred.block<3,3>(0,0);
    Eigen::Matrix3d R_err = R_meas.transpose() * R_pred;
    double ang = rotAngleRad(R_err);
    ang_sum += ang * 180.0 / kPi;
  }

  res.T_BC = T_BC;
  res.T_AD = T_AD;
  res.R_Adj = R_Adj;
  res.mean_pos_err_m = pos_sum / static_cast<double>(N);
  res.mean_ang_err_deg = ang_sum / static_cast<double>(N);
  res.ok = true;
  return res;
}

// -------------------- file IO --------------------
static void loadEEFile(const std::string& ee_path, std::vector<Eigen::Matrix4d>& T_AB_all)
{
  std::ifstream ifs(ee_path);
  if (!ifs.is_open()) throw std::runtime_error("Failed to open EE file: " + ee_path);

  T_AB_all.clear();
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;

    std::istringstream ss(line);
    double r00,r01,r02,px;
    double r10,r11,r12,py;
    double r20,r21,r22,pz;
    if (!(ss >> r00 >> r01 >> r02 >> px
             >> r10 >> r11 >> r12 >> py
             >> r20 >> r21 >> r22 >> pz)) {
      continue;
    }
    Eigen::Matrix3d R;
    R << r00,r01,r02,
         r10,r11,r12,
         r20,r21,r22;
    R = orthonormalizeR(R);
    Eigen::Vector3d p(px,py,pz); // already meters in your writer
    T_AB_all.push_back(makeT(R,p));
  }

  if (T_AB_all.size() < 2) {
    throw std::runtime_error("EE file has <2 valid lines. Need at least 2 samples.");
  }
}

static void loadVRFile_AsPose(const std::string& vr_path,
                             std::vector<Eigen::Matrix4d>& T_DC_all_as_is)
{
  std::ifstream ifs(vr_path);
  if (!ifs.is_open()) throw std::runtime_error("Failed to open VR file: " + vr_path);

  T_DC_all_as_is.clear();
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);

    double x,y,z,qx,qy,qz,qw;
    if (!(ss >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;

    // IMPORTANT: no sign flip here.
    Eigen::Quaterniond q(qw,qx,qy,qz);
    q.normalize();
    Eigen::Matrix3d R = orthonormalizeR(q.toRotationMatrix());
    Eigen::Vector3d p(x,y,z); // already meters in your writer
    T_DC_all_as_is.push_back(makeT(R,p));
  }

  if (T_DC_all_as_is.size() < 2) {
    throw std::runtime_error("VR file has <2 valid lines. Need at least 2 samples.");
  }
}

// -------------------- node --------------------
class VrCalibrationSolverNode : public rclcpp::Node
{
public:
  VrCalibrationSolverNode()
  : Node("vr_calibration_solver")
  {
    // defaults (match your environment)
    const char* home = std::getenv("HOME");
    if (!home) throw std::runtime_error("HOME env not set");

    this->declare_parameter<std::string>("ee_path",
      "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/vr_calibration/ur10_ee.txt");
    this->declare_parameter<std::string>("vr_path",
      "/home/eunseop/dev_ws/src/y2_ur10skku_control/Y2RobMotion/vr_calibration/ur10_vr.txt");
    this->declare_parameter<std::string>("yaml_path",
      std::string(home) + "/nrs_ws/src/vive_tracker_ros2/yaml/calibration_matrix.yaml");

    // VR pose convention in file:
    //   "auto" (default): try both (world<-tracker) and (tracker<-world) and pick smaller residual
    //   "world_from_tracker": treat file pose as (world<-tracker)
    //   "tracker_from_world": treat file pose as (tracker<-world) and invert to get (world<-tracker)
    this->declare_parameter<std::string>("vr_pose_convention", "auto");

    // keep T_CE / T_SA if present
    this->declare_parameter<bool>("keep_existing_T_CE", true);
    this->declare_parameter<bool>("keep_existing_T_SA", true);

    // optional: write R_Adj computed from first two VR poses
    this->declare_parameter<bool>("write_R_Adj", true);

    ee_path_ = this->get_parameter("ee_path").as_string();
    vr_path_ = this->get_parameter("vr_path").as_string();
    yaml_path_ = this->get_parameter("yaml_path").as_string();
    vr_pose_convention_ = this->get_parameter("vr_pose_convention").as_string();
    keep_T_CE_ = this->get_parameter("keep_existing_T_CE").as_bool();
    keep_T_SA_ = this->get_parameter("keep_existing_T_SA").as_bool();
    write_R_Adj_ = this->get_parameter("write_R_Adj").as_bool();
  }

  void run()
  {
    // load inputs
    std::vector<Eigen::Matrix4d> T_AB_all;
    std::vector<Eigen::Matrix4d> T_DC_as_is;
    loadEEFile(ee_path_, T_AB_all);
    loadVRFile_AsPose(vr_path_, T_DC_as_is);

    // load existing yaml constants (optional)
    Eigen::Matrix4d T_CE = Eigen::Matrix4d::Identity();
    T_CE(2,3) = 0.222; // fallback default you used
    Eigen::Matrix4d T_SA = Eigen::Matrix4d::Identity();

    try {
      YAML::Node existing = YAML::LoadFile(yaml_path_);
      if (keep_T_CE_ && existing["T_CE"]) {
        Eigen::Matrix4d tmp;
        if (readMat4(existing["T_CE"], tmp)) T_CE = tmp;
      }
      if (keep_T_SA_ && existing["T_SA"]) {
        Eigen::Matrix4d tmp;
        if (readMat4(existing["T_SA"], tmp)) T_SA = tmp;
      }
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Cannot load existing yaml (will create new). path=%s", yaml_path_.c_str());
    }

    // build two candidates for T_DC:
    //   cand0: as-is (assume world<-tracker)
    //   cand1: inverted (assume tracker<-world in file, invert to world<-tracker)
    std::vector<Eigen::Matrix4d> T_DC_cand0 = T_DC_as_is;
    std::vector<Eigen::Matrix4d> T_DC_cand1;
    T_DC_cand1.reserve(T_DC_as_is.size());
    for (const auto& T : T_DC_as_is) T_DC_cand1.push_back(invT(T));

    auto solveWithConvention = [&](const std::string& mode)->SolveResult {
      if (mode == "world_from_tracker") {
        return solveHandEye_AndEvaluate(T_AB_all, T_DC_cand0);
      } else if (mode == "tracker_from_world") {
        return solveHandEye_AndEvaluate(T_AB_all, T_DC_cand1);
      } else {
        // auto
        SolveResult r0 = solveHandEye_AndEvaluate(T_AB_all, T_DC_cand0);
        SolveResult r1 = solveHandEye_AndEvaluate(T_AB_all, T_DC_cand1);
        if (!r0.ok) return r1;
        if (!r1.ok) return r0;
        // choose smaller combined metric
        const double s0 = r0.mean_pos_err_m + 0.01 * (r0.mean_ang_err_deg); // 1deg ~ 1cm scale
        const double s1 = r1.mean_pos_err_m + 0.01 * (r1.mean_ang_err_deg);
        return (s0 <= s1) ? r0 : r1;
      }
    };

    SolveResult best = solveWithConvention(vr_pose_convention_);
    if (!best.ok) throw std::runtime_error("Solve failed (not enough data or eigen failure).");

    // If user requested explicit mode, print which was used; if auto, infer by recomputing both scores
    std::string chosen = vr_pose_convention_;
    if (vr_pose_convention_ == "auto") {
      SolveResult r0 = solveHandEye_AndEvaluate(T_AB_all, T_DC_cand0);
      SolveResult r1 = solveHandEye_AndEvaluate(T_AB_all, T_DC_cand1);
      const double s0 = r0.mean_pos_err_m + 0.01 * (r0.mean_ang_err_deg);
      const double s1 = r1.mean_pos_err_m + 0.01 * (r1.mean_ang_err_deg);
      chosen = (s0 <= s1) ? "world_from_tracker" : "tracker_from_world";
    }

    RCLCPP_INFO(get_logger(),
      "Solve done. chosen_vr_pose_convention=%s | mean_pos_err=%.6fm | mean_ang_err=%.4fdeg",
      chosen.c_str(), best.mean_pos_err_m, best.mean_ang_err_deg);

    // write yaml
    std::ofstream ofs(yaml_path_, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) throw std::runtime_error("Failed to open yaml for write: " + yaml_path_);

    const int prec = 12;
    ofs << "# VR calibration matrix setting\n";
    ofs << "# Offline solver output (NO manual sign flip for x/wz)\n";
    ofs << "meta:\n";
    ofs << "  vr_pose_convention_chosen: \"" << chosen << "\"\n";
    ofs << "  mean_pos_err_m: " << std::fixed << std::setprecision(prec) << best.mean_pos_err_m << "\n";
    ofs << "  mean_ang_err_deg: " << std::fixed << std::setprecision(prec) << best.mean_ang_err_deg << "\n\n";

    writeMat4(ofs, "T_AD", best.T_AD, prec);
    writeMat4(ofs, "T_BC", best.T_BC, prec);

    if (write_R_Adj_) writeMat3(ofs, "R_Adj", best.R_Adj, prec);
    else             writeMat3(ofs, "R_Adj", Eigen::Matrix3d::Identity(), prec);

    ofs << "# constant offset: keep existing if available\n";
    writeMat4(ofs, "T_CE", T_CE, prec);

    ofs << "# spatial-angle frame alignment: keep existing if available\n";
    writeMat4(ofs, "T_SA", T_SA, prec);

    ofs.flush();

    RCLCPP_INFO(get_logger(), "YAML written: %s", yaml_path_.c_str());
  }

private:
  std::string ee_path_;
  std::string vr_path_;
  std::string yaml_path_;
  std::string vr_pose_convention_;
  bool keep_T_CE_{true};
  bool keep_T_SA_{true};
  bool write_R_Adj_{true};
};

// -------------------- main --------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<VrCalibrationSolverNode>();
    node->run();
  } catch (const std::exception& e) {
    std::cerr << "vr_calibration_solver exception: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
