#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace vr_calib {

// ----------------------------
// Waypoint definition
// pose: [x y z wx wy wz]  (position: mm, rotvec: deg or rad depending on file)
// flag: 0/1 (1 = target)
// ----------------------------
struct Waypoint
{
  std::array<double,6> pose{};
  int flag = 0;
};

inline double rad2deg(double r) { return r * 180.0 / M_PI; }
inline double deg2rad(double d) { return d * M_PI / 180.0; }

// rotvec (rad) -> R
inline Eigen::Matrix3d rotvecToRotMat(const Eigen::Vector3d& w_rad)
{
  const double th = w_rad.norm();
  if (th < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(th, w_rad / th).toRotationMatrix();
}

// angle between rotations (rad)
inline double rotAngleBetweenRad(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2)
{
  const Eigen::Matrix3d R = R1.transpose() * R2;
  double c = (R.trace() - 1.0) * 0.5;
  c = std::min(1.0, std::max(-1.0, c));
  return std::acos(c);
}

inline Eigen::Matrix4d makeT(const Eigen::Matrix3d& R, const Eigen::Vector3d& p)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = p;
  return T;
}

// position distance in mm (cp and target are [mm, rotvec...])
inline double posDistMm(const std::array<double,6>& cp,
                        const std::array<double,6>& target)
{
  const double dx = cp[0] - target[0];
  const double dy = cp[1] - target[1];
  const double dz = cp[2] - target[2];
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// orientation error in deg
// cp rotvec can be deg or rad (cp_deg flag)
// target rotvec can be deg or rad (target_deg flag)
inline double oriErrDeg(const std::array<double,6>& cp,
                        bool cp_deg,
                        const std::array<double,6>& target,
                        bool target_deg)
{
  Eigen::Vector3d w_cp(cp[3], cp[4], cp[5]);
  Eigen::Vector3d w_tg(target[3], target[4], target[5]);
  if (cp_deg)     w_cp *= (M_PI/180.0);
  if (target_deg) w_tg *= (M_PI/180.0);

  const Eigen::Matrix3d Rcp = rotvecToRotMat(w_cp);
  const Eigen::Matrix3d Rtg = rotvecToRotMat(w_tg);
  const double ang = rotAngleBetweenRad(Rcp, Rtg);
  return rad2deg(ang);
}

// -------------------------------------------------------
// Load waypoint file and detect unit for rotvec in waypoint file
// Format each line: x y z wx wy wz [flag]
// If flag missing -> flag=1
// Detect unit: if max(|wx,wy,wz|) > 6 => degrees else radians
// -------------------------------------------------------
inline void loadWaypointsAndDetectUnits(const std::string& path,
                                       std::vector<Waypoint>& out,
                                       bool& rotvec_in_degrees)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Failed to load waypoint file: " + path);
  }

  out.clear();
  double max_abs = 0.0;

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);

    Waypoint w;
    double x,y,z,wx,wy,wz;
    if (!(iss >> x >> y >> z >> wx >> wy >> wz)) continue;

    int flag = 1;
    if (!(iss >> flag)) flag = 1;

    w.pose = {x,y,z,wx,wy,wz};
    w.flag = flag;

    max_abs = std::max(max_abs, std::fabs(wx));
    max_abs = std::max(max_abs, std::fabs(wy));
    max_abs = std::max(max_abs, std::fabs(wz));

    out.push_back(w);
  }

  if (out.empty()) {
    throw std::runtime_error("Waypoint file loaded but empty: " + path);
  }

  rotvec_in_degrees = (max_abs > 6.0);
}

// build indices where flag != 0
inline std::vector<size_t> buildTargetIndices(const std::vector<Waypoint>& wps)
{
  std::vector<size_t> idx;
  for (size_t i=0;i<wps.size();++i) {
    if (wps[i].flag != 0) idx.push_back(i);
  }
  return idx;
}

} // namespace vr_calib
