#pragma once

#include <array>
#include <cmath>
#include <Eigen/Dense>

// ================= Constants =================
static constexpr double kPi = 3.14159265358979323846;

static inline double deg2rad(double d){ return d * kPi / 180.0; }
static inline double rad2deg(double r){ return r * 180.0 / kPi; }

static inline double clampd(double x, double lo, double hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ================= Rodrigues (rotvec -> R) =================
static inline void rotvecToRotMatRad(const std::array<double,3>& w,
                                     std::array<double,9>& R)
{
  double th = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
  R = {1,0,0, 0,1,0, 0,0,1};
  if (th < 1e-12) return;

  double ux = w[0]/th, uy = w[1]/th, uz = w[2]/th;
  double s = std::sin(th), c = std::cos(th), v = 1.0 - c;

  R[0] = ux*ux*v + c;
  R[1] = ux*uy*v - uz*s;
  R[2] = ux*uz*v + uy*s;

  R[3] = uy*ux*v + uz*s;
  R[4] = uy*uy*v + c;
  R[5] = uy*uz*v - ux*s;

  R[6] = uz*ux*v - uy*s;
  R[7] = uz*uy*v + ux*s;
  R[8] = uz*uz*v + c;
}

static inline std::array<double,9> matMul3(const std::array<double,9>& A,
                                           const std::array<double,9>& B)
{
  std::array<double,9> C{};
  for (int r=0;r<3;r++){
    for (int c=0;c<3;c++){
      double s=0;
      for (int k=0;k<3;k++) s += A[r*3+k]*B[k*3+c];
      C[r*3+c]=s;
    }
  }
  return C;
}

static inline std::array<double,9> matT3(const std::array<double,9>& A)
{
  return {A[0],A[3],A[6],
          A[1],A[4],A[7],
          A[2],A[5],A[8]};
}

static inline double trace3(const std::array<double,9>& A)
{
  return A[0] + A[4] + A[8];
}

static inline double rotAngleBetweenRad(const std::array<double,9>& R_target,
                                        const std::array<double,9>& R_current)
{
  auto RtT  = matT3(R_target);
  auto Rrel = matMul3(RtT, R_current);
  double cosang = (trace3(Rrel) - 1.0) * 0.5;
  cosang = clampd(cosang, -1.0, 1.0);
  return std::acos(cosang);
}

// ================= helper: Eigen rigid transform =================
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
  Ti.block<3,1>(0,3) = -R.transpose()*p;
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
