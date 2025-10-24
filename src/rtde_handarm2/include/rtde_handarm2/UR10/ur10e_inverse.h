#ifndef RTDE_HANDARM2_UR10_UR10E_INVERSE_H_
#define RTDE_HANDARM2_UR10_UR10E_INVERSE_H_

#include <Eigen/Dense>
#include "rtde_handarm2/UR10/ur10e_forward.h"

// ---------------------------------------------------------------------------
// TCP → Base Transform (Inverse of Forward Kinematics)
// ---------------------------------------------------------------------------
// 입력: q (6x1 joint angles), tcp_offset (EE +Z offset [m])
// 출력: TCP → Base Homogeneous Transform Matrix (4x4)
Eigen::Matrix4d ur10e_inverse(const Eigen::VectorXd& q, double tcp_offset = 0.0);

#endif  // RTDE_HANDARM2_UR10_INVERSE_H_
