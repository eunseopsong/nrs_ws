#ifndef RTDE_HANDARM2_UR10_UR10E_FORWARD_H_
#define RTDE_HANDARM2_UR10_UR10E_FORWARD_H_

#include <Eigen/Dense>
#include "rtde_handarm2/UR10/Arm_class.h"
#include "rtde_handarm2/UR10/Kinematics.h"

// ---------------------------------------------------------------------------
// Base → TCP Forward Kinematics
// ---------------------------------------------------------------------------
// 입력: q (6x1 joint angles), tcp_offset (EE +Z offset [m])
// 출력: Base → TCP Homogeneous Transform Matrix (4x4)
Eigen::Matrix4d ur10e_forward(const Eigen::VectorXd& q, double tcp_offset = 0.0);

#endif  // RTDE_HANDARM2_UR10_FORWARD_H_
