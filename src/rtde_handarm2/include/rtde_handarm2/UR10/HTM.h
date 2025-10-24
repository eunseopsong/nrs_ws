#ifndef RTDE_HANDARM2_HTM_H_
#define RTDE_HANDARM2_HTM_H_

#include <Eigen/Dense>
#include "rtde_handarm2/UR10/Arm_class.h"
#include "rtde_handarm2/UR10/Kinematics.h"

// =============================================================
//  HTM 계산 함수
// -------------------------------------------------------------
// 입력: 
//   - q : 6×1 joint vector (rad)
//   - tcp_offset : double (EE→TCP z-offset, [m])
// 출력:
//   - T_base_TCP : 4×4 Homogeneous Transform Matrix
// =============================================================
Eigen::Matrix4d HTM(const Eigen::VectorXd& q, double tcp_offset);

#endif  // RTDE_HANDARM2_HTM_H_
