#include "rtde_handarm2/UR10/Inverse.h"

using Eigen::Matrix4d;

Eigen::Matrix4d Inverse(const Eigen::VectorXd& q, double tcp_offset)
{
    // ---- Base → TCP ----
    Matrix4d T_base_TCP = Forward(q, tcp_offset);

    // ---- TCP → Base (Inverse Transform) ----
    Matrix4d T_TCP_base = T_base_TCP.inverse();

    return T_TCP_base;
}
