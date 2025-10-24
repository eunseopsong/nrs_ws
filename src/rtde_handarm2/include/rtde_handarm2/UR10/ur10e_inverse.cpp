#include "rtde_handarm2/UR10/ur10e_inverse.h"

using Eigen::Matrix4d;

Eigen::Matrix4d ur10e_inverse(const Eigen::VectorXd& q, double tcp_offset)
{
    // ---- Base → TCP ----
    Matrix4d T_base_TCP = ur10e_forward(q, tcp_offset);

    // ---- TCP → Base (Inverse Transform) ----
    Matrix4d T_TCP_base = T_base_TCP.inverse();

    return T_TCP_base;
}
