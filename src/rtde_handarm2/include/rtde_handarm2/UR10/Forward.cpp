#include "rtde_handarm2/UR10/Forward.h"

using Eigen::Matrix4d;

Eigen::Matrix4d Forward(const Eigen::VectorXd& q, double tcp_offset)
{
    // ---- Base → EE ----
    CArm arm;
    Kinematic_func kin;
    arm.qc = q;
    kin.ForwardK_T(&arm);  // arm.Tc: Base → EE

    // ---- EE → TCP (Offset along +Z of EE frame) ----
    Matrix4d T_tool = Matrix4d::Identity();
    T_tool(2, 3) = tcp_offset;

    // ---- Base → TCP ----
    Matrix4d T_base_TCP = arm.Tc * T_tool;

    return T_base_TCP;
}
