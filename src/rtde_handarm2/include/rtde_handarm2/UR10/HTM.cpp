#include "rtde_handarm2/UR10/HTM.h"

Eigen::Matrix4d HTM(const Eigen::VectorXd& q, double tcp_offset)
{
    // ---- Forward Kinematics (Base → EE) ----
    CArm arm;
    Kinematic_func kin;
    arm.qc = q;
    kin.ForwardK_T(&arm); // arm.Tc : base → EE

    // ---- TCP offset 변환 ----
    Eigen::Matrix4d T_tool = Eigen::Matrix4d::Identity();
    T_tool(2, 3) = tcp_offset;

    // ---- Base → TCP ----
    Eigen::Matrix4d T_base_TCP = arm.Tc * T_tool;

    return T_base_TCP;
}
