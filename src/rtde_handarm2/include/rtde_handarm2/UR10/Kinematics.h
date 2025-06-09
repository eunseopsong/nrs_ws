////////////////////////////////////////////////////////////
// Kinematics.h
// Kinematics functions
////////////////////////////////////////////////////////////
////////////////// Ver 2.00 ////////////////////////////////
////////////////////////////////////////////////////////////

#ifndef RTDE_HANDARM2_KINEMATIC_FUNC_H_
#define RTDE_HANDARM2_KINEMATIC_FUNC_H_

#include "rtde_handarm2/UR10/Arm_class.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "NRS_yaml_location.h"   // YAML 파일 위치 정의

using namespace Eigen;

#ifndef ZERO_THRESH
#define ZERO_THRESH 1e-8
#endif

#ifndef SIGN
#define SIGN(x) (((x) > 0) - ((x) < 0))
#endif

// 전역 변수는 extern 선언만
extern std::ifstream Kfin1;
extern YAML::Node KNRS_Fcon_setting;

struct Quaternion { double w, x, y, z; };

class Kinematic_func
{
private:
    double s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s6, c6;
    double s23, c23, s34, c34, s234, c234;
    Matrix4d Ycontact_EE2TCP = Matrix4d::Zero();

    double Ycontact_TCP_pos[3];  // TCP offset loaded from YAML
    bool R2E_init_flag = false;
    Vector3d R2E_pre_rpy = Vector3d::Zero();

public:
    Kinematic_func();

    void iForwardK_P(const VectorXd &q, Vector3d &x, double endlength = 0);
    void iForwardK_T(const VectorXd &q, Matrix4d &T, double endlength = 0);
    void ForwardK_P(CArm *A);
    void ForwardK_T(CArm *A);
    void ForwardK_Td(CArm *A);
    void Ycontact_ForwardK_T(CArm *A);
    void Rotation2EulerAngle(CArm *A);
    void Quaternion2Rotation(CArm *A);
    void iRotation2EulerAngle(const Matrix3d &R, Vector3d &th);
    void Rotation2RPY(CArm *A);
    void EulerAngle2Rotation(Matrix3d &R, const Vector3d &th);
    Vector3d VR_Rot2RPY(const Matrix3d &rotationMatrix);

    int InverseK(CArm *qA);
    int Ycontact_InverseK(CArm *qA);
    int InverseK_min(CArm *A);
    int Ycontact_InverseK_min(CArm *A);

    void Jacobian(CArm *A);
    void Jacobian_p(CArm *A);
    void Jacobian_w(CArm *A);

    int sgn(double x);

    Matrix3d RotX(double th);
    Matrix3d RotY(double th);
    Matrix3d RotZ(double th);

    Matrix3d angle_axis_representation(const Vector3d &axis, double angle);
    Matrix3d Qua2Rot(double w, double x, double y, double z);
    Quaterniond Rot2Qua(const Matrix3d &rotationMatrix);
};

typedef Kinematic_func AKfun;

#endif  // RTDE_HANDARM2_KINEMATIC_FUNC_H_
