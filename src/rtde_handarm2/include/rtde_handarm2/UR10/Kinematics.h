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
#include "NRS_yaml_location.h"

using namespace Eigen;

// ---------------------------------------------------------------------
// Legacy macros (fallback)
//  - 실제 동작 값은 YAML (ur10e_spindle_parameter.yaml)의
//    numerical.zero_thresh, numerical.singularity_eps 로부터
//    Kinematic_func 멤버에 로드해서 사용할 예정.
// ---------------------------------------------------------------------
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

typedef class Kinematic_func
{
private:
    // -----------------------------------------------------------------
    // 1) 내부 계산용 삼각함수 캐시
    // -----------------------------------------------------------------
    double s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s6, c6;
    double s23, c23, s34, c34, s234, c234;

    // -----------------------------------------------------------------
    // 2) TCP (Tool Center Point) 오프셋 (EE 기준)
    //    - YAML: tcp.pos[0..2] → Ycontact_TCP_pos[0..2] 에 로드
    //    - EE→TCP 변환행렬: Ycontact_EE2TCP
    // -----------------------------------------------------------------
    Matrix4d Ycontact_EE2TCP;
    double   Ycontact_TCP_pos[3];   // [x, y, z] (EE frame)

    // -----------------------------------------------------------------
    // 3) 수치/연속성 관련 파라미터 (YAML에서 로드)
    //    - numerical.zero_thresh      → zero_thresh_
    //    - numerical.singularity_eps  → singularity_eps_
    //    - rotation_continuity.enable_wrap → rot_enable_wrap_
    //    - rotation_continuity.wrap_2pi    → rot_wrap_2pi_
    // -----------------------------------------------------------------
    double zero_thresh_    = 1.0e-8;  // 기본값: YAML 없을 때 fallback
    double singularity_eps_ = 1.0e-6; // Euler 변환 특이점 판단 기준

    bool   rot_enable_wrap_ = true;   // 연속성 래핑 사용 여부
    bool   rot_wrap_2pi_    = true;   // 음수 각도 시작 시 +2π 할지 여부

    // -----------------------------------------------------------------
    // 4) 회전 연속성 (RPY) 상태
    // -----------------------------------------------------------------
    bool     R2E_init_flag;
    Vector3d R2E_pre_rpy;

    // -----------------------------------------------------------------
    // 5) YAML 파라미터 로더 (Kinematics.cpp 에서 구현)
    // -----------------------------------------------------------------
    void loadYamlParameters();

public:
    Kinematic_func();

    // ROS1 시그니처 유지
    void iForwardK_P(VectorXd &q, Vector3d &x, double endlength = 0);
    void iForwardK_T(VectorXd &q, Matrix4d &T, double endlength = 0);

    void ForwardK_P(CArm *A);
    void ForwardK_T(CArm *A);
    void ForwardK_Td(CArm *A);
    void Ycontact_ForwardK_T(CArm *A);

    void Rotation2EulerAngle(CArm *A);
    void Quaternion2Rotation(CArm *A);
    void iRotation2EulerAngle(Matrix3d &R, Vector3d &th);
    void Rotation2RPY(CArm *A);
    void EulerAngle2Rotation(Matrix3d &R, Vector3d &th);
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

    Matrix3d angle_axis_representation(Vector3d rot_axis, double rot_angle);
    Matrix3d Qua2Rot(double w, double x, double y, double z);
    Quaterniond Rot2Qua(const Matrix3d &rotationMatrix);

    // -----------------------------------------------------------------
    // (선택) 필요하면 YAML 기반 파라미터를 외부에서 확인할 수 있도록
    //   간단한 getter 들 추가해두면 디버깅에 유용함
    // -----------------------------------------------------------------
    inline double zeroThresh()      const { return zero_thresh_; }
    inline double singularityEps()  const { return singularity_eps_; }
    inline bool   enableWrap()      const { return rot_enable_wrap_; }
    inline bool   wrap2Pi()         const { return rot_wrap_2pi_; }

} AKfun;

#endif  // RTDE_HANDARM2_KINEMATIC_FUNC_H_
