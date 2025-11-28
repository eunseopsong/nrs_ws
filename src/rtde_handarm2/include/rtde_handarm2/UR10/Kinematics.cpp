////////////////////////////////////////////////////////////
// Kinematics.cpp
// Kinematics functions for UR-type 6-DOF manipulator
//
// Original creation : 2013-12-24
// Major refactor    : 2025-11-27
//   - UR10e, TCP calibration, contact TCP, cleanup
//   - YAML 기반 파라미터 로딩 추가
////////////////////////////////////////////////////////////
//
// 1. 목적(Purpose)
// --------------------------------------------------------
// 이 파일은 UR10 / UR10e 스타일 6자유도 매니퓰레이터에 대한
//   - 정기구학(Forward Kinematics)
//   - 역기구학(Inverse Kinematics, 최대 8해)
//   - 기하 자코비안(Geometric Jacobian)
//   - 회전/쿼터니언/각-축(angle-axis) 변환
// 을 제공하는 순수 수학 계층이다.
//
// 모든 수식은 "관절공간 q = [q1 ... q6]^T" 에서
//   - EE(End-Effector, 플랜지)의 위치/자세 x(q), R(q)
//   - TCP(Tool Center Point)의 위치/자세 x_TCP(q), R_TCP(q)
//   - 자코비안 J(q) = [ Jp(q); Jw(q) ]
// 을 계산하는 것을 목표로 한다.
//
// 제어 파이프라인의 상위 계층(Admittance control, Cartesian PD 등)
// 은 이 파일에서 제공하는 x(q), R(q), J(q) 를 이용해
// 원하는 작업공간 궤적을 구현한다.
//
//
// 2. 좌표계와 수식 요약
// --------------------------------------------------------
// - q ∈ R^6 : 조인트 각도 벡터 (단위 rad)
// - T_0^6(q) ∈ SE(3) : Base(0) → EE(6) 변환 행렬
// - R_0^6(q) ∈ SO(3) : T_0^6(q)의 상위 3x3 회전 행렬
// - p_0^6(q) ∈ R^3  : T_0^6(q)의 상위 3x1 위치 벡터
// - T_0^TCP(q) = T_0^6(q) * T_6^TCP : EE에서 TCP로의 추가 변환 포함
// - x(q) = p_0^TCP(q) : TCP의 base 좌표 위치
// - rpy(q) : R_0^TCP(q)를 Roll-Pitch-Yaw(Euler)로 변환한 값
//
// 자코비안:
//   - Jp(q) = ∂x/∂q ∈ R^(3x6)  : TCP 위치에 대한 조인트 속도의 선속도 매핑
//   - Jw(q) ∈ R^(3x6)          : TCP 회전축(각속도)에 대한 매핑
//   - J(q)  ∈ R^(6x6)          : [ Jp(q); Jw(q) ]
//
// 역기구학:
//   - InverseK : Td(=T_0^TCP_des) 가 주어졌을 때
//                q 를 해석적으로 0~8개까지 구해 qA->q 에 저장.
//   - InverseK_min : 위에서 구한 해 중에서 현재 qc 에 가장 가까운 해를 선택.
//
//
// 3. 주요 함수별 역할(수식 관점 설명)
// --------------------------------------------------------
// [1] iForwardK_P(VectorXd &q, Vector3d &x, double endlength)
//  - 입력: 조인트 각도 q, 툴 길이 endlength
//  - 출력: x = p_0^TCP(q) (현재 조인트에서의 TCP 위치)
//  - 실제로는:
//      1) UR-타입 DH/기하 모델을 이용해서 EE(플랜지) 위치 p_0^6(q)를 계산.
//      2) 조인트 6 축 방향으로 endlength 만큼 연장 (T_6^TCP 의 z축 이동).
//
// [2] iForwardK_T(VectorXd &q, Matrix4d &T, double endlength)
//  - 입력: q, endlength
//  - 출력: T = T_0^TCP(q)
//
// [3] ForwardK_P(CArm *A)
//  - 플랜지 기준 위치 FK (TCP 오프셋 미포함)
//
// [4] ForwardK_Td(CArm *A)
//  - qd 기준 플랜지 변환 Td, 위치 xd
//
// [5] ForwardK_T(CArm *A)
//  - qc 기준 플랜지 변환 Tc, 위치 xc
//
// [6] Ycontact_ForwardK_T(CArm *A)
//  - EE 기준 FK(A->Tc) 계산 후,
//    내부 EE→TCP 변환행렬(Ycontact_EE2TCP, Ycontact_TCP_pos[0..2])을 곱해서
//    실제 접촉 TCP (스핀들 끝) 위치/자세를 구한다.
//  - 여기서 스핀들 길이 = Ycontact_TCP_pos[2] (현재 기본값 0.185 m)
// ------------------------------------------------------------

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <string>
#include <iostream>

#include "Arm_class.h"
#include "Kinematics.h"

// -----------------------------------------------------------------------------
// 로컬 유틸: UR10e spindle parameter YAML 경로 결정
//   - NRS_yaml_location.h 안에 매크로가 정의되어 있으면 그걸 사용하고,
//   - 없으면 HOME 기반 기본 경로로 폴백.
// -----------------------------------------------------------------------------
namespace {
std::string getSpindleYamlPath()
{
#ifdef NRS_UR10E_SPINDLE_YAML_PATH
    // NRS_yaml_location.h 에서 정의해둔 매크로 사용 (예: "/home/.../ur10e_spindle_parameter.yaml")
    return std::string(NRS_UR10E_SPINDLE_YAML_PATH);
#else
    const char* home = std::getenv("HOME");
    std::string base = home ? std::string(home) : std::string();
    if (!base.empty()) {
        return base + "/nrs_ws/src/rtde_handarm2/yaml/ur10e_spindle_parameter.yaml";
    } else {
        // 최후 폴백: 현재 워킹 디렉토리 기준 상대 경로
        return std::string("./yaml/ur10e_spindle_parameter.yaml");
    }
#endif
}
} // anonymous namespace

// -----------------------------------------------------------------------------
// YAML 파라미터 로드
//  - 파일: ur10e_spindle_parameter.yaml
//    tcp.pos[3]              → Ycontact_TCP_pos[0..2]
//    numerical.zero_thresh   → zero_thresh_
//    numerical.singularity_eps → singularity_eps_
//    rotation_continuity.*   → rot_enable_wrap_, rot_wrap_2pi_
// -----------------------------------------------------------------------------
void Kinematic_func::loadYamlParameters()
{
    // 기본값 세팅 (YAML 실패 시 사용)
    Ycontact_TCP_pos[0] = 0.0;
    Ycontact_TCP_pos[1] = 0.0;
    Ycontact_TCP_pos[2] = 0.185;  // 185 mm spindle length along tool Z

    Ycontact_EE2TCP.setIdentity();
    Ycontact_EE2TCP(0,3) = Ycontact_TCP_pos[0];
    Ycontact_EE2TCP(1,3) = Ycontact_TCP_pos[1];
    Ycontact_EE2TCP(2,3) = Ycontact_TCP_pos[2];

    // numerical / rotation_continuity는 헤더의 기본값(zero_thresh_, singularity_eps_,
    // rot_enable_wrap_, rot_wrap_2pi_)을 우선 사용하고, YAML이 있으면 덮어쓴다.
    const std::string path = getSpindleYamlPath();

    try {
        YAML::Node root = YAML::LoadFile(path);

        // 1) TCP pos
        if (root["tcp"] && root["tcp"]["pos"] && root["tcp"]["pos"].IsSequence()
            && root["tcp"]["pos"].size() >= 3) {
            for (int i = 0; i < 3; ++i) {
                Ycontact_TCP_pos[i] = root["tcp"]["pos"][i].as<double>();
            }
        }

        // EE->TCP 변환행렬 업데이트 (rotation은 현재 사용하지 않고 translation만 반영)
        Ycontact_EE2TCP.setIdentity();
        Ycontact_EE2TCP(0,3) = Ycontact_TCP_pos[0];
        Ycontact_EE2TCP(1,3) = Ycontact_TCP_pos[1];
        Ycontact_EE2TCP(2,3) = Ycontact_TCP_pos[2];

        // 2) numerical 파라미터
        if (root["numerical"]) {
            auto num = root["numerical"];
            if (num["zero_thresh"]) {
                zero_thresh_ = num["zero_thresh"].as<double>();
            }
            if (num["singularity_eps"]) {
                singularity_eps_ = num["singularity_eps"].as<double>();
            }
        }

        // 3) rotation_continuity 파라미터
        if (root["rotation_continuity"]) {
            auto rot = root["rotation_continuity"];
            if (rot["enable_wrap"]) {
                rot_enable_wrap_ = rot["enable_wrap"].as<bool>();
            }
            if (rot["wrap_2pi"]) {
                rot_wrap_2pi_ = rot["wrap_2pi"].as<bool>();
            }
        }

        std::cout << "[Kinematic_func] Loaded spindle/tcp parameters from: "
                  << path << std::endl;
        std::cout << "  tcp.pos = [" << Ycontact_TCP_pos[0] << ", "
                  << Ycontact_TCP_pos[1] << ", "
                  << Ycontact_TCP_pos[2] << "]" << std::endl;
        std::cout << "  numerical.zero_thresh    = " << zero_thresh_ << std::endl;
        std::cout << "  numerical.singularity_eps= " << singularity_eps_ << std::endl;
        std::cout << "  rotation_continuity.enable_wrap = "
                  << (rot_enable_wrap_ ? "true" : "false") << std::endl;
        std::cout << "  rotation_continuity.wrap_2pi    = "
                  << (rot_wrap_2pi_ ? "true" : "false") << std::endl;
    }
    catch (const std::exception& e) {
        // YAML 로딩 실패: 기본값 유지
        std::cerr << "[Kinematic_func] WARNING: Failed to load YAML '"
                  << path << "': " << e.what() << std::endl;
        std::cerr << "  -> Using default TCP pos [0,0,0.185] and numerical settings."
                  << std::endl;
    }
}

// -----------------------------------------------------------------------------
// Kinematic_func 생성자
//  - R2E_init_flag / R2E_pre_rpy: VR_Rot2RPY 연속성 보정용 상태
//  - YAML 파라미터 로드 후, Ycontact_TCP_pos / zero_thresh_ / singularity_eps_ 등 설정
// -----------------------------------------------------------------------------
Kinematic_func::Kinematic_func()
{
    // 회전 연속성 내부 상태 초기화
    R2E_init_flag = false;
    R2E_pre_rpy.setZero();

    // YAML 기반 파라미터 로드
    loadYamlParameters();
}

// =================== Kinematics and Dynamic parameters independent ===================//

void Kinematic_func::iForwardK_P(VectorXd &q, Vector3d &x, double endlength)
{
    // Input = Joint Angle q, additional end length endlength
    // Output = Current Position x (with endlength along joint-6)

    double d6a = d6 + endlength;

    s1 = sin(q(0));
    c1 = cos(q(0));
    s2 = sin(q(1));
    c2 = cos(q(1));
    s3 = sin(q(2));
    c3 = cos(q(2));
    s4 = sin(q(3));
    c4 = cos(q(3));
    s5 = sin(q(4));
    c5 = cos(q(4));
    s6 = sin(q(5));
    c6 = cos(q(5));
    
    s234 = sin(q(1) + q(2) + q(3));
    c234 = cos(q(1) + q(2) + q(3));

    x(0) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1
             + (d6a*(c1*c234-s1*s234)*s5)/2.0 + (d6a*(c1*c234+s1*s234)*s5)/2.0
             - a2*c1*c2 - d6a*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);

    x(1) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1
             + (d6a*(s1*c234+c1*s234)*s5)/2.0 + (d6a*(s1*c234-c1*s234)*s5)/2.0
             + d6a*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);

    x(2) =  (d1 + (d6a*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3)
             + a2*s2 - (d6a*(c234*c5+s234*s5))/2.0 - d5*c234);
}

void Kinematic_func::iForwardK_T(VectorXd &q, Matrix4d &T, double endlength)
{
    // Input = Joint Angle q, additional end length endlength
    // Output = Transformation Matrix T (with endlength on joint-6)

    double d6a = d6 + endlength;

    s1 = sin(q(0));
    c1 = cos(q(0));
    s2 = sin(q(1));
    c2 = cos(q(1));
    s3 = sin(q(2));
    c3 = cos(q(2));
    s4 = sin(q(3));
    c4 = cos(q(3));
    s5 = sin(q(4));
    c5 = cos(q(4));
    s6 = sin(q(5));
    c6 = cos(q(5));
    
    s234 = sin(q(1) + q(2) + q(3));
    c234 = cos(q(1) + q(2) + q(3));

    // 회전부: d6/endlength와 무관
    T(0,0) = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0)
              - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
    T(1,0) = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0)
              + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
    T(2,0) = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
    T(3,0) = 0;

    T(0,1) = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0
              - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
    T(1,1) = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0)
              - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
    T(2,1) = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
    T(3,1) = 0;

    T(0,2) = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
    T(1,2) = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
    T(2,2) = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
    T(3,2) = 0;

    // 위치부: d6 → d6a 로 변경 (endlength 반영)
    T(0,3) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1
               + (d6a*(c1*c234-s1*s234)*s5)/2.0 + (d6a*(c1*c234+s1*s234)*s5)/2.0
               - a2*c1*c2 - d6a*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
    T(1,3) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1
               + (d6a*(s1*c234+c1*s234)*s5)/2.0 + (d6a*(s1*c234-c1*s234)*s5)/2.0
               + d6a*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
    T(2,3) =  (d1 + (d6a*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3)
               + a2*s2 - (d6a*(c234*c5+s234*s5))/2.0 - d5*c234);
    T(3,3) = 1;
}


// =================== Kinematics and Dynamic parameters with Arm Class ===================//

void Kinematic_func::ForwardK_P(CArm *A)
{
    // Input = Current Joint Angle qc
    // Output = Current Position xc  (flange FK, no additional TCP offset here)

    s1 = sin(A->qc(0));
    c1 = cos(A->qc(0));
    s2 = sin(A->qc(1));
    c2 = cos(A->qc(1));
    s3 = sin(A->qc(2));
    c3 = cos(A->qc(2));
    s4 = sin(A->qc(3));
    c4 = cos(A->qc(3));
    s5 = sin(A->qc(4));
    c5 = cos(A->qc(4));
    s6 = sin(A->qc(5));
    c6 = cos(A->qc(5));
    
    s234 = sin(A->qc(1) + A->qc(2) + A->qc(3));
    c234 = cos(A->qc(1) + A->qc(2) + A->qc(3));

    A->xc(0) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1
                 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0
                 - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
    A->xc(1) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1
                 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0
                 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
    A->xc(2) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3)
                 + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
}

void Kinematic_func::ForwardK_Td(CArm *A)
{
    // Input = Desire Joint Angle qd
    // Output = Desire Transform Matrix Td, Desire Position xd (flange-based)

    s1 = sin(A->qd(0));
    c1 = cos(A->qd(0));
    s2 = sin(A->qd(1));
    c2 = cos(A->qd(1));
    s3 = sin(A->qd(2));
    c3 = cos(A->qd(2));
    s4 = sin(A->qd(3));
    c4 = cos(A->qd(3));
    s5 = sin(A->qd(4));
    c5 = cos(A->qd(4));
    s6 = sin(A->qd(5));
    c6 = cos(A->qd(5));

    // s234, c234 는 qd 기준으로 계산
    s234 = sin(A->qd(1) + A->qd(2) + A->qd(3));
    c234 = cos(A->qd(1) + A->qd(2) + A->qd(3));

    A->Td(0,0) = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0)
                  - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
    A->Td(1,0) = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0)
                  + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
    A->Td(2,0) = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
    A->Td(3,0) = 0;

    A->Td(0,1) = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0
                  - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
    A->Td(1,1) = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0)
                  - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
    A->Td(2,1) = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
    A->Td(3,1) = 0;

    A->Td(0,2) = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
    A->Td(1,2) = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
    A->Td(2,2) = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
    A->Td(3,2) = 0;

    A->Td(0,3) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1
                   + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0
                   - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
    A->Td(1,3) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1
                   + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0
                   + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
    A->Td(2,3) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3)
                   + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
    A->Td(3,3) = 1;

    A->xd(0) = A->Td(0,3);
    A->xd(1) = A->Td(1,3);
    A->xd(2) = A->Td(2,3);
}

void Kinematic_func::ForwardK_T(CArm *A)
{
    // Input = Current Joint Angle qc
    // Output = Current Transform Matrix Tc, Current Position xc (flange-based)

    s1 = sin(A->qc(0));
    c1 = cos(A->qc(0));
    s2 = sin(A->qc(1));
    c2 = cos(A->qc(1));
    s3 = sin(A->qc(2));
    c3 = cos(A->qc(2));
    s4 = sin(A->qc(3));
    c4 = cos(A->qc(3));
    s5 = sin(A->qc(4));
    c5 = cos(A->qc(4));
    s6 = sin(A->qc(5));
    c6 = cos(A->qc(5));

    s23 = sin(A->qc(1) + A->qc(2));
    c23 = cos(A->qc(1) + A->qc(2));

    s34 = sin(A->qc(2) + A->qc(3));
    c34 = cos(A->qc(2) + A->qc(3));    
    
    s234 = sin(A->qc(1) + A->qc(2) + A->qc(3));
    c234 = cos(A->qc(1) + A->qc(2) + A->qc(3));

    A->Tc(0,0) = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0)
                  - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
    A->Tc(1,0) = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0)
                  + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
    A->Tc(2,0) = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
    A->Tc(3,0) = 0;

    A->Tc(0,1) = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0
                  - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
    A->Tc(1,1) = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0)
                  - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
    A->Tc(2,1) = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
    A->Tc(3,1) = 0;

    A->Tc(0,2) = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
    A->Tc(1,2) = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
    A->Tc(2,2) = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
    A->Tc(3,2) = 0;

    A->Tc(0,3) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1
                   + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0
                   - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
    A->Tc(1,3) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1
                   + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0
                   + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
    A->Tc(2,3) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3)
                   + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
    A->Tc(3,3) = 1;

    A->xc(0) = A->Tc(0,3);
    A->xc(1) = A->Tc(1,3);
    A->xc(2) = A->Tc(2,3);
}

void Kinematic_func::Ycontact_ForwardK_T(CArm *A)
{
    // Input = Current Joint Angle qc
    // Output = Current Transform Matrix Tc, Current Position xc
    //          including EE→TCP offset (Ycontact_TCP_pos)
    //
    // A->Tc : Base → EE
    // Ycontact_EE2TCP : EE → TCP
    // A->Tc * Ycontact_EE2TCP : Base → TCP

    // EE -> TCP transform (tool/contact offset)
    this->Ycontact_EE2TCP <<  1, 0, 0, this->Ycontact_TCP_pos[0],
                              0, 1, 0, this->Ycontact_TCP_pos[1],
                              0, 0, 1, this->Ycontact_TCP_pos[2],
                              0, 0, 0, 1;

    s1 = sin(A->qc(0));
    c1 = cos(A->qc(0));
    s2 = sin(A->qc(1));
    c2 = cos(A->qc(1));
    s3 = sin(A->qc(2));
    c3 = cos(A->qc(2));
    s4 = sin(A->qc(3));
    c4 = cos(A->qc(3));
    s5 = sin(A->qc(4));
    c5 = cos(A->qc(4));
    s6 = sin(A->qc(5));
    c6 = cos(A->qc(5));

    s23 = sin(A->qc(1) + A->qc(2));
    c23 = cos(A->qc(1) + A->qc(2));

    s34 = sin(A->qc(2) + A->qc(3));
    c34 = cos(A->qc(2) + A->qc(3));    
    
    s234 = sin(A->qc(1) + A->qc(2) + A->qc(3));
    c234 = cos(A->qc(1) + A->qc(2) + A->qc(3));

    A->Tc(0,0) = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0)
                  - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
    A->Tc(1,0) = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0)
                  + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
    A->Tc(2,0) = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
    A->Tc(3,0) = 0;

    A->Tc(0,1) = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0
                  - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
    A->Tc(1,1) = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0)
                  - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
    A->Tc(2,1) = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
    A->Tc(3,1) = 0;

    A->Tc(0,2) = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
    A->Tc(1,2) = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
    A->Tc(2,2) = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
    A->Tc(3,2) = 0;

    A->Tc(0,3) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1
                   + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0
                   - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
    A->Tc(1,3) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1
                   + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0
                   + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
    A->Tc(2,3) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3)
                   + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
    A->Tc(3,3) = 1;

    // Base->EE 에 EE->TCP 오프셋 곱해서 Base->TCP 계산
    A->Tc = A->Tc * this->Ycontact_EE2TCP;

    A->xc(0) = A->Tc(0,3);
    A->xc(1) = A->Tc(1,3);
    A->xc(2) = A->Tc(2,3);
}

void Kinematic_func::Quaternion2Rotation(CArm *A)
{
    A->QuatM(0,0) = 1 - 2*A->Quat[1]*A->Quat[1] - 2*A->Quat[2]*A->Quat[2];
    A->QuatM(0,1) = 2*A->Quat[0]*A->Quat[1] - 2*A->Quat[3]*A->Quat[2];
    A->QuatM(0,2) = 2*A->Quat[0]*A->Quat[2] + 2*A->Quat[3]*A->Quat[1];

    A->QuatM(1,0) = 2*A->Quat[0]*A->Quat[1] + 2*A->Quat[3]*A->Quat[2]; 
    A->QuatM(1,1) = 1 - 2*A->Quat[0]*A->Quat[0] - 2*A->Quat[2]*A->Quat[2];
    A->QuatM(1,2) = 2*A->Quat[1]*A->Quat[2] - 2*A->Quat[3]*A->Quat[0];

    A->QuatM(2,0) = 2*A->Quat[0]*A->Quat[2] - 2*A->Quat[3]*A->Quat[1];
    A->QuatM(2,1) = 2*A->Quat[1]*A->Quat[2] + 2*A->Quat[3]*A->Quat[0];
    A->QuatM(2,2) = 1 - 2*A->Quat[0]*A->Quat[0] - 2*A->Quat[1]*A->Quat[1];

    A->QuatM4(0,0) = A->QuatM(0,0);
    A->QuatM4(0,1) = A->QuatM(0,1);
    A->QuatM4(0,2) = A->QuatM(0,2);

    A->QuatM4(1,0) = A->QuatM(1,0); 
    A->QuatM4(1,1) = A->QuatM(1,1);
    A->QuatM4(1,2) = A->QuatM(1,2);

    A->QuatM4(2,0) = A->QuatM(2,0);
    A->QuatM4(2,1) = A->QuatM(2,1);
    A->QuatM4(2,2) = A->QuatM(2,2);

    A->QuatM4(3,0) = 0;
    A->QuatM4(3,1) = 0;
    A->QuatM4(3,2) = 0;
    A->QuatM4(3,3) = 1;
}

void Kinematic_func::Rotation2EulerAngle(CArm *A)
{
    // Input = Current Rotation Matrix Tc
    // Output = Current Euler Angle thc (RPY)

    double orig, orig_PL, orig_MI;

    float sy = std::sqrt(A->Tc(0,0) * A->Tc(0,0) +  A->Tc(1,0) * A->Tc(1,0) );
 
    bool singular = sy < singularity_eps_; // YAML에서 로드된 임계값 사용
 
    if (!singular)
    {
        A->thc(0) = atan2(A->Tc(2,1) , A->Tc(2,2));
        A->thc(1) = atan2(-A->Tc(2,0), sy);
        A->thc(2) = atan2(A->Tc(1,0), A->Tc(0,0));
    }
    else
    {
        A->thc(0) = atan2(-A->Tc(1,2), A->Tc(1,1));
        A->thc(1) = atan2(-A->Tc(2,0), sy);
        A->thc(2) = 0;
    }

    // ===== 연속성/래핑 옵션 적용 =====
    if (rot_wrap_2pi_) {
        // Step1: Minus → Plus 2π (시작점이 ±180° 근처일 때)
        if (A->thc(0) < 0) A->thc(0) += 2*PI;
    }

    if (rot_enable_wrap_) {
        if(A->R2E_init_flag == false)
        {
            A->pre_thc = A->thc;
            A->R2E_init_flag = true;
        }
        
        for(int i=0;i<3;i++)
        {
            // Step2: 이전 각도와 가장 가까운 branch 선택
            orig    = fabs(A->thc(i)           - A->pre_thc(i));
            orig_PL = fabs(A->thc(i) + 2*PI    - A->pre_thc(i));
            orig_MI = fabs(A->thc(i) - 2*PI    - A->pre_thc(i));

            if((orig <= orig_PL) && (orig <= orig_MI)) {
                // 그대로
            }
            else if((orig_PL <= orig) && (orig_PL <= orig_MI)) {
                A->thc(i) = A->thc(i)+2*PI;
            }
            else {
                A->thc(i) = A->thc(i)-2*PI;
            }
        }

        // data backup
        A->pre_thc = A->thc;
    }
}

void Kinematic_func::iRotation2EulerAngle(Matrix3d &R, Vector3d &th)
{
    // Input = Rotation Matrix R
    // Output = Euler Angle th

    th(1) = atan2(-R(2,0), sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)));
    float cb = cos(th(1));
    th(2) = atan2(R(1,0)/cb, R(0,0)/cb);
    th(0) = atan2(R(2,1)/cb, R(2,2)/cb);
}

void Kinematic_func::EulerAngle2Rotation(Matrix3d &R, Vector3d &th)
{
    R(0,0) = cos(th(2))*cos(th(1));
    R(0,1) = -sin(th(2))*cos(th(0))+cos(th(2))*sin(th(1))*sin(th(0));
    R(0,2) = sin(th(2))*sin(th(0))+cos(th(2))*sin(th(1))*cos(th(0));

    R(1,0) = sin(th(2))*cos(th(1));
    R(1,1) = cos(th(2))*cos(th(0))+sin(th(2))*sin(th(1))*sin(th(0));
    R(1,2) = -cos(th(2))*sin(th(0))+sin(th(2))*sin(th(1))*cos(th(0));

    R(2,0) = -sin(th(1));
    R(2,1) = cos(th(1))*sin(th(0));
    R(2,2) = cos(th(1))*cos(th(0));
}

Vector3d Kinematic_func::VR_Rot2RPY(const Matrix3d& rotationMatrix) 
{
    Vector3d rpy;
    double orig,orig_PL,orig_MI;

    // Check for singularity at rpy(2) = +-pi/2
    if (rotationMatrix(2, 0) > 0.998) { // singularity at north pole
        rpy(0) = atan2(rotationMatrix(0, 1), rotationMatrix(1, 1)); // ROLL
        rpy(1) = M_PI / 2.0; // PITCH
        rpy(2) = 0; // YAW
    } else if (rotationMatrix(2, 0) < -0.998) { // singularity at south pole
        rpy(0) = atan2(rotationMatrix(0, 1), rotationMatrix(1, 1)); // ROLL
        rpy(1) = -M_PI / 2.0; // PITCH
        rpy(2) = 0; // YAW
    } else {
        rpy(0) = atan2(-rotationMatrix(1, 0), rotationMatrix(0, 0)); // ROLL
        rpy(1) = asin(rotationMatrix(2, 0)); // PITCH
        rpy(2) = atan2(-rotationMatrix(2, 1), rotationMatrix(2, 2)); // YAW
    }

    // 연속성/래핑 옵션 반영 (UR10 VR용)
    if (rot_wrap_2pi_) {
        if(rpy(0) < 0) { rpy(0) += 2*PI; }
    }

    if (rot_enable_wrap_) {
        if(this->R2E_init_flag == false)
        {
            this->R2E_pre_rpy = rpy;
            this->R2E_init_flag = true;
        }
        
        for(int i=0;i<3;i++)
        {
            orig    = fabs(rpy(i)           - this->R2E_pre_rpy(i));
            orig_PL = fabs(rpy(i)+2*PI      - this->R2E_pre_rpy(i));
            orig_MI = fabs(rpy(i)-2*PI      - this->R2E_pre_rpy(i));

            if((orig <= orig_PL) && (orig <= orig_MI)) {
                // 그대로
            }
            else if((orig_PL <= orig) && (orig_PL <= orig_MI)) {
                rpy(i) = rpy(i)+2*PI;
            }
            else {
                rpy(i) = rpy(i)-2*PI;
            }
        }

        // data backup
        this->R2E_pre_rpy = rpy;
    }

    return rpy;
}

void Kinematic_func::Rotation2RPY(CArm *A)
{
    A->rpyc(2) = atan2(A->Tc(1,0),A->Tc(0,0));
    float ca = cos(A->rpyc(2));
    float sa = sin(A->rpyc(2));
    A->rpyc(1) = atan2(-A->Tc(2,0),(A->Tc(0,0)*ca+A->Tc(1,0)*sa));
    A->rpyc(0) = atan2((-A->Tc(1,2)*ca-A->Tc(0,2)*sa),(A->Tc(1,1)*ca-A->Tc(0,1)*sa));
}

int Kinematic_func::sgn(double x)
{
    if(x>0) return 1;
    if(x<0) return -1;
    if(x==0) return 0;
    return 0;
}

// ------------------------- IK & Jacobian -------------------------

int Kinematic_func::InverseK(CArm *qA)
{
    // Input = Desired Transform Matrix Td
    // Output = Joint Angle q
    
    int num_sols = 0;

    double T00 =  qA->Td(0,0);
    double T10 =  qA->Td(1,0); 
    double T20 =  qA->Td(2,0); 
    double T30 =  qA->Td(3,0); 

    double T01 =  qA->Td(0,1); 
    double T11 =  qA->Td(1,1); 
    double T21 =  qA->Td(2,1); 
    double T31 =  qA->Td(3,1); 

    double T02 =  qA->Td(0,2); 
    double T12 =  qA->Td(1,2); 
    double T22 =  qA->Td(2,2); 
    double T32 =  qA->Td(3,2); 

    double T03 =  qA->Td(0,3); 
    double T13 =  qA->Td(1,3); 
    double T23 =  qA->Td(2,3); 
    double T33 =  qA->Td(3,3); 

    
    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
      double A = d6*T12 - T13;
      double B = d6*T02 - T03;
      double R = A*A + B*B;
      if(fabs(A) < zero_thresh_) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < zero_thresh_)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < zero_thresh_)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < zero_thresh_) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < zero_thresh_)
          div = SIGN(d4)*SIGN(A);
        else
          div = d4/A;
        double arccos = acos(div);
        q1[0] = arccos;
        q1[1] = 2.0*PI - arccos;
      }
      else if(d4*d4 > R) {
        return num_sols;
      }
      else {
        double arccos = acos(d4 / sqrt(R)) ;
        double arctan = atan2(-B, A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if(fabs(pos) < zero_thresh_)
          pos = 0.0;
        if(fabs(neg) < zero_thresh_)
          neg = 0.0;
        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*PI + pos;
        if(neg >= 0.0)
          q1[1] = neg; 
        else
          q1[1] = 2.0*PI + neg;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
      for(int i=0;i<2;i++) {
        double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
        double div;
        if(fabs(fabs(numer) - fabs(d6)) < zero_thresh_)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = acos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0*PI - arccos;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
      for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
          double c1 = cos(q1[i]), s1 = sin(q1[i]);
          double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(fabs(s5) < zero_thresh_)
            q6 = 0;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                       SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < zero_thresh_)
              q6 = 0.0;
            if(q6 < 0.0)
              q6 += 2.0*PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double q2[2], q3[2], q4[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = cos(q6), s6 = sin(q6);
          double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + 
                        T03*c1 + T13*s1;
          double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

          double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
          if(fabs(fabs(c3) - 1.0) < zero_thresh_)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // NO SOLUTION
            continue;
          }
          double arccos = acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0*PI - arccos;
          double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
          double s3 = sin(arccos);
          double A = (a2 + a3*c3), B = a3*s3;
          q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
          q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
          double c23_0 = cos(q2[0]+q3[0]);
          double s23_0 = sin(q2[0]+q3[0]);
          double c23_1 = cos(q2[1]+q3[1]);
          double s23_1 = sin(q2[1]+q3[1]);
          q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
          ////////////////////////////////////////////////////////////////////////////////
          for(int k=0;k<2;k++) {
            if(fabs(q2[k]) < zero_thresh_)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < zero_thresh_)
              q4[k] = 0.0;
            else if(q4[k] < 0.0) q4[k] += 2.0*PI;

            qA->q(num_sols*6+0) = q1[i];    
            qA->q(num_sols*6+1) = q2[k]; 
            qA->q(num_sols*6+2) = q3[k];    
            qA->q(num_sols*6+3) = q4[k]; 
            qA->q(num_sols*6+4) = q5[i][j]; 
            qA->q(num_sols*6+5) = q6;
            num_sols++;
          }
        }
      }
    }
    return num_sols;
}

int Kinematic_func::InverseK_min(CArm *A){

  int ret;
  if(ret=InverseK(A)){
    double minerrsum=100000000;
    int idx = 0;
    for(int i=0;i<ret;i++){
      double errsum=0;
      for(int j=0;j<6;j++){
        double qother;
        if(A->q(i*6+j)>0)
            qother=A->q(i*6+j)-2*PI;
        else if(A->q(i*6+j)<0)
            qother=A->q(i*6+j)+2*PI;
        else
            qother=A->q(i*6+j);

        double err1=A->qc(j)-qother;
        double err=A->qc(j)-A->q(i*6+j);
        if((err1*err1)<(err*err)){
            A->q(i*6+j)=qother;
            err=err1;
        }
        errsum += (err*err);
      }
      if(minerrsum>errsum){
        minerrsum=errsum;
        idx=i;
      }
    }
    for(int i=0;i<6;i++)
      A->qd(i)=A->q(idx*6+i); // output of inverse kinematics
  }
  return ret;
}

int Kinematic_func::Ycontact_InverseK(CArm *qA)
{
    // Input = Desired Transform Matrix Td (TCP 기준)
    // Output = Joint Angle q (EE 기준 IK 수행)

    this->Ycontact_EE2TCP <<  1, 0, 0, this->Ycontact_TCP_pos[0],
                              0, 1, 0, this->Ycontact_TCP_pos[1],
                              0, 0, 1, this->Ycontact_TCP_pos[2],
                              0, 0, 0, 1;

    // Td: Base->TCP  →  Base->EE 로 변환
    qA->Td = qA->Td*this->Ycontact_EE2TCP.inverse();

    int num_sols = 0;

    double T00 =  qA->Td(0,0);
    double T10 =  qA->Td(1,0); 
    double T20 =  qA->Td(2,0); 
    double T30 =  qA->Td(3,0); 

    double T01 =  qA->Td(0,1); 
    double T11 =  qA->Td(1,1); 
    double T21 =  qA->Td(2,1); 
    double T31 =  qA->Td(3,1); 

    double T02 =  qA->Td(0,2); 
    double T12 =  qA->Td(1,2); 
    double T22 =  qA->Td(2,2); 
    double T32 =  qA->Td(3,2); 

    double T03 =  qA->Td(0,3); 
    double T13 =  qA->Td(1,3); 
    double T23 =  qA->Td(2,3); 
    double T33 =  qA->Td(3,3); 

    
    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
      double A = d6*T12 - T13;
      double B = d6*T02 - T03;
      double R = A*A + B*B;
      if(fabs(A) < zero_thresh_) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < zero_thresh_)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < zero_thresh_)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < zero_thresh_) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < zero_thresh_)
          div = SIGN(d4)*SIGN(A);
        else
          div = d4/A;
        double arccos = acos(div);
        q1[0] = arccos;
        q1[1] = 2.0*PI - arccos;
      }
      else if(d4*d4 > R) {
        return num_sols;
      }
      else {
        double arccos = acos(d4 / sqrt(R)) ;
        double arctan = atan2(-B, A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if(fabs(pos) < zero_thresh_)
          pos = 0.0;
        if(fabs(neg) < zero_thresh_)
          neg = 0.0;
        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*PI + pos;
        if(neg >= 0.0)
          q1[1] = neg; 
        else
          q1[1] = 2.0*PI + neg;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
      for(int i=0;i<2;i++) {
        double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
        double div;
        if(fabs(fabs(numer) - fabs(d6)) < zero_thresh_)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = acos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0*PI - arccos;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
      for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
          double c1 = cos(q1[i]), s1 = sin(q1[i]);
          double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(fabs(s5) < zero_thresh_)
            q6 = 0;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                       SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < zero_thresh_)
              q6 = 0.0;
            if(q6 < 0.0)
              q6 += 2.0*PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double q2[2], q3[2], q4[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = cos(q6), s6 = sin(q6);
          double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + 
                        T03*c1 + T13*s1;
          double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

          double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
          if(fabs(fabs(c3) - 1.0) < zero_thresh_)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // NO SOLUTION
            continue;
          }
          double arccos = acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0*PI - arccos;
          double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
          double s3 = sin(arccos);
          double A = (a2 + a3*c3), B = a3*s3;
          q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
          q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
          double c23_0 = cos(q2[0]+q3[0]);
          double s23_0 = sin(q2[0]+q3[0]);
          double c23_1 = cos(q2[1]+q3[1]);
          double s23_1 = sin(q2[1]+q3[1]);
          q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
          ////////////////////////////////////////////////////////////////////////////////
          for(int k=0;k<2;k++) {
            if(fabs(q2[k]) < zero_thresh_)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < zero_thresh_)
              q4[k] = 0.0;
            else if(q4[k] < 0.0) q4[k] += 2.0*PI;

            qA->q(num_sols*6+0) = q1[i];    
            qA->q(num_sols*6+1) = q2[k]; 
            qA->q(num_sols*6+2) = q3[k];    
            qA->q(num_sols*6+3) = q4[k]; 
            qA->q(num_sols*6+4) = q5[i][j]; 
            qA->q(num_sols*6+5) = q6;
            num_sols++;
          }
        }
      }
    }
    return num_sols;
}

int Kinematic_func::Ycontact_InverseK_min(CArm *A)
{
  int ret;
  if(ret=Ycontact_InverseK(A)){
    double minerrsum=100000000;
    int idx = 0;
    for(int i=0;i<ret;i++){
      double errsum=0;
      for(int j=0;j<6;j++){
        double qother;
        if(A->q(i*6+j)>0)
            qother=A->q(i*6+j)-2*PI;
        else if(A->q(i*6+j)<0)
            qother=A->q(i*6+j)+2*PI;
        else
            qother=A->q(i*6+j);

        double err1=A->qc(j)-qother;
        double err=A->qc(j)-A->q(i*6+j);
        if((err1*err1)<(err*err)){
            A->q(i*6+j)=qother;
            err=err1;
        }
        errsum += (err*err);
      }
      if(minerrsum>errsum){
        minerrsum=errsum;
        idx=i;
      }
    }
    for(int i=0;i<6;i++)
      A->qd(i)=A->q(idx*6+i); // output of inverse kinematics
  }
  return ret;
}

void Kinematic_func::Jacobian(CArm *A)
{    
    s1 = sin(A->qc(0));
    c1 = cos(A->qc(0));
    s2 = sin(A->qc(1));
    c2 = cos(A->qc(1));
    s3 = sin(A->qc(2));
    c3 = cos(A->qc(2));
    s4 = sin(A->qc(3));
    c4 = cos(A->qc(3));
    s5 = sin(A->qc(4));
    c5 = cos(A->qc(4));
    s6 = sin(A->qc(5));
    c6 = cos(A->qc(5));
    
    s234 = sin(A->qc(1) + A->qc(2) + A->qc(3));
    c234 = cos(A->qc(1) + A->qc(2) + A->qc(3));

    s23 = sin(A->qc(1) + A->qc(2));
    c23 = cos(A->qc(1) + A->qc(2));
    
    A->Jp(0,0) = c1*(d6*c5 + d4) + s1*(d6*s5*c234 - d5*s234 - a3*c23 - a2*c2);
    A->Jp(0,1) = c1*(d6*s234*s5 + d5*c234 - a3*s23 - a2*s2);
    A->Jp(0,2) = c1*(d6*s234*s5 + d5*c234 - a3*s23);
    A->Jp(0,3) = c1*(d6*s234*s5 + d5*c234);
    A->Jp(0,4) = -c1*d6*c234*c5 - s1*d6*s5;
    A->Jp(0,5) = 0;

    A->Jp(1,0) = s1*(d6*c5 + d4) + c1*(-d6*s5*c234 + d5*s234 + a3*c23 + a2*c2);
    A->Jp(1,1) = s1*(d6*s234*s5 + d5*c234 - a3*s23 - a2*s2);
    A->Jp(1,2) = s1*(d6*s234*s5 + d5*c234 - a3*s23);
    A->Jp(1,3) = s1*(d6*s234*s5 + d5*c234);
    A->Jp(1,4) = -s1*d6*c234*c5 + c1*d6*s5;
    A->Jp(1,5) = 0;

    A->Jp(2,0) = 0;
    A->Jp(2,1) = a3*c23 + a2*c2 + d5*s234 - d6*c234*s5;
    A->Jp(2,2) = a3*c23 + d5*s234 - d6*c234*s5;
    A->Jp(2,3) = d5*s234 - d6*c234*s5;
    A->Jp(2,4) = -d6*s234*c5;
    A->Jp(2,5) = 0;

    
    A->Jw(0,0) = 0;
    A->Jw(0,1) = s1;
    A->Jw(0,2) = s1;
    A->Jw(0,3) = s1;
    A->Jw(0,4) = c1*s234;
    A->Jw(0,5) = c5*s1 - c1*c234*s5;

    A->Jw(1,0) = 0;
    A->Jw(1,1) = -c1;
    A->Jw(1,2) = -c1;
    A->Jw(1,3) = -c1;
    A->Jw(1,4) = s1*s234;
    A->Jw(1,5) = -s1*c234*s5 - c1*c5;

    A->Jw(2,0) = 1;
    A->Jw(2,1) = 0;
    A->Jw(2,2) = 0;
    A->Jw(2,3) = 0;
    A->Jw(2,4) = -c234;
    A->Jw(2,5) = -s234*s5;

    for(int i=0;i<3;i++){
        for(int j=0;j<6;j++){
            A->J(i,j)=A->Jp(i,j);
            A->J(3+i,j)=A->Jw(i,j);
        }
    }
}

void Kinematic_func::Jacobian_p(CArm *A)
{
    // Position Jacobian only

    s1 = sin(A->qc(0));
    c1 = cos(A->qc(0));
    s2 = sin(A->qc(1));
    c2 = cos(A->qc(1));
    s3 = sin(A->qc(2));
    c3 = cos(A->qc(2));
    s4 = sin(A->qc(3));
    c4 = cos(A->qc(3));
    s5 = sin(A->qc(4));
    c5 = cos(A->qc(4));
    s6 = sin(A->qc(5));
    c6 = cos(A->qc(5));

    s234 = sin(A->qc(1) + A->qc(2) + A->qc(3));
    c234 = cos(A->qc(1) + A->qc(2) + A->qc(3));
    s23  = sin(A->qc(1) + A->qc(2));
    c23  = cos(A->qc(1) + A->qc(2));

    A->Jp(0,0) = c1*(d6*c5 + d4) + s1*(d6*s5*c234 - d5*s234 - a3*c23 - a2*c2);
    A->Jp(0,1) = c1*(d6*s234*s5 + d5*c234 - a3*s23 - a2*s2);
    A->Jp(0,2) = c1*(d6*s234*s5 + d5*c234 - a3*s23);
    A->Jp(0,3) = c1*(d6*s234*s5 + d5*c234);
    A->Jp(0,4) = -c1*d6*c234*c5 - s1*d6*s5;
    A->Jp(0,5) = 0.0;

    A->Jp(1,0) = s1*(d6*c5 + d4) + c1*(-d6*s5*c234 + d5*s234 + a3*c23 + a2*c2);
    A->Jp(1,1) = s1*(d6*s234*s5 + d5*c234 - a3*s23 - a2*s2);
    A->Jp(1,2) = s1*(d6*s234*s5 + d5*c234 - a3*s23);
    A->Jp(1,3) = s1*(d6*s234*s5 + d5*c234);
    A->Jp(1,4) = -s1*d6*c234*c5 + c1*d6*s5;
    A->Jp(1,5) = 0.0;

    A->Jp(2,0) = 0.0;
    A->Jp(2,1) = a3*c23 + a2*c2 + d5*s234 - d6*c234*s5;
    A->Jp(2,2) = a3*c23 + d5*s234 - d6*c234*s5;
    A->Jp(2,3) = d5*s234 - d6*c234*s5;
    A->Jp(2,4) = -d6*s234*c5;
    A->Jp(2,5) = 0.0;
}

void Kinematic_func::Jacobian_w(CArm *A)
{
    A->Jw(0,0) = 0;
    A->Jw(0,1) = s1;
    A->Jw(0,2) = s1;
    A->Jw(0,3) = s1;
    A->Jw(0,4) = c1*s234;
    A->Jw(0,5) = c5*s1 - c1*c234*s5;

    A->Jw(1,0) = 0;
    A->Jw(1,1) = -c1;
    A->Jw(1,2) = -c1;
    A->Jw(1,3) = -c1;
    A->Jw(1,4) = s1*s234;
    A->Jw(1,5) = -s1*c234*s5 - c1*c5;

    A->Jw(2,0) = 1;
    A->Jw(2,1) = 0;
    A->Jw(2,2) = 0;
    A->Jw(2,3) = 0;
    A->Jw(2,4) = -c234;
    A->Jw(2,5) = -s234*s5;
}
    
Matrix3d Kinematic_func::RotZ(double th) // input unit : rad
{
    Matrix3d RotX_cal;

    RotX_cal << cos(th), -sin(th), 0,
                sin(th),  cos(th), 0,
                0      ,        0, 1;

    return RotX_cal;
}

Matrix3d Kinematic_func::RotY(double th) // input unit : rad
{
    Matrix3d RotY_cal;

    RotY_cal <<  cos(th), 0, sin(th),
                       0, 1,       0,
                -sin(th), 0, cos(th);
    return RotY_cal;
}

Matrix3d Kinematic_func::RotX(double th) // input unit : rad
{
    Matrix3d RotZ_cal;

    RotZ_cal << 1,        0,        0,
                0,  cos(th), -sin(th),
                0,  sin(th),  cos(th);

    return RotZ_cal;
}

Matrix3d Kinematic_func::angle_axis_representation(Eigen::Vector3d rot_axis, double rot_angle)
{
    Eigen::Matrix3d pre_rot_mat = Eigen::Matrix3d::Identity(); 
    
    double c = cos(rot_angle);
    double s = sin(rot_angle);
    double t = 1 - c;
    
    double x = rot_axis(0);
    double y = rot_axis(1);
    double z = rot_axis(2);
    
    pre_rot_mat(0, 0) = t * x * x + c;
    pre_rot_mat(0, 1) = t * x * y - s * z;
    pre_rot_mat(0, 2) = t * x * z + s * y;
    
    pre_rot_mat(1, 0) = t * x * y + s * z;
    pre_rot_mat(1, 1) = t * y * y + c;
    pre_rot_mat(1, 2) = t * y * z - s * x;
    
    pre_rot_mat(2, 0) = t * x * z - s * y;
    pre_rot_mat(2, 1) = t * y * z + s * x;
    pre_rot_mat(2, 2) = t * z * z + c;
    
    return pre_rot_mat;
}

Matrix3d Kinematic_func::Qua2Rot(double w,double x, double y, double z)
{
    Matrix3d Rot_out;
    Rot_out(0,0) = (1 - 2*y*y - 2*z*z);
    Rot_out(0,1) = (2*x*y - 2*z*w);
    Rot_out(0,2) = (2*x*z + 2*y*w);

    Rot_out(1,0) = (2*x*y + 2*z*w);
    Rot_out(1,1) = (1 - 2*x*x - 2*z*z);
    Rot_out(1,2) = (2*y*z - 2*x*w);

    Rot_out(2,0) = (2*x*z - 2*y*w);
    Rot_out(2,1) = (2*y*z + 2*x*w);
    Rot_out(2,2) = (1 - 2*x*x - 2*y*y);

    return Rot_out;
}

Quaterniond Kinematic_func::Rot2Qua(const Matrix3d& rotationMatrix) {
    Quaterniond q(rotationMatrix);
    return q;
}
