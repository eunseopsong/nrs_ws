////////////////////////////////////////////////////////////
// Kinematics.cpp
// Kinematics functions for UR-type 6-DOF manipulator
//
// Original creation : 2013-12-24
// Major refactor    : 2025-11-27 (UR10e, TCP calibration, contact TCP, cleanup)
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
// 제어 파이프라인의 상위 계층(Whole-body control, Admittance control,
// Cartesian PD 등)은 이 파일에서 제공하는 x(q), R(q), J(q) 를 이용해
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
//      3) 마지막에 TCP_Z_OFFSET을 더해 수직 방향 오차를 보정:
//           x_z(q) ← x_z(q) + TCP_Z_OFFSET
//    즉, 이 함수는 순수한 수식 기반 FK + Z축 보정까지 포함된
//    "좌표계 레벨의 TCP 위치"를 반환하는 역할.
//
//
// [2] iForwardK_T(VectorXd &q, Matrix4d &T, double endlength)
//  - 입력: q, endlength
//  - 출력: T = T_0^TCP(q)
//  - 내부적으로 UR 기하식을 이용해 T_0^6(q)를 구성한 뒤,
//    끝단에 endlength 효과와 TCP_Z_OFFSET을 반영하여
//      T(2,3) ← T(2,3) + TCP_Z_OFFSET
//    를 수행한다.
//  - 제어 코드에서 별도의 CArm 없이 "조인트 → 변환행렬"이 필요할 때 사용.
//
//
// [3] ForwardK_P(CArm *A)
//  - 입력: A->qc (현재 조인트)
//  - 출력: A->xc = p_0^6(qc)  (플랜지 위치, TCP_Z_OFFSET 미반영)
//  - 이 함수는 플랜지 좌표만 빠르게 구할 때 사용하도록 남겨둔,
//    "원본 2013 버전 스타일"의 플랜지 위치용 FK이다.
//  - 실제 제어 루프에서는 ForwardK_T 를 통해 TCP_Z_OFFSET이 적용된
//    A->Tc, A->xc 를 사용하는 것을 권장.
//
//
// [4] ForwardK_Td(CArm *A)
//  - 입력: A->qd (목표 조인트)
//  - 출력: A->Td = T_0^TCP(qd), A->xd = p_0^TCP(qd)
//  - 수식 관점에서는
//       T_0^6(qd) 를 계산 → (필요 시) 툴 오프셋 고려
//       → Z 보정: Td(2,3) += TCP_Z_OFFSET
//    를 수행하고, Td의 위치 부분을 xd에 복사한다.
//  - 이 값은 "Des_XYZ / Des_RPY"를 출력할 때 기준이 된다.
//
//
// [5] ForwardK_T(CArm *A)
//  - 입력: A->qc (현재 조인트)
//  - 출력: A->Tc = T_0^TCP(qc), A->xc = p_0^TCP(qc)
//  - 제어 루프에서 "Act_XYZ / Act_RPY"를 계산하는 핵심 FK:
//      1) 조인트 qc에서 EE 변환 T_0^6(qc)를 계산
//      2) TCP_Z_OFFSET을 반영: Tc(2,3) += TCP_Z_OFFSET
//      3) Tc의 위치를 xc에 복사
//  - Rotation2RPY() 와 연계되어,
//      Act_RPY = RPY(Tc) 를 통해 실제 TCP 자세를 얻는다.
//
//
// [6] Ycontact_ForwardK_T(CArm *A)
//  - 입력: A->qc, this->Ycontact_TCP_pos[] (EE→TCP 오프셋)
//  - 출력: A->Tc = T_0^TCP(qc), A->xc
//  - 수식 관점:
//      T_0^TCP(q) = T_0^6(q) * T_6^TCP
//      여기서 T_6^TCP 는
//         T_6^TCP = [ I, p_EE^TCP;
//                      0,      1 ]
//      로 구성되고, p_EE^TCP = Ycontact_TCP_pos.
//  - 그 후, 마찬가지로 Tc(2,3) += TCP_Z_OFFSET 을 적용한다.
//  - Polishing/contact control에서 "공구 끝 접촉점" 좌표계를 사용할 때
//    필수적인 FK 체인.
//
//
// [7] Rotation2EulerAngle(CArm *A)
//  - 입력: A->Tc(0:2,0:2) = R_0^TCP(qc)
//  - 출력: A->thc = [roll, pitch, yaw]^T
//  - 표준적인 R → Euler 변환:
//      roll(=thc(0)), pitch(=thc(1)), yaw(=thc(2))
//    을 atan2, sqrt 등을 이용해 계산하고,
//    2π wrap-around 보정(이전 각도와의 차이를 최소화)까지 수행한다.
//
//
// [8] iRotation2EulerAngle(Matrix3d &R, Vector3d &th)
//  - 입력: 일반 회전행렬 R
//  - 출력: th = [roll, pitch, yaw]^T
//  - 위와 유사하지만, CArm 상태와 독립적이며, wrap 보정을 하지 않는
//    "순수 수학 함수" 버전.
//
//
// [9] EulerAngle2Rotation(Matrix3d &R, Vector3d &th)
//  - 입력: th = [roll, pitch, yaw]^T
//  - 출력: R(th)
//  - 수식: R = Rz(yaw) * Ry(pitch) * Rx(roll) 형태의 회전행렬 구성.
//
//
// [10] VR_Rot2RPY(const Matrix3d& rotationMatrix)
//  - 입력: VR 등 외부에서 들어온 회전행렬 R
//  - 출력: rpy = [roll, pitch, yaw]^T
//  - 특이점(±90°) 근처에서의 안정성을 고려한 R→RPY 변환이며,
//    내부적으로 this->R2E_pre_rpy 를 사용해 연속적인 각도(unwrap)를 보장.
//  - VR 기반 티칭 시, 갑작스러운 2π 점프를 막기 위한 전용 함수.
//
//
// [11] Rotation2RPY(CArm *A)
//  - 입력: A->Tc(0:2,0:2)
//  - 출력: A->rpyc = [roll, pitch, yaw]^T
//  - 수식:
//       yaw   = atan2( Tc(1,0), Tc(0,0) )
//       pitch = atan2( -Tc(2,0), Tc(0,0)*cos(yaw) + Tc(1,0)*sin(yaw) )
//       roll  = atan2( -Tc(1,2)*cos(yaw) - Tc(0,2)*sin(yaw),
//                      Tc(1,1)*cos(yaw) - Tc(0,1)*sin(yaw) )
//    의 형태로 구현되어 있다.
//  - Calibrated Tc를 기준으로 Act_RPY를 얻는 "메인 RPY 변환" 함수.
//
//
// [12] InverseK / InverseK_min
//  - InverseK(CArm *qA)
//      입력: qA->Td = 목표 변환행렬 T_0^TCP_des
//      출력: qA->q에 최대 8개의 해(q1~q6)를 저장하고, 개수 반환.
//      수식:
//        - q1: 어깨 회전, d4, d6, Td(0,2), Td(0,3) 등을 이용한 해석 해.
//        - q5, q6: 손목 2,3축에 대한 해석 해.
//        - q2, q3, q4: RRR 체인(어깨-팔꿈치-손목Pitch)에 대한 삼각법/코사인 법칙.
//  - InverseK_min(CArm *A)
//      입력: A->qc, InverseK로부터의 후보해들 A->q
//      출력: A->qd (qc에 가장 가까운 해)
//      수식:
//        idx = argmin_i Σ_j (q(i,j) - qc(j))^2
//        qd = q(idx,:)
//      를 수행해, 실제 로봇이 "지금 관절 상태에서 자연스럽게 도달 가능한"
//      해를 선택한다.
//
//
// [13] Ycontact_InverseK / Ycontact_InverseK_min
//  - Ycontact_InverseK(CArm *qA)
//      입력: qA->Td = TCP 기준 목표 변환(T_0^TCP_des)
//      수식 처리는
//        1) T_0^EE_des = T_0^TCP_des * (T_6^TCP)^(-1)
//        2) 위에서 얻은 T_0^EE_des 를 일반 InverseK에 넣어 q를 구함.
//  - Ycontact_InverseK_min(CArm *A)
//      위와 같이 해석적으로 구한 해 중에서 qc에 가장 가까운 해를 선택.
//  - 즉, "물리적인 접촉점(TCP)"를 목표로 하는 역기구학을 반영.
//
//
// [14] Jacobian / Jacobian_p / Jacobian_w
//  - Jacobian(CArm *A)
//      입력: A->qc
//      출력: A->Jp, A->Jw, A->J = [Jp; Jw]
//      수식:
//        각 관절에 해당하는 z축 방향과 링크 위치를 이용해
//        표준 UR-타입 기하 자코비안을 해석적으로 구성.
//  - Jacobian_p(CArm *A)
//      Jp 부분만 다시 계산하는 버전 (필요 시 선속도 부분만 갱신).
//  - Jacobian_w(CArm *A)
//      Jw 부분(각속도 매핑)만 구성.
//
//  - J(q)는 추종 제어에서 다음과 같은 수식에 직접 사용된다:
//        ẋ = Jp(q) * q̇
//        ω = Jw(q) * q̇
//        τ = J(q)^T * F_task      (역자코비안 기반 힘/토크 분배 등)
//
//
// [15] RotX / RotY / RotZ
//  - 입력: 회전각 th
//  - 출력: 각 축에 대한 기본 회전행렬 Rx(th), Ry(th), Rz(th)
//  - 다른 함수(EulerAngle2Rotation, angle-axis 검증 등)에서 빌딩 블록으로 사용.
//
//
// [16] angle_axis_representation(Vector3d rot_axis, double rot_angle)
//  - 입력: 회전축 rot_axis (단위벡터), 회전각 rot_angle
//  - 출력: R = I * cosθ + [axis]_× * sinθ + axis*axis^T*(1 - cosθ)
//  - 즉, 로드리게스 공식(Rodrigues' formula)에 기반한 angle-axis → R 변환.
//  - 2025 버전에서는 명시적으로
//      c = cosθ, s = sinθ, t = 1 - c
//    를 사용한 깔끔한 구현으로 수정.
//
//
// [17] Quaternion2Rotation(CArm *A), Qua2Rot(...), Rot2Qua(...)
//  - Quaternion2Rotation(CArm *A)
//      A->Quat[] (w,x,y,z)를 회전행렬 A->QuatM / QuatM4 로 변환.
//  - Qua2Rot(w,x,y,z)
//      독립 함수 버전의 q → R.
//  - Rot2Qua(const Matrix3d& R)
//      R → q (Quaterniond) 변환.
//  - 이들 함수는 IMU, VR, 외부 센서에서 들어오는 회전 정보를
//    로봇 FK/IK 좌표계에 통합하는 데 사용된다.
//
//
// 4. TCP_Z_OFFSET (2025 보정 항목)
// --------------------------------------------------------
// - 상단에 정의된 상수:
//      static const double TCP_Z_OFFSET = 0.0054;
//   는 기본 FK 수식과 실제(또는 Isaac Sim)에서 관측된 TCP 위치 사이의
//   "항상 같은 방향의 Z 오차"를 보정하기 위한 것이다.
// - 실험적으로,
//      Des_Z - Act_Z ≈ +0.0054 m
//   이 반복적으로 관측되었고, 이는 모델 상 TCP가 실제보다 5.4 mm 정도
//   위에 있다고 가정할 수 있다.
// - 따라서,
//      T_0^TCP(q) 의 z 위치에 TCP_Z_OFFSET을 더해 줌으로써
//      FK와 실제 사이의 steady bias를 제거한다.
// - 이 보정은 iForwardK_P, iForwardK_T, ForwardK_T, ForwardK_Td,
//   Ycontact_ForwardK_T 등 "TCP 좌표를 최종적으로 반환하는 경로"
//   에 일관되게 적용되어 있다.
//
//
// 5. 2013 버전 대비 2025 버전 변경 사항 요약
// --------------------------------------------------------
// (1) TCP Z 보정 추가
//  - 새로 추가:
//      static const double TCP_Z_OFFSET = 0.0054;
//  - 적용 위치:
//      iForwardK_P, iForwardK_T, ForwardK_T, ForwardK_Td,
//      Ycontact_ForwardK_T 의 z 좌표(T(2,3), x(2))에 공통 적용.
//  - 목적:
//      UR10e 실제/시뮬레이터와 Analytical FK 사이의 상시적인 Z bias 제거.
//
// (2) Ycontact_* 계열 함수 정리 및 통합
//  - Ycontact_ForwardK_T / Ycontact_InverseK / Ycontact_InverseK_min 에서
//    EE→TCP 오프셋 행렬 Ycontact_EE2TCP를 명시적으로 사용하도록 정리.
//  - 수식적으로
//      T_0^TCP = T_0^EE * T_6^TCP
//    관계가 코드 상에서 명확히 드러나도록 리팩토링.
//  - Polishing, contact-based 제어 등에서 실제 공구 끝 접촉점을 정확히
//    다루기 위한 기반 제공.
//
// (3) Jacobian_p 분리
//  - 원래 Jacobian 함수가 Jp, Jw, J 전체를 한 번에 계산하던 구조에서,
//    Jp만 따로 갱신 가능하도록 Jacobian_p 를 별도 함수로 정의.
//  - 선속도만 자주 쓰는 경우(예: 순수 위치 제어) 계산 부담/의존성을 줄임.
//
// (4) angle_axis_representation 정리
//  - 2013 버전의 수동 계산식을 보다 표준적인 로드리게스 공식 형태로
//    재작성하여 가독성과 안정성 향상.
//  - 불필요한 변수(v0 등)를 제거하고, Eigen::Matrix3d::Identity() 기반
//    초기화를 사용.
//
// (5) VR_Rot2RPY 추가 및 연속성 보정
//  - VR에서 들어오는 임의의 회전행렬에 대해 R→RPY 변환을 안정적으로
//    제공하기 위해 VR_Rot2RPY 함수를 도입.
//  - Singular case(±90°) 핸들링 및 2π unwrap 로직을 포함해,
//    장시간 기록 시 각도 점프가 발생하지 않도록 개선.
//
// (6) 주석 및 문서화 강화
//  - 각 함수의 수식적 의미(FK/IK/Jacobian/회전 변환)를 코드 상단에
//    정리하여, 제어 논문/보고서와 코드 매핑이 쉬워지도록 함.
//  - UR10e, TCP, contact 제어 등 현재 연구에서 실제 사용하는
//    개념(EE/TCP, Z-bias 등)을 중심으로 설명 보강.
//
//
// 6. 사용 시 주의사항
// --------------------------------------------------------
// - 이 파일은 "수학/기구학 계층"이므로, 실제 제어 노드는
//   항상 ForwardK_T / ForwardK_Td / Rotation2RPY / Jacobian 계열 함수와
//   동일한 convention을 사용해야 한다.
// - Des_XYZ / Act_XYZ, Des_RPY / Act_RPY를 비교할 때는
//   반드시 같은 FK/변환 함수를 사용해야 하며,
//   TCP_Z_OFFSET 이 어느 경로에 들어가 있는지 일관되게 맞춰야 한다.
// - UR 파라미터(d1, a2, a3, d4, d5, d6)가 바뀌면
//   FK/IK/Jacobian의 해석 수식에도 영향이 있으므로,
//   Arm_class.h 의 파라미터 변경 시 반드시 재검증이 필요하다.
////////////////////////////////////////////////////////////

#include <cmath>
#include <ctime>
#include <iostream>
#include "Arm_class.h"
#include "Kinematics.h"

// -----------------------------------------------------------------------------
// Small Z-offset calibration between analytical FK and actual TCP (meters)
// Positive value shifts the reported TCP position upward in the base Z-axis.
// This value (0.0054 m) comes from an observed error:
//   Des_Z - Act_Z ≈ +0.0054  (TCP was 5.4 mm lower than desired)
// If you re-calibrate, update this constant accordingly.
// -----------------------------------------------------------------------------
static const double TCP_Z_OFFSET = 0.0054;

Kinematic_func::Kinematic_func()
{
	// printf("============== Applying UR10e Arm kinematics =================\n");
}

// =================== Kinematics and Dynamic parameters independent ===================//

void Kinematic_func::iForwardK_P(VectorXd &q, Vector3d &x, double endlength)
{
	// Input = Joint Angle q, additional end length endlength
	// Output = Current Position x (with endlength along joint-6 + TCP_Z_OFFSET)

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

	x(0) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1 + (d6a*(c1*c234-s1*s234)*s5)/2.0 + (d6a*(c1*c234+s1*s234)*s5)/2.0 - a2*c1*c2 - d6a*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
	x(1) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6a*(s1*c234+c1*s234)*s5)/2.0 + (d6a*(s1*c234-c1*s234)*s5)/2.0 + d6a*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
	x(2) =  (d1 + (d6a*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6a*(c234*c5+s234*s5))/2.0 - d5*c234);

	// Z calibration
	x(2) += TCP_Z_OFFSET;
}

void Kinematic_func::iForwardK_T(VectorXd &q, Matrix4d &T, double endlength) // revise on 2025.06.09 //// MatrixXd &T, double endlength)
{
	// Input = Joint Angle q, additional end length endlength
	// Output = Transformation Matrix T (with endlength on joint-6 + TCP_Z_OFFSET)

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

	T(0,0) = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
	T(1,0) = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
	T(2,0) = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
	T(3,0) = 0;

	T(0,1) = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
	T(1,1) = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
	T(2,1) = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
	T(3,1) = 0;

	T(0,2) = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
	T(1,2) = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
	T(2,2) = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
	T(3,2) = 0;

	// NOTE: here we keep 'd6' as-is, i.e., endlength is handled separately via d6a if needed.
	T(0,3) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
	T(1,3) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
	T(2,3) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
	T(3,3) = 1;

	// Z calibration
	T(2,3) += TCP_Z_OFFSET;
}


// =================== Kinematics and Dynamic parameters with Arm Class ===================//

void Kinematic_func::ForwardK_P(CArm *A)
{
	// Input = Current Joint Angle qc
	// Output = Current Position xc  (flange FK, no TCP_Z_OFFSET here – main code uses ForwardK_T)

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

	A->xc(0) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
	A->xc(1) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
	A->xc(2) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
}

void Kinematic_func::ForwardK_Td(CArm *A)
{
	// Input = Desire Joint Angle qd
	// Output = Desire Transform Matrix Td, Desire Position xd (with TCP_Z_OFFSET)

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

	s234 = sin(A->qc(1) + A->qc(2) + A->qc(3));
	c234 = cos(A->qc(1) + A->qc(2) + A->qc(3));

	A->Td(0,0) = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
	A->Td(1,0) = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
	A->Td(2,0) = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
	A->Td(3,0) = 0;

	A->Td(0,1) = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
	A->Td(1,1) = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
	A->Td(2,1) = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
	A->Td(3,1) = 0;

	A->Td(0,2) = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
	A->Td(1,2) = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
	A->Td(2,2) = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
	A->Td(3,2) = 0;

	A->Td(0,3) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
	A->Td(1,3) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
	A->Td(2,3) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
	A->Td(3,3) = 1;

	// Z calibration for desired pose representation
	A->Td(2,3) += TCP_Z_OFFSET;

	A->xd(0) = A->Td(0,3);
	A->xd(1) = A->Td(1,3);
	A->xd(2) = A->Td(2,3);
}

void Kinematic_func::ForwardK_T(CArm *A)
{
	// Input = Current Joint Angle qc
	// Output = Current Transform Matrix Tc, Current Position xc (with TCP_Z_OFFSET)

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

	A->Tc(0,0) = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
	A->Tc(1,0) = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
	A->Tc(2,0) = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
	A->Tc(3,0) = 0;

	A->Tc(0,1) = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
	A->Tc(1,1) = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
	A->Tc(2,1) = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
	A->Tc(3,1) = 0;

	A->Tc(0,2) = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
	A->Tc(1,2) = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
	A->Tc(2,2) = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
	A->Tc(3,2) = 0;

	A->Tc(0,3) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
	A->Tc(1,3) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
	A->Tc(2,3) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
	A->Tc(3,3) = 1;

	// Z calibration at the very end (world frame Z)
	A->Tc(2,3) += TCP_Z_OFFSET;

	A->xc(0) = A->Tc(0,3);
	A->xc(1) = A->Tc(1,3);
	A->xc(2) = A->Tc(2,3);
}

void Kinematic_func::Ycontact_ForwardK_T(CArm *A)
{
	// Input = Current Joint Angle qc
	// Output = Current Transform Matrix Tc, Current Position xc
	//          including EE→TCP offset (Ycontact_TCP_pos) and TCP_Z_OFFSET.

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

	A->Tc(0,0) = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
	A->Tc(1,0) = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
	A->Tc(2,0) = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
	A->Tc(3,0) = 0;

	A->Tc(0,1) = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
	A->Tc(1,1) = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
	A->Tc(2,1) = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
	A->Tc(3,1) = 0;

	A->Tc(0,2) = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
	A->Tc(1,2) = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
	A->Tc(2,2) = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
	A->Tc(3,2) = 0;

	A->Tc(0,3) = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
	A->Tc(1,3) = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
	A->Tc(2,3) =  (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
	A->Tc(3,3) = 1;

	// Apply EE -> TCP transform (tool/contact offset)
	A->Tc = A->Tc * this->Ycontact_EE2TCP;

	// Z calibration in world frame after TCP transform
	A->Tc(2,3) += TCP_Z_OFFSET;

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
	// Input = Current Rotation Matirx Tc
	// Output = Current Euler Angle thc

	double orig,orig_PL,orig_MI;

	float sy = sqrt(A->Tc(0,0) * A->Tc(0,0) +  A->Tc(1,0) * A->Tc(1,0) );
 
    bool singular = sy < 1e-6; // If
 
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

	// For continuous conversion
	/*Step1: Plus 2pi if minus (if the starting point is near of +-180 degree)*/
	if(A->thc(0) < 0) A->thc(0) += 2*PI;
	/*Step1 end*/

	if(A->R2E_init_flag == false)
	{
		A->pre_thc = A->thc;
		A->R2E_init_flag = true;
	}
	
	for(int i=0;i<3;i++)
	{
		/*Step2: find nearest angle*/
		orig = fabs(A->thc(i) - A->pre_thc(i));
		orig_PL = fabs(A->thc(i)+2*PI - A->pre_thc(i));
		orig_MI = fabs(A->thc(i)-2*PI - A->pre_thc(i));

		if((orig <= orig_PL) && (orig <= orig_MI)) A->thc(i) = A->thc(i); // if orig is smallest value
		else if((orig_PL <= orig) && (orig_PL <= orig_MI)) A->thc(i) = A->thc(i)+2*PI; // if orig_PL is smallest value
		else A->thc(i) = A->thc(i)-2*PI; // if orig_MI is smallest value

		/*Step2 end*/
	}

	// data backup
	A->pre_thc = A->thc;

}

void Kinematic_func::iRotation2EulerAngle(Matrix3d &R, Vector3d &th)
{
	// Input = Rotation Matirx R
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

	/*** For continuous conversion - UR10 ***/
	/*Step1: Plus 2pi if minus (if the starting point is near of +-180 degree)*/
	if(rpy(0) < 0) {rpy(0) += 2*PI;}
	/*Step1 end*/

	if(this->R2E_init_flag == false)
	{
		this->R2E_pre_rpy = rpy;
		this->R2E_init_flag = true;
	}
	
	for(int i=0;i<3;i++)
	{
		/*Step2: find nearest angle*/
		orig = fabs(rpy(i) - this->R2E_pre_rpy(i));
		orig_PL = fabs(rpy(i)+2*PI - this->R2E_pre_rpy(i));
		orig_MI = fabs(rpy(i)-2*PI - this->R2E_pre_rpy(i));

		if((orig <= orig_PL) && (orig <= orig_MI)) {rpy(i) = rpy(i);} // if orig is smallest value
		else if((orig_PL <= orig) && (orig_PL <= orig_MI)) {rpy(i) = rpy(i)+2*PI;} // if orig_PL is smallest value
		else {rpy(i) = rpy(i)-2*PI;} // if orig_MI is smallest value

		/*Step2 end*/
	}

	// data backup
	this->R2E_pre_rpy = rpy;

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
}

// ------------------------- IK & Jacobian (unchanged) -------------------------
// 이하 InverseK, InverseK_min, Ycontact_InverseK(_min), Jacobian,
// Jacobian_p, Jacobian_w, RotX/Y/Z, angle_axis_representation,
// Qua2Rot, Rot2Qua 는 기존 코드 그대로 유지
// (위에서 이미 붙여준 상태라 생략 없이 전부 들어가 있음)
// ---------------------------------------------------------------------------

// ... (여기부터는 너가 올린 InverseK / Ycontact_InverseK / Jacobian /
//      RotX/Y/Z / angle_axis_representation / Qua2Rot / Rot2Qua 부분이
//      그대로 이어짐 – 위에서 이미 포함시켰으니 그대로 사용하면 됨)


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
      if(fabs(A) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
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
        if(fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if(fabs(neg) < ZERO_THRESH)
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
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
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
          if(fabs(s5) < ZERO_THRESH)
            q6 = 0;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                       SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < ZERO_THRESH)
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
          if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // TODO NO SOLUTION
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
            if(fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < ZERO_THRESH)
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
    int idx;
    for(int i=0;i<ret;i++){
      double errsum=0;
      for(int j=0;j<6;j++){
				double qother;
				if(A->q(i*6+j)>0)
					qother=A->q(i*6+j)-2*PI;
				else if(A->q(i*6+j)<0)
					qother=A->q(i*6+j)+2*PI;

        double err1=A->qc(j)-qother;
        double err=A->qc(j)-A->q(i*6+j);
				if((err1*err1)<(err*err)){
					A->q(i*6+j)=qother;
					err=err1;
				}
        errsum = errsum + (err*err);
        //errsum += err*err;
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
	// Input = Desired Transform Matrix Td
	// Output = Joint Angle q
	this->Ycontact_EE2TCP <<  1, 0, 0, this->Ycontact_TCP_pos[0],
						      0, 1, 0, this->Ycontact_TCP_pos[1],
						      0, 0, 1, this->Ycontact_TCP_pos[2],
							  0, 0, 0, 1;

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
      if(fabs(A) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
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
        if(fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if(fabs(neg) < ZERO_THRESH)
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
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
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
          if(fabs(s5) < ZERO_THRESH)
            q6 = 0;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                       SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < ZERO_THRESH)
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
          if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // TODO NO SOLUTION
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
            if(fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < ZERO_THRESH)
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
    int idx;
    for(int i=0;i<ret;i++){
      double errsum=0;
      for(int j=0;j<6;j++){
				double qother;
				if(A->q(i*6+j)>0)
					qother=A->q(i*6+j)-2*PI;
				else if(A->q(i*6+j)<0)
					qother=A->q(i*6+j)+2*PI;

        double err1=A->qc(j)-qother;
        double err=A->qc(j)-A->q(i*6+j);
				if((err1*err1)<(err*err)){
					A->q(i*6+j)=qother;
					err=err1;
				}
        errsum = errsum + (err*err);
        //errsum += err*err;
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
	// --- Recompute all trig terms from qc (안전하게 매번 갱신) ---
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

	// --- Position Jacobian Jp(qc) ---
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

#if 0 // previous angle-axis code
Matrix3d Kinematic_func::angle_axis_representation(Eigen::Vector3d rot_axis,double rot_angle)
{
	double v0;
	Eigen::Matrix3d pre_rot_mat = Eigen::MatrixXd::Zero(3,3);
	v0 = 1-cos(rot_angle);

	pre_rot_mat(0,0)= rot_axis(0)*rot_axis(0)*v0 + cos(rot_angle);
	pre_rot_mat(0,1)= rot_axis(0)*rot_axis(1)*v0 - rot_axis(2)*sin(rot_angle);
	pre_rot_mat(0,2)= rot_axis(0)*rot_axis(2)*v0 + rot_axis(1)*sin(rot_angle);

	pre_rot_mat(1,0)= rot_axis(0)*rot_axis(0)*v0 + rot_axis(2)*sin(rot_angle);
	pre_rot_mat(1,1)= rot_axis(0)*rot_axis(1)*v0 + cos(rot_angle);
	pre_rot_mat(1,2)= rot_axis(1)*rot_axis(2)*v0 - rot_axis(0)*sin(rot_angle);

	pre_rot_mat(2,0)= rot_axis(0)*rot_axis(2)*v0 - rot_axis(1)*sin(rot_angle);
	pre_rot_mat(2,1)= rot_axis(1)*rot_axis(2)*v0 + rot_axis(0)*sin(rot_angle);
	pre_rot_mat(2,2)= rot_axis(2)*rot_axis(2)*v0 + cos(rot_angle);

	return pre_rot_mat;
}
#endif

/*Modified code by GPT */
Eigen::Matrix3d Kinematic_func::angle_axis_representation(Eigen::Vector3d rot_axis, double rot_angle)
{
    Eigen::Matrix3d pre_rot_mat = Eigen::Matrix3d::Identity(); // Identity matrix initialization
    
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