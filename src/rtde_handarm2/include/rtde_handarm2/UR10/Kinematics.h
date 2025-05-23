////////////////////////////////////////////////////////////
// Kinematics.h
// Kinematics functions
////////////////////////////////////////////////////////////
////////////////// Ver 2.00 ////////////////////////////////
////////////////////////////////////////////////////////////

#ifndef Kinematic_func_H_
#define Kinematic_func_H_

#include "Arm_class.h"
#include <Eigen/Dense>

/* Yaml file headers */
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "../../../NRS_yaml/NRS_yaml_location.h"

using namespace Eigen;

#define ZERO_THRESH 0.00000001
#define SIGN(x) ( ( (x) > 0 ) - ( (x) < 0 ) )

struct Quaternion{double w,x,y,z;};

/* Yaml file load */
std::ifstream Kfin1(NRS_Fcon_setting_loc);
YAML::Node KNRS_Fcon_setting = YAML::Load(Kfin1);

typedef class Kinematic_func
{
private:
	double s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s6, c6, s23, c23, s34, c34, s234, c234;
	Matrix4d Ycontact_EE2TCP = Matrix4d::Zero(4,4);
	// double Ycontact_TCP_pos[3] = {0, 0.114, 0.221}; // From E.E to TCP at E.E. frame

	double Ycontact_TCP_pos[3] = 
	{KNRS_Fcon_setting["TCP_center"]["TCP_x"].as<double>(), KNRS_Fcon_setting["TCP_center"]["TCP_y"].as<double>(), KNRS_Fcon_setting["TCP_center"]["TCP_z"].as<double>()};

	bool R2E_init_flag = false;
	Vector3d R2E_pre_rpy = Vector3d::Zero(3);

public:
	Kinematic_func();

	void iForwardK_P(VectorXd &q, Vector3d &x, double endlength = 0);
	void iForwardK_T(VectorXd &q, MatrixXd &T, double endlength = 0);
	
	void ForwardK_P(CArm *A);
	void ForwardK_T(CArm *A);
	void ForwardK_Td(CArm *A);
	void Ycontact_ForwardK_T(CArm *A);
	void Rotation2EulerAngle(CArm *A);
	void Quaternion2Rotation(CArm *A);
	void iRotation2EulerAngle(Matrix3d &R, Vector3d &th);
	void Rotation2RPY(CArm *A);
	void EulerAngle2Rotation(Matrix3d &R, Vector3d &th);
	Vector3d VR_Rot2RPY(const Matrix3d& rotationMatrix); // Singular compensation

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

	Matrix3d angle_axis_representation(Eigen::Vector3d rot_axis,double rot_angle);
	Matrix3d Qua2Rot(double w,double x, double y, double z);
	Quaterniond Rot2Qua(const Matrix3d& rotationMatrix);

} AKfun;

#endif

