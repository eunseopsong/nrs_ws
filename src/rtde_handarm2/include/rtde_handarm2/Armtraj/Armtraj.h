#ifndef __ARMTRAJ__
#define __ARMTRAJ__

#include	<iostream>
#include 	<fcntl.h>
#include 	<math.h>
#include 	<stdio.h>
#include 	<stdlib.h>
#include 	<unistd.h>
#include 	<termios.h>
#include 	<errno.h>
#include 	<sys/ioctl.h>
#include	<string.h>
#include	<Eigen/Dense>
#include	"../UR10/Arm_class.h"

using namespace Eigen;

#define		AHz				      (125)
#define PI 3.14159265359

typedef class Armtraj
{
private:
	double a[6],Dir[6];
	double MaxFinalTime;
	double s,sdt,sdtdt;
	int num = 0;
	int SEQ = 0;
	double JointMaxVel = 0.05;
	double JointMaxAcc = 0.05;
	double TaskMaxVel = 0.1;
	double TaskMaxAcc = 0.1;
	double vel[6],acc[6];
	double beta = 1;
	unsigned int	TaskIteration;
	double		TaskTime;
	VectorXd	Distance;
	double MaxDistance = 0;
	VectorXd	JointPosStart;
	VectorXd	JointPosStop;
	VectorXd	TaskPosStart;
	VectorXd	TaskPosStop;
	VectorXd	JointVelStart;
	VectorXd	TaskVelStart;
	double t1[6],t2[6],tj[6],ta[6],tup,tm,tv,tj1,ta1,ti[6];
	double jerk;
	unsigned int j = 1;
	Vector3d Dist, tasktime, v0;
	VectorXd InitDist;
	VectorXd Dist_cal;
	double vtop;
	float aMax;
	float vMax;
	float tMax;

	double r_link[6];
	double dist;
	double dist_h;
	double Qmax;
	double Q_num;
	Vector3d xt_ee;
	Vector3d xc_ee;
	Vector3d vc_ee;
	Vector3d init_xt_ee;
	Vector3d vd_ee;
	Vector3d dxd_ee;
	Vector3d xd_ee;
	double d_target;
	double d_z;
	Vector3d x_dist;
	double x_angle;
	double dist_num;
	Vector4d Q_dist;
	Vector4d Q_theta;
	Vector4d Q_target;
	Vector4d Q_vel;
	Vector4d Q_det;
	Vector4d Q;
	Vector4d ch;
	double theta;
	Vector4d theta_s;
	double angle;
	double z_ch;
	double z_change;

	//Fogale
	MatrixXd n_origin2;
	Vector4d a_f;
	MatrixXd v;
	Vector3d s_f;
	Vector3d u;
	double M;
	double d_low = 0.08;
	double d_high = 0.2;
	Vector3d v_res;
	double w_k;
	double w_t = 1;

public:
	Armtraj();

	int TaskTraj_lspb_realtime(CArm *A, float CustomTaskMaxVel, float CustomTaskMaxAcc);
	void PD(CArm *A);
	int Traj3(CArm *A);
	int Traj5(CArm *A);
	int Traj_rspb(CArm *A);
	int TaskTraj_rspb(CArm *A);
	int Traj_lspb(CArm *A);
	int Traj_scurve(CArm *A);
	int Traj_rspb_c(CArm *A, float CustomJointMaxVel, float CustomJointMaxAcc);
	int Traj_scurve_c(CArm *A, float CustomJointMaxVel, float CustomJointMaxAcc, float Custombeta);
	int Traj_rspb_velocity(CArm *A);



} Ctraj;


#endif
