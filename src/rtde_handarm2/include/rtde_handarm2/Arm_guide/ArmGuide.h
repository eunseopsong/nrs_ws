/****************************************************
  * 
  * ArmGide header file (ArmTrajectory.h) for 6-DOF ARM 
  *
  * Created on 2016. 9. 7.
  * Created by Gitae
  *
****************************************************/

#ifndef ArmGuide_H_
#define ArmGuide_H_

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
#include	"../UR10/Kinematics.h"

using namespace Eigen;


class ArmGuide
{
private:


public:

  ArmGuide();
  ~ArmGuide();
	int ArmForceGuide4(AKfun *AKin, CArm *RArm);
  int ArmForceGuide_position(AKfun *AKin, CArm *RArm);
	int MovingAverageFilter(double *x, int length, int filternum);
	int LowpassFilter(double *x, int length, double alpha);


};


#endif

