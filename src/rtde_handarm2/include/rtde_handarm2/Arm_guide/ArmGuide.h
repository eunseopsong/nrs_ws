/****************************************************
  *
  * ArmGuide header file (ArmTrajectory.h) for 6-DOF ARM
  *
  * Created on 2016. 9. 7.
  * Created by Gitae
  *
  * Revised on 2025. 6. 9.
  * Revised by Eunseop
****************************************************/

#ifndef RTDE_HANDARM2_ARMGUIDE_H_
#define RTDE_HANDARM2_ARMGUIDE_H_

#include <iostream>
#include <fcntl.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <sys/ioctl.h>
#include <cstring>
#include <Eigen/Dense>

// 이전: "../UR10/Arm_class.h"
// 변경: include 디렉토리에서 바로 찾도록 절대 경로 사용
#include "rtde_handarm2/UR10/Arm_class.h"
#include "rtde_handarm2/UR10/Kinematics.h"

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

#endif  // RTDE_HANDARM2_ARMGUIDE_H_
