#pragma once

#include "Y2Kinematics/Kinematics.hpp"

#define M_PI 3.141592
#define DegreeToRadian(degree) ((degree) * M_PI / 180.0)
#define RadianToDegree(radian) ((radian) * 180.0 / M_PI)

class KinematicsUR10 : public Kinematics {
private:
    /* UR10 Kinematics parameters (mm) */
    // Link lengths
    static constexpr double a2 = -612.0;    // Upper arm length
    static constexpr double a3 = -572.3;    // Forearm length
    
    // Link offsets
    static constexpr double d1 = 128.0;    // Base height
    static constexpr double d4 = 163.9;    // Wrist 1 offset
    static constexpr double d5 = 115.7;    // Wrist 2 offset  
    static constexpr double d6 = 92.2;     // Wrist 3 offset

    /* Parameters for joint limit */
    std::vector<double> q_min = {-2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI};
    std::vector<double> q_max = { 2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI};

    std::vector<double> qd_min = 
    {-DegreeToRadian(60*1.5),-DegreeToRadian(60*1.5), -DegreeToRadian(90*1.5),
        -DegreeToRadian(100*1.5),-DegreeToRadian(120*1.5),-DegreeToRadian(120*1.5)};
    std::vector<double> qd_max =
    {DegreeToRadian(60*1.5), DegreeToRadian(60*1.5), DegreeToRadian(90*1.5), 
        DegreeToRadian(100*1.5), DegreeToRadian(120*1.5), DegreeToRadian(120*1.5)};

    std::vector<double> qdd_min =
    {-DegreeToRadian(200*1.5),-DegreeToRadian(200*1.5),-DegreeToRadian(250*1.5),
    -DegreeToRadian(700*1.5),-DegreeToRadian(700*1.5),-DegreeToRadian(700*1.5)};
    std::vector<double> qdd_max=
    {DegreeToRadian(200*1.5), DegreeToRadian(200*1.5), DegreeToRadian(250*1.5),
     DegreeToRadian(700*1.5), DegreeToRadian(700*1.5), DegreeToRadian(700*1.5)};

    
public:
    KinematicsUR10(double SamplingTime_ = 0.01, size_t numOfAxis_ = 6, const YMatrix& EE2TCP_ = YMatrix::identity(4));

    // Forward Kinematics - T06 transformation matrix
    YMatrix forwardKinematics(const std::vector<double>& q) override;
    
    // Jacobian calculation 
    YMatrix calculateJacobian(const std::vector<double>& q) override;
};