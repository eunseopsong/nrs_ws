#pragma once

#include "Y2Kinematics/Kinematics.hpp"

#define M_PI 3.141592
#define DegreeToRadian(degree) ((degree) * M_PI / 180.0)
#define RadianToDegree(radian) ((radian) * 180.0 / M_PI)

class KinematicsKUKAiiwa : public Kinematics {
private:
    // Kinematics parameters (mm)
    static constexpr double d1 = 340.0;
    static constexpr double d3 = 400.0;
    static constexpr double d5 = 400.0;
    static constexpr double d7 = 126.0;

    // Parameters for joint limit
    std::vector<double> q_min =
    {-DegreeToRadian(170),-DegreeToRadian(120),-DegreeToRadian(170),
    -DegreeToRadian(120),-DegreeToRadian(170),-DegreeToRadian(120),-DegreeToRadian(175)};
    std::vector<double> q_max =
    {DegreeToRadian(170), DegreeToRadian(120), DegreeToRadian(170),
     DegreeToRadian(120), DegreeToRadian(170), DegreeToRadian(120), DegreeToRadian(175)};
    
    std::vector<double> qd_min = 
    {-DegreeToRadian(60),-DegreeToRadian(60), -DegreeToRadian(0), // 70
     -DegreeToRadian(90),-DegreeToRadian(100),-DegreeToRadian(120),-DegreeToRadian(120)};
    std::vector<double> qd_max =
    {DegreeToRadian(60), DegreeToRadian(60), DegreeToRadian(0), //70
     DegreeToRadian(90), DegreeToRadian(100), DegreeToRadian(120), DegreeToRadian(120)};

     std::vector<double> qdd_min = 
    {-DegreeToRadian(200),-DegreeToRadian(200),-DegreeToRadian(250),
    -DegreeToRadian(400),-DegreeToRadian(400),-DegreeToRadian(700),-DegreeToRadian(700)};
    std::vector<double> qdd_max=
    {DegreeToRadian(200), DegreeToRadian(200), DegreeToRadian(250),
     DegreeToRadian(400), DegreeToRadian(400), DegreeToRadian(700), DegreeToRadian(700)};
    
public:
    KinematicsKUKAiiwa(double SamplingTime_ = 0.01, size_t numOfAxis_ = 7, const YMatrix& EE2TCP_ = YMatrix::identity(4));

    // Complete Forward Kinematics (as shown in previous artifact)
    YMatrix forwardKinematics(const std::vector<double>& q) override;
    
    // Complete Jacobian calculation 
    YMatrix calculateJacobian(const std::vector<double>& q) override;
    
    
};