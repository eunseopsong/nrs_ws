#include "Y2Kinematics/KinematicsKUKAiiwa.hpp"

KinematicsKUKAiiwa::KinematicsKUKAiiwa(double SamplingTime_, size_t numOfAxis_, const YMatrix& EE2TCP_) 
: Kinematics(SamplingTime_, numOfAxis_, EE2TCP_) 
{
    // Set QP - parameters
    setControlGains(1.0, 1.0);
    setQPWeights(1.0, 0.1, 0.01);

    // Set joint limits
    setJointLimits(q_min, q_max, qd_min, qd_max);
    setAccelLimits(qdd_min,qdd_max);
}


// Forward Kinematics - MATLAB 결과와 정확히 일치
YMatrix KinematicsKUKAiiwa::forwardKinematics(const std::vector<double>& q) {
    if (q.size() != numOfAxis) {
        throw std::invalid_argument("Joint angles must have 7 elements");
    }
    
    double th1 = q[0], th2 = q[1], th3 = q[2], th4 = q[3];
    double th5 = q[4], th6 = q[5], th7 = q[6];
    
    // Pre-calculate trigonometric functions
    double s1 = sin(th1), c1 = cos(th1);
    double s2 = sin(th2), c2 = cos(th2);
    double s3 = sin(th3), c3 = cos(th3);
    double s4 = sin(th4), c4 = cos(th4);
    double s5 = sin(th5), c5 = cos(th5);
    double s6 = sin(th6), c6 = cos(th6);
    double s7 = sin(th7), c7 = cos(th7);
    
    YMatrix T(4, 4);
    // Row 1 - Direct MATLAB implementation
    T[0][0] = s7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - 
              c7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + 
                  c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)));
    
    T[0][1] = c7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + 
              s7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + 
                  c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)));
    
    T[0][2] = c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - 
              s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
    
    // MATLAB: d7*(cos(th6)*(...) - sin(th6)*(...)) + d5*(...) + d3*cos(th1)*sin(th2)
    T[0][3] = d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - 
                  s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + 
              d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + 
              d3*c1*s2;
    
    // Row 2 - Direct MATLAB implementation  
    T[1][0] = c7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + 
                  c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - 
              s7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
    
    T[1][1] = -c7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - 
               s7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + 
                   c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)));
    
    T[1][2] = s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - 
              c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
    
    // MATLAB: d3*sin(th1)*sin(th2) - d5*(...) - d7*(cos(th6)*(...) - sin(th6)*(...))
    T[1][3] = d3*s1*s2 - 
              d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - 
              d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - 
                  s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)));
    
    // Row 3 - Direct MATLAB implementation
    T[2][0] = c7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4)) - 
              s7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3);
    
    T[2][1] = -c7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - 
               s7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4));
    
    T[2][2] = s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4);
    
    // MATLAB: d1 + d5*(...) + d7*(...) + d3*cos(th2)
    T[2][3] = d1 + 
              d5*(c2*c4 + c3*s2*s4) + 
              d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + 
              d3*c2;
    
    // Row 4
    T[3][0] = 0; T[3][1] = 0; T[3][2] = 0; T[3][3] = 1;
    
    return T*EE2TCP;
}
    
// Jacobian calculation - MATLAB 결과와 정확히 일치
YMatrix KinematicsKUKAiiwa::calculateJacobian(const std::vector<double>& q) {
    if (q.size() != numOfAxis) {
        throw std::invalid_argument("Joint angles must have 7 elements");
    }
    
    double th1 = q[0], th2 = q[1], th3 = q[2], th4 = q[3];
    double th5 = q[4], th6 = q[5], th7 = q[6];
    
    // Pre-calculate trigonometric functions
    double s1 = sin(th1), c1 = cos(th1);
    double s2 = sin(th2), c2 = cos(th2);
    double s3 = sin(th3), c3 = cos(th3);
    double s4 = sin(th4), c4 = cos(th4);
    double s5 = sin(th5), c5 = cos(th5);
    double s6 = sin(th6), c6 = cos(th6);
    
    YMatrix J(6, numOfAxis);
    
    // First row (Jv_x) - MATLAB 첫 번째 행
    J[0][0] = d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - 
                  s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + 
              d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - 
              d3*s1*s2;
    
    J[0][1] = c1*(d5*(c2*c4 + c3*s2*s4) + 
                  d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + 
                  d3*c2);
    
    J[0][2] = d5*c3*s1*s4 + d5*c1*c2*s3*s4 + d7*c3*c6*s1*s4 + d7*s1*s3*s5*s6 + 
              d7*c1*c2*c6*s3*s4 - d7*c1*c2*c3*s5*s6 - d7*c3*c4*c5*s1*s6 - 
              d7*c1*c2*c4*c5*s3*s6;
    
    J[0][3] = -(c1*c3 - c2*s1*s3)*(d5*(c2*c4 + c3*s2*s4) + 
                                   d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + 
                                       c6*(c2*c4 + c3*s2*s4))) - 
              s2*s3*(d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - 
                          s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + 
                              s5*(c1*c3 - c2*s1*s3))) + 
                     d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2));
    
    J[0][4] = d7*(c2*c4 + c3*s2*s4)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - 
                                     s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + 
                                         s5*(c1*c3 - c2*s1*s3))) - 
              d7*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + 
                                                      c6*(c2*c4 + c3*s2*s4));
    
    J[0][5] = -d7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - 
                                                       s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + 
                                                           s5*(c1*c3 - c2*s1*s3))) - 
               d7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*
                  (s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4));
    
    J[0][6] = 0;
    
    // Second row (Jv_y) - MATLAB 두 번째 행
    J[1][0] = d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - 
                  s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + 
              d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + 
              d3*c1*s2;
    
    J[1][1] = s1*(d5*(c2*c4 + c3*s2*s4) + 
                  d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + 
                  d3*c2);
    
    J[1][2] = d5*c2*s1*s3*s4 - d7*c1*c3*c6*s4 - d5*c1*c3*s4 - d7*c1*s3*s5*s6 + 
              d7*c1*c3*c4*c5*s6 + d7*c2*c6*s1*s3*s4 - d7*c2*c3*s1*s5*s6 - 
              d7*c2*c4*c5*s1*s3*s6;
    
    J[1][3] = -(c3*s1 + c1*c2*s3)*(d5*(c2*c4 + c3*s2*s4) + 
                                   d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + 
                                       c6*(c2*c4 + c3*s2*s4))) - 
              s2*s3*(d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - 
                          s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + 
                              s5*(c3*s1 + c1*c2*s3))) + 
                     d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2));
    
    J[1][4] = d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - 
                  s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*
                 (c2*c4 + c3*s2*s4) - 
              d7*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + 
                                                      c6*(c2*c4 + c3*s2*s4));
    
    J[1][5] = -d7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - 
                                                       s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + 
                                                           s5*(c3*s1 + c1*c2*s3))) - 
               d7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*
                  (s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4));
    
    J[1][6] = 0;
    
    // Third row (Jv_z) - MATLAB 세 번째 행
    J[2][0] = 0;
    
    J[2][1] = d5*c2*c3*s4 - d5*c4*s2 - d3*s2 - d7*c4*c6*s2 + d7*c2*c3*c6*s4 + 
              d7*c2*s3*s5*s6 - d7*c5*s2*s4*s6 - d7*c2*c3*c4*c5*s6;
    
    J[2][2] = -s2*(d5*s3*s4 + d7*c6*s3*s4 - d7*c3*s5*s6 - d7*c4*c5*s3*s6);
    
    J[2][3] = d5*c3*c4*s2 - d5*c2*s4 - d7*c2*c6*s4 + d7*c3*c4*c6*s2 + d7*c2*c4*c5*s6 + 
              d7*c3*c5*s2*s4*s6;
    
    J[2][4] = d7*s6*(c5*s2*s3 - c2*s4*s5 + c3*c4*s2*s5);
    
    J[2][5] = -d7*(c2*c4*s6 - c2*c5*c6*s4 + c3*s2*s4*s6 - c6*s2*s3*s5 + c3*c4*c5*c6*s2);
    
    J[2][6] = 0;
    
    // Fourth row (Jw_x) - MATLAB 네 번째 행
    J[3][0] = 0;
    J[3][1] = -s1;
    J[3][2] = c1*s2;
    J[3][3] = c3*s1 + c1*c2*s3;
    J[3][4] = s4*(s1*s3 - c1*c2*c3) + c1*c4*s2;
    J[3][5] = s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3);
    J[3][6] = c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - 
              s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
    
    // Fifth row (Jw_y) - MATLAB 다섯 번째 행
    J[4][0] = 0;
    J[4][1] = c1;
    J[4][2] = s1*s2;
    J[4][3] = c2*s1*s3 - c1*c3;
    J[4][4] = c4*s1*s2 - s4*(c1*s3 + c2*c3*s1);
    J[4][5] = c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4);
    J[4][6] = s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - 
              c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
    
    // Sixth row (Jw_z) - MATLAB 여섯 번째 행
    J[5][0] = 1;
    J[5][1] = 0;
    J[5][2] = c2;
    J[5][3] = -s2*s3;
    J[5][4] = c2*c4 + c3*s2*s4;
    J[5][5] = c5*s2*s3 - s5*(c2*s4 - c3*c4*s2);
    J[5][6] = s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4);
    
    return J;
}