#if 1 // cost - function q^2

#pragma once

#include "Y2Matrix/YMatrix.hpp"
#include "Y2Kinematics/QP_solver.hpp"
#include <iostream>
#include <vector>
#include <limits>

class Kinematics {
public:
    Kinematics(double SamplingTime_ = 0.01, size_t numOfAxis_ = 7, const YMatrix& EE2TCP_ = YMatrix::identity(4));

    // Forward Kinematics (Based on End-Effector)
    virtual YMatrix forwardKinematics(const std::vector<double>& q) = 0;

    // Jacobian Calculation (Based on End-Effector)
    virtual YMatrix calculateJacobian(const std::vector<double>& q) = 0;

    // Enhanced QP-based IK solver (Based on End-Effector)
    std::vector<double> solve_IK(const std::vector<double>& q_current, 
                                 const YMatrix& target_HTM);

    // Utility methods
    static double roundToNthDecimal(double value, int n);

    void setControlGains(double kp_pos, double kp_rot);
    void setQPWeights(double omega_p_val, double alpha_val, double lambda_val);

    void setJointLimits(const std::vector<double>& q_min, 
                        const std::vector<double>& q_max,
                        const std::vector<double>& qd_min, 
                        const std::vector<double>& qd_max);

    // Acceleration limits and previous state setters
    void setAccelLimits(const std::vector<double>& a_min,
                        const std::vector<double>& a_max);

    // Provide previous joint positions q_prev (k-1) explicitly, if available
    void setPrevQ(const std::vector<double>& q_prev_in);

    // Print pose in a readable format
    void printPose(const YMatrix& pose, const std::string& label);

protected:
    double dt;                      // Sampling time
    size_t numOfAxis;               // Number of joints
    YMatrix EE2TCP;                 // End-Effector to TCP transformation matrix
    int QP_precision = 4;           // Precision for rounding
    double QP_tolerance = pow(10,-QP_precision); // QP solver tolerance

    // Control gains for QP-based IK
    double Kp_pos = 1.0;
    double Kp_rot = 1.0;
        
    // QP parameters
    double omega_p = 1.0;
    double alpha = 0.1;   // default 0.1
    double lambda = 0.01;
        
    // Joint limits
    std::vector<double> q_min, q_max;
    std::vector<double> qd_min, qd_max;

    // Acceleration limits
    std::vector<double> a_min, a_max;

    // Previous joint position (k-1)
    std::vector<double> q_prev;
    bool has_prev = false;
    
private:
    QPSolver qp_solver;
};
#endif

#if 0 // cost - function jq-v
#pragma once

#include "Y2Matrix/YMatrix.hpp"
#include "Y2Kinematics/QP_solver.hpp"
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>   // std::pow

class Kinematics {
public:
    Kinematics(double SamplingTime_ = 0.01, size_t numOfAxis_ = 7, const YMatrix& EE2TCP_ = YMatrix::identity(4));

    // Forward Kinematics (Based on End-Effector)
    virtual YMatrix forwardKinematics(const std::vector<double>& q) = 0;

    // Jacobian Calculation (Based on End-Effector)
    virtual YMatrix calculateJacobian(const std::vector<double>& q) = 0;

    // QP-based IK solver (Δq formulation per slide)
    std::vector<double> solve_IK(const std::vector<double>& q_current, 
                                 const YMatrix& target_HTM);

    // Utility
    static double roundToNthDecimal(double value, int n);

    void setControlGains(double kp_pos, double kp_rot);
    // omega_p: position weight, alpha: orientation weight, lambda: damping
    void setQPWeights(double omega_p_val, double alpha_val, double lambda_val);

    void setJointLimits(const std::vector<double>& q_min, 
                        const std::vector<double>& q_max,
                        const std::vector<double>& qd_min, 
                        const std::vector<double>& qd_max);

    void setAccelLimits(const std::vector<double>& a_min,
                        const std::vector<double>& a_max);

    void setPrevQ(const std::vector<double>& q_prev_in);

    void printPose(const YMatrix& pose, const std::string& label);

protected:
    double dt;                      // Sampling time
    size_t numOfAxis;               // Number of joints
    YMatrix EE2TCP;                 // End-Effector to TCP transform
    int QP_precision = 4;
    double QP_tolerance = std::pow(10.0, -4); // 1e-4

    // Task-space gains to build Δx_des
    double Kp_pos = 1.0; // 1.0
    double Kp_rot = 1.0; // 1.0
        
    // QP weights
    double omega_p = 1.0; // position weight in W (1.0)
    double alpha   = 10.0; // orientation weight in W (0.1)
    double lambda  = 0.01; // damping (Tikhonov)

    // Joint limits
    std::vector<double> q_min, q_max;
    std::vector<double> qd_min, qd_max;

    // Acceleration limits
    std::vector<double> a_min, a_max;

    // Previous joint position (k-1)
    std::vector<double> q_prev;
    bool has_prev = false;
    
private:
    QPSolver qp_solver;
};
#endif

