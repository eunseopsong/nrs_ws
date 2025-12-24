#include <iostream>
#include <vector>
#include <stdexcept>
#include "Y2Matrix/YMatrix.hpp"
#include "Y2Kinematics/KinematicsKUKAiiwa.hpp"

// Complete example usage
int main() {
    // Create an instance of KinematicsKUKAiiwa
    KinematicsKUKAiiwa KUKAKinematics(0.01, static_cast<size_t>(7), YMatrix::identity(4)); // Sampling time 10ms, 7 DOF, HTM of EE to TCP as identity matrix
    
    // Set QP - parameters
    KUKAKinematics.setControlGains(1.0, 1.0);
    KUKAKinematics.setQPWeights(1.0, 0.1, 0.01);

    // Set joint limits
    std::vector<double> q_min = {-2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI};
    std::vector<double> q_max = { 2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI};
    std::vector<double> qd_min = {-314, -314, -314, -314, -314, -314, -314};
    std::vector<double> qd_max = { 314,  314,   314,  314,  314,  314,  314};
    KUKAKinematics.setJointLimits(q_min, q_max, qd_min, qd_max);
    
    // Initial joint configuration (Need real-time update)
    std::vector<double> q_current = {0, -M_PI/2, 0, M_PI/2, 0, 0, 0};
    
    // Calculate target (move 10mm in x direction)
    YMatrix current_HTM = KUKAKinematics.forwardKinematics(q_current);
    YMatrix target_HTM = current_HTM;
    target_HTM[0][3] += 10.0;
    
    std::cout << "=== QP-based Inverse Kinematics Results ===" << std::endl;
    KUKAKinematics.printPose(current_HTM, "Initial");
    KUKAKinematics.printPose(target_HTM, "Target");
    
    // Solve IK
    std::vector<double> q_solution = KUKAKinematics.solve_IK(q_current, target_HTM);
    
    // Verify result
    YMatrix result_HTM = KUKAKinematics.forwardKinematics(q_solution);
    KUKAKinematics.printPose(result_HTM, "Result");
    
    // Print joint angles
    std::cout << "Joint angles (degrees): ";
    for (size_t i = 0; i < q_solution.size(); ++i) {
        std::cout << q_solution[i] * 180.0 / M_PI;
        if (i < q_solution.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    
    return 0;
}