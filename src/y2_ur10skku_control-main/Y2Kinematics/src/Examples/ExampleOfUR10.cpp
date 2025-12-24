#include <iostream>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include "Y2Matrix/YMatrix.hpp"
#include "Y2Kinematics/KinematicsUR10.hpp"
#define M_PI 3.14159265358979323846
#define DegreeToRadian(degree) ((degree) * M_PI / 180.0)
#define RadianToDegree(radian) ((radian) * 180.0 / M_PI)

// Forward declaration
Quaternion slerpQuaternion(const Quaternion& q1, const Quaternion& q2, double t);

// SLERP function for quaternion interpolation
Quaternion slerpQuaternion(const Quaternion& q1, const Quaternion& q2, double t) {
    // Ensure shortest path by checking dot product
    Quaternion q2_adjusted = q2;
    double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    
    // If dot product is negative, negate q2 to ensure shortest path
    if (dot < 0.0) {
        q2_adjusted.w = -q2.w;
        q2_adjusted.x = -q2.x;
        q2_adjusted.y = -q2.y;
        q2_adjusted.z = -q2.z;
        dot = -dot;
    }
    
    // Clamp dot to avoid numerical issues
    dot = std::min(1.0, std::max(-1.0, dot));
    
    Quaternion result;
    
    // If quaternions are very close, use linear interpolation
    if (dot > 0.9995) {
        result.w = q1.w + t * (q2_adjusted.w - q1.w);
        result.x = q1.x + t * (q2_adjusted.x - q1.x);
        result.y = q1.y + t * (q2_adjusted.y - q1.y);
        result.z = q1.z + t * (q2_adjusted.z - q1.z);
    } else {
        // Spherical interpolation
        double theta_0 = acos(dot);
        double sin_theta_0 = sin(theta_0);
        double theta = theta_0 * t;
        double sin_theta = sin(theta);
        
        double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
        double s1 = sin_theta / sin_theta_0;
        
        result.w = s0 * q1.w + s1 * q2_adjusted.w;
        result.x = s0 * q1.x + s1 * q2_adjusted.x;
        result.y = s0 * q1.y + s1 * q2_adjusted.y;
        result.z = s0 * q1.z + s1 * q2_adjusted.z;
    }
    
    // Normalize the result quaternion
    double norm = sqrt(result.w * result.w + result.x * result.x + 
                      result.y * result.y + result.z * result.z);
    if (norm > 1e-10) {
        result.w /= norm;
        result.x /= norm;
        result.y /= norm;
        result.z /= norm;
    }
    
    return result;
}

// Enhanced interpolation function with orientation support
YMatrix interpolateHTM(const YMatrix& start_HTM, const YMatrix& end_HTM, double t) {
    // t: 0.0 (start) to 1.0 (end)
    
    // 1. Linear interpolation for position (translation part)
    YMatrix result(4, 4);
    for (int i = 0; i < 3; ++i) {
        result[i][3] = start_HTM[i][3] + t * (end_HTM[i][3] - start_HTM[i][3]);
    }
    
    // 2. SLERP (Spherical Linear Interpolation) for orientation
    // Extract rotation matrices
    YMatrix R_start(3, 3);
    YMatrix R_end(3, 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_start[i][j] = start_HTM[i][j];
            R_end[i][j] = end_HTM[i][j];
        }
    }
    
    // Convert rotation matrices to quaternions
    std::vector<double> q_start = R_start.toQuaternion();
    std::vector<double> q_end = R_end.toQuaternion();
    
    // Create Quaternion structs for easier handling
    Quaternion quat_start(q_start[0], q_start[1], q_start[2], q_start[3]); // w, x, y, z
    Quaternion quat_end(q_end[0], q_end[1], q_end[2], q_end[3]);
    
    // Perform SLERP interpolation
    Quaternion quat_interpolated = slerpQuaternion(quat_start, quat_end, t);
    
    // Convert interpolated quaternion back to rotation matrix
    YMatrix R_interpolated = YMatrix::fromQuaternion(quat_interpolated.w, quat_interpolated.x, 
                                                    quat_interpolated.y, quat_interpolated.z);
    
    // Assemble the final HTM
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = R_interpolated[i][j];
        }
    }
    
    // Set homogeneous coordinates
    result[3][0] = 0.0;
    result[3][1] = 0.0;
    result[3][2] = 0.0;
    result[3][3] = 1.0;
    
    return result;
}

// SLERP function for quaternion interpolation (moved before interpolateHTM)
// (This function is now defined above, before interpolateHTM function)

// Smooth interpolation using S-curve (acceleration/deceleration)
double smoothInterpolation(double t) {
    // S-curve: smooth acceleration and deceleration
    // Uses 3rd order polynomial: 3t² - 2t³
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    return 3.0 * t * t - 2.0 * t * t * t;
}

// Function to create target HTM with position and orientation changes
YMatrix createTargetHTM(const YMatrix& start_HTM, 
                        const std::vector<double>& position_offset,  // [dx, dy, dz]
                        const std::vector<double>& rotation_offset)  // [roll, pitch, yaw] in degrees
{
    YMatrix target_HTM = start_HTM;
    
    // Apply position offset
    target_HTM[0][3] += position_offset[0];  // X offset
    target_HTM[1][3] += position_offset[1];  // Y offset
    target_HTM[2][3] += position_offset[2];  // Z offset
    
    // Apply rotation offset
    double roll = DegreeToRadian(rotation_offset[0]);
    double pitch = DegreeToRadian(rotation_offset[1]);
    double yaw = DegreeToRadian(rotation_offset[2]);
    
    // Create rotation matrix from RPY
    YMatrix R_offset = YMatrix::fromRPY(roll, pitch, yaw);
    
    // Extract current rotation matrix
    YMatrix R_current(3, 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_current[i][j] = start_HTM[i][j];
        }
    }
    
    // Apply rotation: R_new = R_offset * R_current
    YMatrix R_new = R_offset * R_current;
    
    // Update rotation part of target HTM
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            target_HTM[i][j] = R_new[i][j];
        }
    }
    
    return target_HTM;
}

// Function to calculate orientation error between two HTMs
double calculateOrientationError(const YMatrix& HTM1, const YMatrix& HTM2) {
    // Extract rotation matrices
    YMatrix R1(3, 3), R2(3, 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R1[i][j] = HTM1[i][j];
            R2[i][j] = HTM2[i][j];
        }
    }
    
    // Calculate relative rotation: R_rel = R2 * R1^T
    YMatrix R1T = R1.transpose();
    YMatrix R_rel = R2 * R1T;
    
    // Calculate rotation angle from trace
    double trace = R_rel[0][0] + R_rel[1][1] + R_rel[2][2];
    double angle = acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
    
    return RadianToDegree(angle);
}

int main() {
    // Create an instance of KinematicsUR10
    KinematicsUR10 UR10Kinematics(0.01, static_cast<size_t>(6), YMatrix::identity(4)); // Sampling time 10ms (100Hz), 6 DOF
    
    // Set QP parameters
    UR10Kinematics.setControlGains(1.0, 1.0);
    UR10Kinematics.setQPWeights(1.0, 0.1, 0.01);

    // Set joint limits
    std::vector<double> q_min = {-2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI};
    std::vector<double> q_max = { 2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI,  2*M_PI};
    std::vector<double> qd_min = {-314, -314, -314, -314, -314, -314};
    std::vector<double> qd_max = { 314,  314,   314,  314,  314,  314};
    UR10Kinematics.setJointLimits(q_min, q_max, qd_min, qd_max);
    
    // Initial joint configuration
    std::vector<double> q_current = {DegreeToRadian(53.30), DegreeToRadian(-126.81), DegreeToRadian(116.34),
                                   DegreeToRadian(-79.43), DegreeToRadian(-93.48), DegreeToRadian(-26.17)};
    
    // Calculate start position
    YMatrix start_HTM = UR10Kinematics.forwardKinematics(q_current);
    
    // Define target motion: position and orientation changes
    std::vector<double> position_offset = {10.0, 5.0, -3.0};  // Move 10mm in X, 5mm in Y, -3mm in Z
    std::vector<double> rotation_offset = {15.0, -10.0, 20.0}; // Rotate 15° roll, -10° pitch, 20° yaw
    
    // Create target HTM with both position and orientation changes
    YMatrix end_HTM = createTargetHTM(start_HTM, position_offset, rotation_offset);
    
    std::cout << "=== Enhanced Interpolated Motion Control with Orientation Changes ===" << std::endl;
    std::cout << "Motion: 10 seconds, Position + Orientation interpolation" << std::endl;
    std::cout << "Position offset: [" << position_offset[0] << ", " << position_offset[1] 
              << ", " << position_offset[2] << "] mm" << std::endl;
    std::cout << "Rotation offset: [" << rotation_offset[0] << ", " << rotation_offset[1] 
              << ", " << rotation_offset[2] << "] degrees" << std::endl;
    
    UR10Kinematics.printPose(start_HTM, "Start Position");
    UR10Kinematics.printPose(end_HTM, "End Position");
    
    // Motion parameters
    const double total_time = 10.0;      // 10 seconds
    const double dt = 0.01;              // 10ms sampling time (100Hz)
    const int total_steps = static_cast<int>(total_time / dt);
    
    std::cout << "\nStarting enhanced interpolated motion..." << std::endl;
    std::cout << "Total steps: " << total_steps << std::endl;
    
    // Store trajectory data for analysis
    std::vector<std::vector<double>> joint_trajectory;
    std::vector<YMatrix> cartesian_trajectory;
    std::vector<double> orientation_errors;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Main control loop
    for (int step = 0; step < total_steps; ++step) {
        // Calculate current time and interpolation parameter
        double current_time = step * dt;
        double t_linear = current_time / total_time;  // Linear time parameter (0 to 1)
        double t_smooth = smoothInterpolation(t_linear);  // Smooth S-curve interpolation

        // Interpolate target position and orientation
        YMatrix target_HTM = interpolateHTM(start_HTM, end_HTM, t_smooth);

        // Solve inverse kinematics
        std::vector<double> q_target = UR10Kinematics.solve_IK(q_current, target_HTM);
        
        // Update current joint configuration for next iteration
        q_current = q_target;
        
        // Calculate actual HTM and errors
        YMatrix actual_HTM = UR10Kinematics.forwardKinematics(q_target);
        double orientation_error = calculateOrientationError(target_HTM, actual_HTM);
        
        // Store trajectory data
        joint_trajectory.push_back(q_target);
        cartesian_trajectory.push_back(actual_HTM);
        orientation_errors.push_back(orientation_error);
        
        // Print progress every 100 steps (1 second)
        if (step % 100 == 0) {
            std::cout << "Step " << step << " (t=" << current_time << "s): "
                      << "Pos=[" << actual_HTM[0][3] << ", " << actual_HTM[1][3] << ", " << actual_HTM[2][3] << "] mm, "
                      << "Orient_err=" << orientation_error << "°" << std::endl;
        }
        
        // Sleep to maintain 100Hz rate (optional - remove for simulation)
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "\nMotion completed!" << std::endl;
    std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
    
    // Verify final position and orientation
    YMatrix final_HTM = UR10Kinematics.forwardKinematics(q_current);
    UR10Kinematics.printPose(final_HTM, "Final Position");
    
    // Calculate position error
    double position_error = sqrt(
        pow(final_HTM[0][3] - end_HTM[0][3], 2) +
        pow(final_HTM[1][3] - end_HTM[1][3], 2) +
        pow(final_HTM[2][3] - end_HTM[2][3], 2)
    );
    
    // Calculate final orientation error
    double final_orientation_error = calculateOrientationError(end_HTM, final_HTM);
    
    std::cout << "Position error: " << position_error << " mm" << std::endl;
    std::cout << "Orientation error: " << final_orientation_error << " degrees" << std::endl;
    
    // Print final joint angles
    std::cout << "\nFinal joint angles (degrees): ";
    for (size_t i = 0; i < q_current.size(); ++i) {
        std::cout << RadianToDegree(q_current[i]);
        if (i < q_current.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    
    // Calculate and print motion statistics
    double max_orientation_error = 0.0;
    if (!orientation_errors.empty()) {
        max_orientation_error = *std::max_element(orientation_errors.begin(), orientation_errors.end());
    }
    
    double avg_orientation_error = 0.0;
    for (double err : orientation_errors) {
        avg_orientation_error += err;
    }
    if (!orientation_errors.empty()) {
        avg_orientation_error /= orientation_errors.size();
    }
    
    std::cout << "\nMotion Statistics:" << std::endl;
    std::cout << "Max orientation error: " << max_orientation_error << " degrees" << std::endl;
    std::cout << "Avg orientation error: " << avg_orientation_error << " degrees" << std::endl;
    
    // Optional: Save trajectory to file
    std::cout << "\nSaving enhanced trajectory data..." << std::endl;
    
    // Save joint trajectory
    YMatrix joint_traj_matrix(total_steps, 6);
    for (int i = 0; i < total_steps; ++i) {
        for (int j = 0; j < 6; ++j) {
            joint_traj_matrix[i][j] = RadianToDegree(joint_trajectory[i][j]);
        }
    }
    joint_traj_matrix.saveToFile("joint_trajectory_enhanced.txt", 6);
    
    // Save cartesian trajectory (position + orientation)
    YMatrix cartesian_traj_matrix(total_steps, 6);  // X, Y, Z, Roll, Pitch, Yaw
    for (int i = 0; i < total_steps; ++i) {
        // Position
        cartesian_traj_matrix[i][0] = cartesian_trajectory[i][0][3];  // X
        cartesian_traj_matrix[i][1] = cartesian_trajectory[i][1][3];  // Y
        cartesian_traj_matrix[i][2] = cartesian_trajectory[i][2][3];  // Z
        
        // Orientation (convert rotation matrix to RPY)
        YMatrix R_current(3, 3);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                R_current[row][col] = cartesian_trajectory[i][row][col];
            }
        }
        std::vector<double> rpy = R_current.toRPY();
        cartesian_traj_matrix[i][3] = RadianToDegree(rpy[0]);  // Roll
        cartesian_traj_matrix[i][4] = RadianToDegree(rpy[1]);  // Pitch
        cartesian_traj_matrix[i][5] = RadianToDegree(rpy[2]);  // Yaw
    }
    cartesian_traj_matrix.saveToFile("cartesian_trajectory_enhanced.txt", 6);
    
    // Save orientation errors
    YMatrix orientation_error_matrix(total_steps, 1);
    for (int i = 0; i < total_steps; ++i) {
        orientation_error_matrix[i][0] = orientation_errors[i];
    }
    orientation_error_matrix.saveToFile("orientation_errors.txt", 6);
    
    std::cout << "Enhanced trajectory data saved to:" << std::endl;
    std::cout << "- joint_trajectory_enhanced.txt" << std::endl;
    std::cout << "- cartesian_trajectory_enhanced.txt" << std::endl;
    std::cout << "- orientation_errors.txt" << std::endl;
    
    return 0;
}