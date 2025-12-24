#include "Y2Matrix/YMatrix.hpp"

int main() {

    // 예제 1: RPY에서 회전 행렬로 변환
    std::cout << "=== Example 1: RPY to Rotation Matrix ===" << std::endl;
    double roll = 0.1, pitch = 0.2, yaw = 0.3;  // 라디안
    YMatrix R_from_rpy = YMatrix::fromRPY(roll, pitch, yaw);
    
    std::cout << "Original RPY (rad): Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw << std::endl;
    std::cout << "Rotation Matrix from RPY:" << std::endl;
    R_from_rpy.print();
    
    // 회전 행렬을 다시 RPY로 변환
    std::vector<double> rpy_converted = R_from_rpy.toRPY();
    std::cout << "Converted back to RPY: Roll=" << rpy_converted[0] 
              << ", Pitch=" << rpy_converted[1] << ", Yaw=" << rpy_converted[2] << std::endl;
    std::cout << std::endl;
    
    // 예제 2: Quaternion에서 회전 행렬로 변환
    std::cout << "=== Example 2: Quaternion to Rotation Matrix ===" << std::endl;
    double w = 0.9659, x = 0.0, y = 0.0, z = 0.2588;  // 30도 Z축 회전에 해당
    YMatrix R_from_quat = YMatrix::fromQuaternion(w, x, y, z);
    
    std::cout << "Original Quaternion: w=" << w << ", x=" << x << ", y=" << y << ", z=" << z << std::endl;
    std::cout << "Rotation Matrix from Quaternion:" << std::endl;
    R_from_quat.print();
    
    // 회전 행렬을 다시 Quaternion으로 변환
    std::vector<double> quat_converted = R_from_quat.toQuaternion();
    std::cout << "Converted back to Quaternion: w=" << quat_converted[0] 
              << ", x=" << quat_converted[1] << ", y=" << quat_converted[2] << ", z=" << quat_converted[3] << std::endl;
    std::cout << std::endl;
    
    // 예제 3: Angle-Axis에서 회전 행렬로 변환
    std::cout << "=== Example 3: Angle-Axis to Rotation Matrix ===" << std::endl;
    double axis_x = 0.0, axis_y = 0.0, axis_z = 1.0;  // Z축
    double angle = M_PI / 6;  // 30도
    YMatrix R_from_aa = YMatrix::fromAngleAxis(axis_x, axis_y, axis_z, angle);
    
    std::cout << "Original Angle-Axis: axis=(" << axis_x << ", " << axis_y << ", " << axis_z 
              << "), angle=" << angle << " rad" << std::endl;
    std::cout << "Rotation Matrix from Angle-Axis:" << std::endl;
    R_from_aa.print();
    
    // 회전 행렬을 다시 Angle-Axis로 변환
    std::vector<double> aa_converted = R_from_aa.toAngleAxis();
    std::cout << "Converted back to Angle-Axis: axis=(" << aa_converted[0] << ", " 
              << aa_converted[1] << ", " << aa_converted[2] << "), angle=" << aa_converted[3] << " rad" << std::endl;
    std::cout << std::endl;
    
    // 예제 4: 다양한 변환 체인
    std::cout << "=== Example 4: Conversion Chain ===" << std::endl;
    
    // RPY -> Rotation Matrix -> Quaternion -> Rotation Matrix -> Angle-Axis
    std::cout << "Conversion Chain: RPY -> RotMat -> Quat -> RotMat -> AngleAxis" << std::endl;
    
    // 시작: RPY
    double start_roll = 0.5, start_pitch = 0.3, start_yaw = 0.8;
    std::cout << "Start RPY: (" << start_roll << ", " << start_pitch << ", " << start_yaw << ")" << std::endl;
    
    // RPY -> Rotation Matrix
    YMatrix R1 = YMatrix::fromRPY(start_roll, start_pitch, start_yaw);
    
    // Rotation Matrix -> Quaternion
    std::vector<double> quat = R1.toQuaternion();
    std::cout << "-> Quaternion: (" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << ")" << std::endl;
    
    // Quaternion -> Rotation Matrix
    YMatrix R2 = YMatrix::fromQuaternion(quat[0], quat[1], quat[2], quat[3]);
    
    // Rotation Matrix -> Angle-Axis
    std::vector<double> aa = R2.toAngleAxis();
    std::cout << "-> Angle-Axis: axis(" << aa[0] << ", " << aa[1] << ", " << aa[2] << "), angle=" << aa[3] << std::endl;
    
    // 최종 검증: Angle-Axis -> Rotation Matrix -> RPY
    YMatrix R3 = YMatrix::fromAngleAxis(aa[0], aa[1], aa[2], aa[3]);
    std::vector<double> final_rpy = R3.toRPY();
    std::cout << "-> Final RPY: (" << final_rpy[0] << ", " << final_rpy[1] << ", " << final_rpy[2] << ")" << std::endl;
    std::cout << std::endl;
    
    // 예제 5: 회전 행렬 검증
    std::cout << "=== Example 5: Rotation Matrix Validation ===" << std::endl;
    
    std::cout << "Is R1 a valid rotation matrix? " << (R1.isRotationMatrix() ? "Yes" : "No") << std::endl;
    std::cout << "Determinant of R1: " << R1.determinant() << std::endl;
    std::cout << "Frobenius norm of R1: " << R1.norm() << std::endl;
    std::cout << std::endl;
    
    // 예제 6: 회전 합성
    std::cout << "=== Example 6: Rotation Composition ===" << std::endl;
    
    YMatrix R_x = YMatrix::fromRPY(0.5, 0.0, 0.0);  // X축 회전
    YMatrix R_y = YMatrix::fromRPY(0.0, 0.3, 0.0);  // Y축 회전
    YMatrix R_z = YMatrix::fromRPY(0.0, 0.0, 0.8);  // Z축 회전
    
    std::cout << "X-axis rotation (0.5 rad):" << std::endl;
    R_x.print();
    
    std::cout << "Y-axis rotation (0.3 rad):" << std::endl;
    R_y.print();
    
    std::cout << "Z-axis rotation (0.8 rad):" << std::endl;
    R_z.print();
    
    // 회전 합성: R_combined = R_z * R_y * R_x
    YMatrix R_combined = R_z * R_y * R_x;
    std::cout << "Combined rotation (Z*Y*X):" << std::endl;
    R_combined.print();
    
    std::vector<double> combined_rpy = R_combined.toRPY();
    std::cout << "Combined rotation as RPY: (" << combined_rpy[0] << ", " 
              << combined_rpy[1] << ", " << combined_rpy[2] << ")" << std::endl;
    
    return 0;
}