/*
    This code is publicly available on GitHub.
    You are free to use, modify, and distribute it.
    However, you must clearly credit the source as follows:

    Source: -
    Author: jaeyun Sim
    Email: wodbs02221@gmail.com
*/

#pragma once

#include "Y2Matrix/YMatrix.hpp"
#include <vector>
#include <cmath>


// 확장된 Quaternion 구조체 (YMatrix의 Quaternion을 기반으로 연산자 추가)
struct QuaternionEx {
    double w, x, y, z;
    
    QuaternionEx(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0) : w(w), x(x), y(y), z(z) {}
    
    // YMatrix::Quaternion에서 변환
    QuaternionEx(const Quaternion& q) : w(q.w), x(q.x), y(q.y), z(q.z) {}
    
    // YMatrix::Quaternion으로 변환
    Quaternion toQuaternion() const {
        return {w, x, y, z};
    }
    
    // 덧셈 연산자 (일반적으로 쿼터니언에서는 사용하지 않지만 보간에 필요)
    QuaternionEx operator+(const QuaternionEx& other) const {
        return QuaternionEx(w + other.w, x + other.x, y + other.y, z + other.z);
    }
    
    // 뺄셈 연산자
    QuaternionEx operator-(const QuaternionEx& other) const {
        return QuaternionEx(w - other.w, x - other.x, y - other.y, z - other.z);
    }
    
    // 스칼라 곱셈 연산자
    QuaternionEx operator*(double scalar) const {
        return QuaternionEx(w * scalar, x * scalar, y * scalar, z * scalar);
    }
    
    // 쿼터니언 곱셈 (Hamilton product)
    QuaternionEx operator*(const QuaternionEx& other) const {
        return QuaternionEx(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }
    
    // 내적
    double dot(const QuaternionEx& other) const {
        return w * other.w + x * other.x + y * other.y + z * other.z;
    }
    
    // 크기(magnitude) 계산
    double magnitude() const {
        return sqrt(w*w + x*x + y*y + z*z);
    }
    
    // 정규화
    QuaternionEx normalized() const {
        double mag = magnitude();
        if (mag < 1e-10) {
            return QuaternionEx(1.0, 0.0, 0.0, 0.0);
        }
        return QuaternionEx(w/mag, x/mag, y/mag, z/mag);
    }
    
    // 켤레(conjugate)
    QuaternionEx conjugate() const {
        return QuaternionEx(w, -x, -y, -z);
    }
    
    // 역원(inverse)
    QuaternionEx inverse() const {
        double magSq = w*w + x*x + y*y + z*z;
        QuaternionEx conj = conjugate();
        return conj * (1.0 / magSq);
    }
    
    // 두 쿼터니언 사이의 각도
    double angleTo(const QuaternionEx& other) const {
        double dotProduct = abs(dot(other.normalized()));
        if (dotProduct > 1.0) dotProduct = 1.0;
        return 2.0 * acos(dotProduct);
    }
};

class QuaternionInterpolator {

public:
    // SLERP (Spherical Linear Interpolation)
    static QuaternionEx slerp(const QuaternionEx& start, const QuaternionEx& end, double t);
    
    // 두 쿼터니언 사이를 지정된 개수로 보간
    static std::vector<QuaternionEx> interpolate(const QuaternionEx& start, const QuaternionEx& end, int numSteps);
    
    // 여러 쿼터니언을 경유하는 회전 시퀀스 생성
    static std::vector<QuaternionEx> interpolateMultiPoint(const std::vector<QuaternionEx>& waypoints, int stepsPerSegment);
    
    // 원형 회전 경로 생성 (축 기준으로 회전)
    static std::vector<QuaternionEx> generateCircularRotation(const QuaternionEx& center, 
                                                              const QuaternionEx& axisRotation, 
                                                              double totalAngle, int numPoints);
    
    // Squad (Spherical Quadrangle) 보간 - 부드러운 곡선 보간 (4개의 제어점 사용용)
    static std::vector<QuaternionEx> squadInterpolate(const std::vector<QuaternionEx>& controlPoints, int pointsPerSegment);
    
    // 연속적인 회전을 위한 적응형 보간 (각속도 기반, 각속도 제한 기반 동적 스텝 조정정)
    static std::vector<QuaternionEx> adaptiveInterpolate(const QuaternionEx& start, const QuaternionEx& end, 
                                                         double maxAngularStep);
    
    // 쿼터니언을 출력하는 헬퍼 함수
    static void printQuaternion(const QuaternionEx& q, const std::string& name = "");
    
    // 쿼터니언 배열을 출력하는 헬퍼 함수
    static void printQuaternionArray(const std::vector<QuaternionEx>& quaternions);
    
    // RPY로 변환하여 출력
    static void printAsRPY(const QuaternionEx& q, const std::string& name = "");
    
    // 회전 시퀀스의 총 회전각 계산
    static double calculateTotalRotation(const std::vector<QuaternionEx>& rotations);
    
    // 각속도 계산 (연속된 쿼터니언들 사이의)
    static std::vector<double> calculateAngularVelocities(const std::vector<QuaternionEx>& rotations, double timeStep);

private:
    // Squad 보간을 위한 중간 쿼터니언 계산
    static QuaternionEx computeIntermediate(const QuaternionEx& prev, const QuaternionEx& curr, const QuaternionEx& next);
    
    // Squad 보간 계산
    static QuaternionEx squad(const QuaternionEx& q0, const QuaternionEx& q1, 
                             const QuaternionEx& q2, const QuaternionEx& q3, double t);

    // 쿼터니언의 최단 경로 선택
    static QuaternionEx selectShortestPath(const QuaternionEx& from, const QuaternionEx& to);
};