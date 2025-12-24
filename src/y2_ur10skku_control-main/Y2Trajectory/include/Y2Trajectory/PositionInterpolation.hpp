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

struct Position3D {
    double x, y, z;
    
    Position3D(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}
    
    // 덧셈 연산자
    Position3D operator+(const Position3D& other) const {
        return Position3D(x + other.x, y + other.y, z + other.z);
    }
    
    // 뺄셈 연산자
    Position3D operator-(const Position3D& other) const {
        return Position3D(x - other.x, y - other.y, z - other.z);
    }
    
    // 스칼라 곱셈 연산자
    Position3D operator*(double scalar) const {
        return Position3D(x * scalar, y * scalar, z * scalar);
    }
    
    // 거리 계산
    double distance(const Position3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // 크기(norm) 계산
    double magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }
};

class PositionInterpolator {
public:
    // 두 위치 사이의 선형 보간
    static Position3D lerp(const Position3D& start, const Position3D& end, double t);
    
    // 두 위치 사이를 지정된 개수로 선형 보간
    static std::vector<Position3D> interpolate(const Position3D& start, const Position3D& end, int numSteps);
    
    // 여러 위치점들을 경유하는 경로 생성 (각 구간을 지정된 개수로 분할)
    static std::vector<Position3D> interpolateMultiPoint(const std::vector<Position3D>& waypoints, int stepsPerSegment);
    
    // 원형 경로 생성
    static std::vector<Position3D> generateCirclePath(const Position3D& center, double radius, 
                                                      const Position3D& normal, int numPoints);
    
    // 3차원 스플라인 보간 (단순한 Catmull-Rom 스플라인)
    static std::vector<Position3D> splineInterpolate(const std::vector<Position3D>& controlPoints, int pointsPerSegment);

    // 위치를 출력하는 헬퍼 함수
    static void printPosition(const Position3D& pos, const std::string& name = "");
    
    // 위치 배열을 출력하는 헬퍼 함수
    static void printPositionArray(const std::vector<Position3D>& positions);
    
    // 경로의 총 길이 계산
    static double calculatePathLength(const std::vector<Position3D>& path);

private:
    // Catmull-Rom 스플라인 계산
    static Position3D catmullRomSpline(const Position3D& p0, const Position3D& p1, 
                                       const Position3D& p2, const Position3D& p3, double t);
};