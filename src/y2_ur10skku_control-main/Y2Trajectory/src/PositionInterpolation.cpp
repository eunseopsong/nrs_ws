#include "Y2Trajectory/PositionInterpolation.hpp"

// 두 위치 사이의 선형 보간
Position3D PositionInterpolator::lerp(const Position3D& start, const Position3D& end, double t) {
    // t가 0~1 범위를 벗어나지 않도록 클램핑
    t = std::max(0.0, std::min(1.0, t));
    
    return Position3D(
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t,
        start.z + (end.z - start.z) * t
    );
}
    
// 두 위치 사이를 지정된 개수로 선형 보간
std::vector<Position3D> PositionInterpolator::interpolate(const Position3D& start, const Position3D& end, int numSteps) {
    std::vector<Position3D> result;
    
    if (numSteps <= 0) {
        return result;
    }
    
    if (numSteps == 1) {
        result.push_back(start);
        return result;
    }
    
    // 시작점 추가
    result.push_back(start);
    
    // 중간점들 계산
    for (int i = 1; i < numSteps - 1; ++i) {
        double t = static_cast<double>(i) / (numSteps - 1);
        Position3D interpolated = lerp(start, end, t);
        result.push_back(interpolated);
    }
    
    // 끝점 추가
    result.push_back(end);
    
    return result;
}
    
// 여러 위치점들을 경유하는 경로 생성 (각 구간을 지정된 개수로 분할)
std::vector<Position3D> PositionInterpolator::interpolateMultiPoint(const std::vector<Position3D>& waypoints, int stepsPerSegment) {
    std::vector<Position3D> result;
    
    if (waypoints.size() < 2) {
        return waypoints;
    }
    
    // 첫 번째 점 추가
    result.push_back(waypoints[0]);
    
    // 각 구간별로 보간
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        std::vector<Position3D> segment = interpolate(waypoints[i], waypoints[i + 1], stepsPerSegment);
        
        // 첫 번째 점은 이미 추가되었으므로 제외하고 추가
        for (size_t j = 1; j < segment.size(); ++j) {
            result.push_back(segment[j]);
        }
    }
    
    return result;
}
    
// 원형 경로 생성
std::vector<Position3D> PositionInterpolator::generateCirclePath(const Position3D& center, double radius, 
                                                    const Position3D& normal, int numPoints) {
    std::vector<Position3D> result;
    
    // 정규화된 법선 벡터 계산
    double normMag = normal.magnitude();
    if (normMag < 1e-10) {
        return result; // 유효하지 않은 법선 벡터
    }
    
    Position3D n = normal * (1.0 / normMag);
    
    // 법선에 수직인 두 벡터 생성 (Gram-Schmidt 과정 간소화)
    Position3D u, v;
    if (abs(n.x) < 0.9) {
        u = Position3D(1.0, 0.0, 0.0);
    } else {
        u = Position3D(0.0, 1.0, 0.0);
    }
    
    // u를 n에 수직이 되도록 조정
    double dot = u.x * n.x + u.y * n.y + u.z * n.z;
    u = u - n * dot;
    double uMag = u.magnitude();
    u = u * (1.0 / uMag);
    
    // v = n × u (외적)
    v = Position3D(
        n.y * u.z - n.z * u.y,
        n.z * u.x - n.x * u.z,
        n.x * u.y - n.y * u.x
    );
    
    // 원 위의 점들 생성
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * M_PI * i / numPoints;
        double x = radius * cos(angle);
        double y = radius * sin(angle);
        
        Position3D point = center + u * x + v * y;
        result.push_back(point);
    }
    
    return result;
}
    
// 3차원 스플라인 보간 (단순한 Catmull-Rom 스플라인)
std::vector<Position3D> PositionInterpolator::splineInterpolate(const std::vector<Position3D>& controlPoints, int pointsPerSegment) {
    std::vector<Position3D> result;
    
    if (controlPoints.size() < 4) {
        return interpolateMultiPoint(controlPoints, pointsPerSegment);
    }
    
    for (size_t i = 0; i < controlPoints.size() - 3; ++i) {
        for (int j = 0; j < pointsPerSegment; ++j) {
            double t = static_cast<double>(j) / pointsPerSegment;
            Position3D point = catmullRomSpline(controlPoints[i], controlPoints[i+1], 
                                                controlPoints[i+2], controlPoints[i+3], t);
            result.push_back(point);
        }
    }
    
    // 마지막 점 추가
    result.push_back(controlPoints.back());
    
    return result;
}
    
// Catmull-Rom 스플라인 계산
Position3D PositionInterpolator::catmullRomSpline(const Position3D& p0, const Position3D& p1, 
                                                const Position3D& p2, const Position3D& p3, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    
    Position3D result = p0 * (-0.5 * t3 + t2 - 0.5 * t) +
                        p1 * (1.5 * t3 - 2.5 * t2 + 1.0) +
                        p2 * (-1.5 * t3 + 2.0 * t2 + 0.5 * t) +
                        p3 * (0.5 * t3 - 0.5 * t2);
    
    return result;
}
    
// 위치를 출력하는 헬퍼 함수
void PositionInterpolator::printPosition(const Position3D& pos, const std::string& name) {
    if (!name.empty()) {
        std::cout << name << ": ";
    }
    std::cout << "x=" << std::setprecision(6) << pos.x 
                << ", y=" << pos.y 
                << ", z=" << pos.z << std::endl;
}
    
// 위치 배열을 출력하는 헬퍼 함수
void PositionInterpolator::printPositionArray(const std::vector<Position3D>& positions) {
    for (size_t i = 0; i < positions.size(); ++i) {
        std::cout << "Point " << i + 1 << " - ";
        printPosition(positions[i]);
    }
}

// 경로의 총 길이 계산
double PositionInterpolator::calculatePathLength(const std::vector<Position3D>& path) {
    double totalLength = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        totalLength += path[i-1].distance(path[i]);
    }
    return totalLength;
}


