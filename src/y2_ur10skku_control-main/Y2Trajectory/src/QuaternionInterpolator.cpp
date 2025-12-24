#include "Y2Trajectory/QuaternionInterpolator.hpp"

// 쿼터니언의 최단 경로 선택
QuaternionEx QuaternionInterpolator::selectShortestPath(const QuaternionEx& from, const QuaternionEx& to) {
    if (from.dot(to) < 0.0) {
        return to * (-1.0);
    }
    return to;
}

// SLERP (Spherical Linear Interpolation)
QuaternionEx QuaternionInterpolator::slerp(const QuaternionEx& start, const QuaternionEx& end, double t) {
    t = std::max(0.0, std::min(1.0, t));
    
    QuaternionEx from = start.normalized();
    QuaternionEx to = selectShortestPath(from, end.normalized());
    
    double cosOmega = from.dot(to);
    
    // 거의 같은 쿼터니언인 경우 선형 보간
    if (cosOmega > 0.9995) {
        QuaternionEx result = from * (1.0 - t) + to * t;
        return result.normalized();
    }
    
    // SLERP 계산
    double omega = acos(abs(cosOmega));
    double sinOmega = sin(omega);
    
    double fromFactor = sin((1.0 - t) * omega) / sinOmega;
    double toFactor = sin(t * omega) / sinOmega;
    
    QuaternionEx result = from * fromFactor + to * toFactor;
    return result.normalized();
}
    
// 두 쿼터니언 사이를 지정된 개수로 보간
std::vector<QuaternionEx> QuaternionInterpolator::interpolate(const QuaternionEx& start, const QuaternionEx& end, int numSteps) {
    std::vector<QuaternionEx> result;
    
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
        QuaternionEx interpolated = slerp(start, end, t);
        result.push_back(interpolated);
    }
    
    // 끝점 추가
    result.push_back(end);
    
    return result;
}
    
// 여러 쿼터니언을 경유하는 회전 시퀀스 생성
std::vector<QuaternionEx> QuaternionInterpolator::interpolateMultiPoint(const std::vector<QuaternionEx>& waypoints, int stepsPerSegment) {
    std::vector<QuaternionEx> result;
    
    if (waypoints.size() < 2) {
        return waypoints;
    }
    
    // 첫 번째 점 추가
    result.push_back(waypoints[0]);
    
    // 각 구간별로 보간
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        std::vector<QuaternionEx> segment = interpolate(waypoints[i], waypoints[i + 1], stepsPerSegment);
        
        // 첫 번째 점은 이미 추가되었으므로 제외하고 추가
        for (size_t j = 1; j < segment.size(); ++j) {
            result.push_back(segment[j]);
        }
    }
    
    return result;
}
    
// 원형 회전 경로 생성 (축 기준으로 회전)
std::vector<QuaternionEx> QuaternionInterpolator::generateCircularRotation(const QuaternionEx& center, 
                                                            const QuaternionEx& axisRotation, 
                                                            double totalAngle, int numPoints) {
    std::vector<QuaternionEx> result;
    
    for (int i = 0; i < numPoints; ++i) {
        double angle = totalAngle * i / (numPoints - 1);
        
        // 축 기준 회전 쿼터니언 생성
        QuaternionEx rotation = axisRotation * sin(angle / 2.0);
        rotation.w = cos(angle / 2.0);
        rotation = rotation.normalized();
        
        // 중심 회전에 적용
        QuaternionEx finalRotation = center * rotation;
        result.push_back(finalRotation.normalized());
    }
    
    return result;
}
    
// Squad (Spherical Quadrangle) 보간 - 부드러운 곡선 보간 (4개의 제어점 사용용)
std::vector<QuaternionEx> QuaternionInterpolator::squadInterpolate(const std::vector<QuaternionEx>& controlPoints, int pointsPerSegment) {
    std::vector<QuaternionEx> result;
    
    if (controlPoints.size() < 4) {
        return interpolateMultiPoint(controlPoints, pointsPerSegment);
    }
    
    for (size_t i = 0; i < controlPoints.size() - 3; ++i) {
        for (int j = 0; j < pointsPerSegment; ++j) {
            double t = static_cast<double>(j) / pointsPerSegment;
            QuaternionEx point = squad(controlPoints[i], controlPoints[i+1], 
                                        controlPoints[i+2], controlPoints[i+3], t);
            result.push_back(point);
        }
    }
    
    // 마지막 점 추가
    result.push_back(controlPoints.back());
    
    return result;
}
    
// 연속적인 회전을 위한 적응형 보간 (각속도 기반, 각속도 제한 기반 동적 스텝 조정정)
std::vector<QuaternionEx> QuaternionInterpolator::adaptiveInterpolate(const QuaternionEx& start, const QuaternionEx& end, 
                                                        double maxAngularStep) {
    std::vector<QuaternionEx> result;
    
    double totalAngle = start.angleTo(end);
    int numSteps = static_cast<int>(ceil(totalAngle / maxAngularStep)) + 1;
    
    return interpolate(start, end, numSteps);
}
    
// Squad 보간을 위한 중간 쿼터니언 계산
QuaternionEx QuaternionInterpolator::computeIntermediate(const QuaternionEx& prev, const QuaternionEx& curr, const QuaternionEx& next) {
    QuaternionEx q1 = selectShortestPath(curr, prev);
    QuaternionEx q2 = selectShortestPath(curr, next);
    
    // 로그 계산 근사
    QuaternionEx sum = q1 + q2;
    QuaternionEx intermediate = curr * QuaternionEx(
        exp(-0.25 * sum.w),
        -0.25 * sum.x,
        -0.25 * sum.y,
        -0.25 * sum.z
    ).normalized();
    
    return intermediate.normalized();
}
    
// Squad 보간 계산
QuaternionEx QuaternionInterpolator::squad(const QuaternionEx& q0, const QuaternionEx& q1, 
                            const QuaternionEx& q2, const QuaternionEx& q3, double t) {
    QuaternionEx s1 = computeIntermediate(q0, q1, q2);
    QuaternionEx s2 = computeIntermediate(q1, q2, q3);
    
    QuaternionEx slerp1 = slerp(q1, q2, t);
    QuaternionEx slerp2 = slerp(s1, s2, t);
    
    return slerp(slerp1, slerp2, 2.0 * t * (1.0 - t));
}
    
// 쿼터니언을 출력하는 헬퍼 함수
void QuaternionInterpolator::printQuaternion(const QuaternionEx& q, const std::string& name) {
    if (!name.empty()) {
        std::cout << name << ": ";
    }
    std::cout << "w=" << std::setprecision(6) << q.w 
                << ", x=" << q.x 
                << ", y=" << q.y 
                << ", z=" << q.z 
                << " (|q|=" << q.magnitude() << ")" << std::endl;
}

// 쿼터니언 배열을 출력하는 헬퍼 함수
void QuaternionInterpolator::printQuaternionArray(const std::vector<QuaternionEx>& quaternions) {
    for (size_t i = 0; i < quaternions.size(); ++i) {
        std::cout << "Step " << i + 1 << " - ";
        printQuaternion(quaternions[i]);
    }
}

// RPY로 변환하여 출력
void QuaternionInterpolator::printAsRPY(const QuaternionEx& q, const std::string& name) {
    Quaternion ymatQ = q.toQuaternion();
    RPY rpy = YMatrix::quaternionToRPY(ymatQ);
    
    if (!name.empty()) {
        std::cout << name << ": ";
    }
    std::cout << "Roll=" << rpy.roll * 180.0/M_PI << "°, "
                << "Pitch=" << rpy.pitch * 180.0/M_PI << "°, "
                << "Yaw=" << rpy.yaw * 180.0/M_PI << "°" << std::endl;
}
    
// 회전 시퀀스의 총 회전각 계산
double QuaternionInterpolator::calculateTotalRotation(const std::vector<QuaternionEx>& rotations) {
    double totalAngle = 0.0;
    for (size_t i = 1; i < rotations.size(); ++i) {
        totalAngle += rotations[i-1].angleTo(rotations[i]);
    }
    return totalAngle;
}

// 각속도 계산 (연속된 쿼터니언들 사이의)
std::vector<double> QuaternionInterpolator::calculateAngularVelocities(const std::vector<QuaternionEx>& rotations, double timeStep) {
    std::vector<double> velocities;
    for (size_t i = 1; i < rotations.size(); ++i) {
        double angle = rotations[i-1].angleTo(rotations[i]);
        velocities.push_back(angle / timeStep);
    }
    return velocities;
}

