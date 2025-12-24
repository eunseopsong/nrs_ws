#include "Y2Trajectory/PositionInterpolation.hpp"

// 사용 예제
int main() {
    // 예제 1: 두 점 사이의 직선 보간
    std::cout << "=== 직선 보간 예제 ===" << std::endl;
    Position3D start(0.0, 0.0, 0.0);
    Position3D end(10.0, 5.0, 3.0);
    
    std::cout << "시작점: ";
    PositionInterpolator::printPosition(start);
    std::cout << "끝점: ";
    PositionInterpolator::printPosition(end);
    std::cout << std::endl;
    
    // 6개의 스텝으로 보간
    int steps = 6;
    std::vector<Position3D> linearPath = PositionInterpolator::interpolate(start, end, steps);
    
    std::cout << steps << "개 스텝으로 보간된 직선 경로:" << std::endl;
    PositionInterpolator::printPositionArray(linearPath);
    std::cout << "경로 길이: " << PositionInterpolator::calculatePathLength(linearPath) << std::endl;
    std::cout << std::endl;
    
    // 예제 2: 여러 경유점을 통한 경로
    std::cout << "=== 다중 경유점 보간 예제 ===" << std::endl;
    std::vector<Position3D> waypoints = {
        Position3D(0.0, 0.0, 0.0),
        Position3D(5.0, 3.0, 2.0),
        Position3D(8.0, 1.0, 5.0),
        Position3D(12.0, 4.0, 3.0)
    };
    
    std::cout << "경유점들:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "Waypoint " << i + 1 << " - ";
        PositionInterpolator::printPosition(waypoints[i]);
    }
    std::cout << std::endl;
    
    std::vector<Position3D> multiPath = PositionInterpolator::interpolateMultiPoint(waypoints, 4);
    std::cout << "각 구간을 4개 스텝으로 보간한 경로 (" << multiPath.size() << "개 점):" << std::endl;
    PositionInterpolator::printPositionArray(multiPath);
    std::cout << "경로 길이: " << PositionInterpolator::calculatePathLength(multiPath) << std::endl;
    std::cout << std::endl;
    
    // 예제 3: 원형 경로
    std::cout << "=== 원형 경로 예제 ===" << std::endl;
    Position3D circleCenter(5.0, 5.0, 0.0);
    double radius = 3.0;
    Position3D normal(0.0, 0.0, 1.0);  // XY 평면의 원
    
    std::vector<Position3D> circlePath = PositionInterpolator::generateCirclePath(circleCenter, radius, normal, 8);
    std::cout << "원형 경로 (중심: (5,5,0), 반지름: 3.0, 8개 점):" << std::endl;
    PositionInterpolator::printPositionArray(circlePath);
    std::cout << std::endl;
    
    // 예제 4: 스플라인 보간
    std::cout << "=== 스플라인 보간 예제 ===" << std::endl;
    std::vector<Position3D> controlPoints = {
        Position3D(0.0, 0.0, 0.0),
        Position3D(2.0, 4.0, 1.0),
        Position3D(6.0, 3.0, 3.0),
        Position3D(8.0, 1.0, 2.0),
        Position3D(10.0, 5.0, 4.0)
    };
    
    std::vector<Position3D> splinePath = PositionInterpolator::splineInterpolate(controlPoints, 5);
    std::cout << "Catmull-Rom 스플라인 경로 (" << splinePath.size() << "개 점):" << std::endl;
    PositionInterpolator::printPositionArray(splinePath);
    std::cout << "경로 길이: " << PositionInterpolator::calculatePathLength(splinePath) << std::endl;
    
    return 0;
}