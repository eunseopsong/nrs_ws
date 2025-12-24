#include "Y2Trajectory/QuaternionInterpolator.hpp"

// 사용 예제
int main() {
    // 예제 1: 기본 SLERP 보간
    std::cout << "=== 기본 SLERP 보간 예제 ===" << std::endl;
    QuaternionEx start(1.0, 0.0, 0.0, 0.0);  // 단위 쿼터니언
    QuaternionEx end(0.707, 0.0, 0.707, 0.0);  // Y축 기준 90도 회전
    
    std::cout << "시작 쿼터니언: ";
    QuaternionInterpolator::printQuaternion(start);
    QuaternionInterpolator::printAsRPY(start, "RPY");
    
    std::cout << "끝 쿼터니언: ";
    QuaternionInterpolator::printQuaternion(end);
    QuaternionInterpolator::printAsRPY(end, "RPY");
    std::cout << std::endl;
    
    // 5개의 스텝으로 보간
    int steps = 5;
    std::vector<QuaternionEx> slerpPath = QuaternionInterpolator::interpolate(start, end, steps);
    
    std::cout << steps << "개 스텝으로 보간된 회전 경로:" << std::endl;
    for (size_t i = 0; i < slerpPath.size(); ++i) {
        std::cout << "Step " << i + 1 << " - ";
        QuaternionInterpolator::printQuaternion(slerpPath[i]);
        QuaternionInterpolator::printAsRPY(slerpPath[i], "       RPY");
    }
    std::cout << "총 회전각: " << QuaternionInterpolator::calculateTotalRotation(slerpPath) * 180.0/M_PI << "°" << std::endl;
    std::cout << std::endl;
    
    // 예제 2: 다중 경유점 회전 시퀀스
    std::cout << "=== 다중 경유점 회전 시퀀스 예제 ===" << std::endl;
    std::vector<QuaternionEx> waypoints = {
        QuaternionEx(YMatrix::rpyToQuaternion(0.0, 0.0, 0.0)),           // 0, 0, 0
        QuaternionEx(YMatrix::rpyToQuaternion(M_PI/6, 0.0, 0.0)),       // 30도 Roll
        QuaternionEx(YMatrix::rpyToQuaternion(M_PI/6, M_PI/4, 0.0)),    // + 45도 Pitch
        QuaternionEx(YMatrix::rpyToQuaternion(0.0, M_PI/4, M_PI/3))     // 60도 Yaw로 변경
    };
    
    std::cout << "회전 경유점들:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "Waypoint " << i + 1 << " - ";
        QuaternionInterpolator::printAsRPY(waypoints[i]);
    }
    std::cout << std::endl;
    
    std::vector<QuaternionEx> multiPath = QuaternionInterpolator::interpolateMultiPoint(waypoints, 4);
    std::cout << "각 구간을 4개 스텝으로 보간한 회전 시퀀스 (" << multiPath.size() << "개 점):" << std::endl;
    for (size_t i = 0; i < multiPath.size(); ++i) {
        if (i % 3 == 0) {  // 일부만 출력
            std::cout << "Step " << i + 1 << " - ";
            QuaternionInterpolator::printAsRPY(multiPath[i]);
        }
    }
    std::cout << "총 회전각: " << QuaternionInterpolator::calculateTotalRotation(multiPath) * 180.0/M_PI << "°" << std::endl;
    std::cout << std::endl;
    
    // 예제 3: 적응형 보간 (최대 각속도 제한)
    std::cout << "=== 적응형 보간 예제 ===" << std::endl;
    QuaternionEx bigStart(1.0, 0.0, 0.0, 0.0);
    QuaternionEx bigEnd(YMatrix::rpyToQuaternion(0.0, 0.0, M_PI));  // 180도 Yaw 회전
    
    double maxAngularStep = M_PI / 12;  // 15도 최대 스텝
    std::vector<QuaternionEx> adaptivePath = QuaternionInterpolator::adaptiveInterpolate(bigStart, bigEnd, maxAngularStep);
    
    std::cout << "180도 회전을 15도 최대 스텝으로 적응형 보간 (" << adaptivePath.size() << "개 점):" << std::endl;
    for (size_t i = 0; i < adaptivePath.size(); ++i) {
        std::cout << "Step " << i + 1 << " - ";
        QuaternionInterpolator::printAsRPY(adaptivePath[i]);
    }
    
    // 각속도 계산
    double timeStep = 0.1;  // 0.1초 간격
    std::vector<double> angularVels = QuaternionInterpolator::calculateAngularVelocities(adaptivePath, timeStep);
    std::cout << "각속도 (rad/s): ";
    for (double vel : angularVels) {
        std::cout << vel << " ";
    }
    std::cout << std::endl << std::endl;
    
    // 예제 4: Squad 보간
    std::cout << "=== Squad 보간 예제 ===" << std::endl;
    std::vector<QuaternionEx> controlPoints = {
        QuaternionEx(YMatrix::rpyToQuaternion(0.0, 0.0, 0.0)),
        QuaternionEx(YMatrix::rpyToQuaternion(M_PI/4, 0.0, 0.0)),
        QuaternionEx(YMatrix::rpyToQuaternion(M_PI/4, M_PI/4, 0.0)),
        QuaternionEx(YMatrix::rpyToQuaternion(0.0, M_PI/4, M_PI/4)),
        QuaternionEx(YMatrix::rpyToQuaternion(0.0, 0.0, M_PI/2))
    };
    
    std::vector<QuaternionEx> squadPath = QuaternionInterpolator::squadInterpolate(controlPoints, 5);
    std::cout << "Squad 보간된 부드러운 회전 경로 (" << squadPath.size() << "개 점):" << std::endl;
    for (size_t i = 0; i < squadPath.size(); i += 3) {  // 일부만 출력
        std::cout << "Step " << i + 1 << " - ";
        QuaternionInterpolator::printAsRPY(squadPath[i]);
    }
    std::cout << "총 회전각: " << QuaternionInterpolator::calculateTotalRotation(squadPath) * 180.0/M_PI << "°" << std::endl;
    
    return 0;
}