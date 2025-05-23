#include <iostream>
#include <thread>
#include <vector>
#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_io_interface.h"

using namespace ur_rtde;
using namespace std;

// string robot_ip = "192.168.0.47";
// double rtde_frequency = 500.0; // 안정적인 주파수
// uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
// RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags);
// RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency);

string robot_ip = "192.168.0.47";
double rtde_frequency = 500.0; // Hz 500
uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
int ur_cap_port = 50002;

/* ur_rtde realtime priorities */
int rt_receive_priority = 90;
int rt_control_priority = 85;

RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);
RTDEIOInterface rtde_io(robot_ip);

int main() {

    try {


        for(int i = 0; i< 100; i++){
            if (!rtde_control.isProgramRunning()) {
                std::cerr << "RTDE 프로그램이 실행되고 있지 않습니다!"<< std::endl;
                return -1;
            }

            if (!rtde_receive.isConnected()) {
                std::cerr << "RTDE와 로봇 간 연결이 끊어졌습니다!" << std::endl;
                return -1;
            }

            // TCP 위치 확인
            try {
                vector<double> tcp_pose = rtde_receive.getActualTCPPose();
                std::cout << "현재 TCP 위치: ";
                for (const auto &value : tcp_pose) {
                    std::cout << value << " ";
                }
                std::cout << ":" << i<< std::endl;
            } catch (const std::exception &e) {
                std::cerr << "TCP 위치 가져오기 오류: " << e.what() << std::endl;
            }

        }

    } catch (const std::exception &e) {
        std::cerr << "오류 발생: " << e.what() << std::endl;
    }

    return 0;
}
