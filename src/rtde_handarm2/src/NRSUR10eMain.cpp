#include <ros/ros.h>
#include "ROSManager.h"
#include <yaml-cpp/yaml.h>
#include <string>

int main(int argc, char** argv) {
    // YAML 파일 로드
    std::ifstream fin("config.yaml");  // YAML 파일 경로 설정 (예: "config.yaml")
    YAML::Node config = YAML::Load(fin);
    
    // YAML에서 robot_ip 추출
    std::string robot_ip = config["UR10IP"].as<std::string>();  // "UR10IP"는 YAML 파일 내의 robot IP 항목명

    // ROS 노드 초기화
    ros::init(argc, argv, "Yoon_UR10e_control_node");  // Initialize ROS node

    // ROSManager 인스턴스 생성 (IP를 YAML에서 읽어와 사용)
    ROSManager ros_manager(robot_ip, 500.0, 85, 90);  // Replace with appropriate priorities

    // Initialize and start control
    ros_manager.initialize();
    
    // Main loop
    ros_manager.run();

    // Graceful shutdown
    ros_manager.shutdown();
    
    return 0;
}
