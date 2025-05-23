#include "ROSManager.h"

ROSManager::ROSManager(const std::string& robot_ip, double rtde_frequency, int rt_control_priority, int rt_receive_priority) {
    robot_control = new NRSUR10eControl(robot_ip, rtde_frequency, rt_control_priority, rt_receive_priority);
}

void ROSManager::initialize() {
    robot_control->initialize();  // Initialize the control system (subscribers, publishers, etc.)
}

void ROSManager::run() {
    robot_control->runControlLoop();  // Run the main control loop
}

void ROSManager::shutdown() {
    robot_control->shutdown();  // Clean shutdown
    delete robot_control;
}
