#ifndef ROSMANAGER_H
#define ROSMANAGER_H

#include <ros/ros.h>
#include "NRSUR10eControl.h"

class ROSManager {
public:
    ROSManager(const std::string& robot_ip, double rtde_frequency, int rt_control_priority, int rt_receive_priority);
    void initialize();
    void run();
    void shutdown();

private:
    NRSUR10eControl* robot_control;
    ros::NodeHandle nh;
};

#endif // ROSMANAGER_H
