#ifndef JOINTCONTROL_H
#define JOINTCONTROL_H

#include "func.h"

class JointControl : public rclcpp::Node
{
public:
    JointControl();

private:
    void CalculateAndPublishJoint();

    // // Subscriber declarations


    // // Publisher declarations


    // // Timer


};

#endif // JOINTCONTROL_H