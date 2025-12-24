#pragma once

#include "Y2Trajectory/MotionBlender.hpp"
#include <filesystem>

// position_ =  (x, y, z, wx, wy, wz, fx, fy, fz)
// velocity_ = Linear velocity beween each position)
// angVelLimit_ = Angular velocity limit (rad/s)

#define FROFILE_DOF 9

class MotionBlender9D : public MotionBlender {
    public:
        // Constructor
        MotionBlender9D(const YMatrix& position_, const std::vector<double>& velocity_,
                        const std::vector<double>& ang_velocity_, const std::vector<double>& holding_time_,
                        double angVelLimit_,
                        double startingTime_, double lastRestingTime_, 
                        double accelerationTime_, double samplingTime_)
                        : MotionBlender(position_, velocity_,ang_velocity_,holding_time_, angVelLimit_,
                        startingTime_, lastRestingTime_, accelerationTime_, samplingTime_) {}

        YMatrix blendMotion(double Defualt_travelTime = 5) override;
};