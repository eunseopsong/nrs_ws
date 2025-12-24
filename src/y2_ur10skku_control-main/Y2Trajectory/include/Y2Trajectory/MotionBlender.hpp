#pragma once

#include "Y2Matrix/YMatrix.hpp"
#include "Y2Trajectory/PositionInterpolation.hpp"
#include "Y2Trajectory/QuaternionInterpolator.hpp"
#include "Y2Trajectory/AccProfiler.hpp"

#include <fstream>

class MotionBlender {
    public:
        // Constructor
        MotionBlender(const YMatrix& position_, const std::vector<double>& velocity_,
                        const std::vector<double>& ang_velocity_,const std::vector<double>& holding_time_, 
                        double angVelLimit_,
                        double startingTime_, double lastRestingTime_, 
                        double accelerationTime_, double samplingTime_)
            : position(position_), velocity(velocity_), ang_velocity(ang_velocity_), holding_time(holding_time_),
              angVelLimit(angVelLimit_),
              startingTime(startingTime_), lastRestingTime(lastRestingTime_), 
              accelerationTime(accelerationTime_), samplingTime(samplingTime_) {}

        virtual YMatrix blendMotion(double Defualt_travelTime = 5) = 0;

    protected:
        YMatrix position;  // Position data
        std::vector<double> velocity;  // Velocity data
        std::vector<double> ang_velocity;  // Angular velocity data
        std::vector<double> holding_time;  // Holding_time data

        double angVelLimit;  // Angular velocity limits
        double startingTime;  // Starting time of the motion
        double lastRestingTime;  // Time when the motion comes to rest
        double accelerationTime;  // Time taken to accelerate
        double samplingTime;  // Sampling time interval
};