/*
    This code is publicly available on GitHub.
    You are free to use, modify, and distribute it.
    However, you must clearly credit the source as follows:

    Source: https://github.com/kinggotgamja/nrs_blender_pkg
    Author: Jaeyoon Shim
*/

#include "Y2Trajectory/AccProfiler.hpp"

// Profiler Constructor
Profiler::Profiler(const std::vector<double>& data, 
                   double StartingTime, double LastRestingTime, 
                   double AccelerationTime, double SamplingTime)
    : data(data), StartingTime(StartingTime), 
      LastRestingTime(LastRestingTime), AccelerationTime(AccelerationTime), 
      SamplingTime(SamplingTime) {}

// NRSProfiler Constructor
YAccProfiler::YAccProfiler(const std::vector<double>& data, 
                         double StartingTime, double LastRestingTime, 
                         double AccelerationTime, double SamplingTime)
    : Profiler(data, StartingTime, LastRestingTime, AccelerationTime, SamplingTime) {}

// NRS Acc-Dcc Profiling
YMatrix YAccProfiler::AccDecProfiling() {
    YMatrix Final_pos_interval(1,2);
    std::vector<double> Target_velocity(data.size(), 0.0);
    double Ti = StartingTime;
    double Ta = AccelerationTime;
    double Ts = SamplingTime;
    double Tl = LastRestingTime;
    double Tf = Ti + Ts * data.size() + Tl;

    // Target Velocity Calculation
    for (size_t j = 1; j < data.size(); j++) {
        Target_velocity[j] = (data[j] - data[j - 1]) / Ts;
    }

    std::vector<double> t;
    for (double i = 0; i <= Tf; i += Ts) {
        t.push_back(i);
    }
 
    // Allocate data to interpolated matrix
    YMatrix Interpolated(t.size(), 2);
    size_t Last_flag = 0;
    for (size_t i = 0; i < t.size(); i++) {
        if (t[i] <= Ti) {
            Interpolated[i] = { t[i], 0 };
            Last_flag++;
        } else if (t[i] <= Ti + Ts * data.size()) {
            Interpolated[i] = { t[i], Target_velocity[i - Last_flag] };
        } else {
            Interpolated[i] = { t[i], 0 };
        }
    }

    // Velocity Profiling
    double m = Ta / Ts;
    YMatrix Final(t.size(), 2);
    Final_pos_interval[0] = {t[0], data[0]};

    for (size_t i = 1; i < t.size(); i++) {
        if (i <= m) {
            Final[i] = { t[i], Final[i - 1][1] + (Interpolated[i][1] - Interpolated[0][1]) / (i) };
        } else {
            Final[i] = { t[i], Final[i - 1][1] + (Interpolated[i][1] - Interpolated[i - (int)m][1]) / m };
        }
        Final_pos_interval.appendV({{ t[i], Final_pos_interval[i - 1][1] + Final[i][1] * Ts }});
    }
    
    return Final_pos_interval;
}

// Display function
void Profiler::DisplayData(const YMatrix& outputData) {
    for (int i = 0; i < outputData.rows(); i++) {
        auto row = outputData[i];
        std::cout << row[0] << ", " << row[1] << std::endl;
    }
}

