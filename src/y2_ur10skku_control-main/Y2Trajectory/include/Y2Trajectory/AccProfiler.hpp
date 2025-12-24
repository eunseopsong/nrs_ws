/*
    This code is publicly available on GitHub.
    You are free to use, modify, and distribute it.
    However, you must clearly credit the source as follows:

    Source: -
    Author: jaeyun Sim
    Email: wodbs02221@gmail.com
*/

#pragma once

#include "Y2Matrix/YMatrix.hpp"

// Basic Profiler Class
class Profiler {
protected:
    std::vector<double> data;
    double SamplingTime;
    double StartingTime;
    double LastRestingTime;
    double AccelerationTime;

public:
    // Constructor
    Profiler(const std::vector<double>& data, 
             double StartingTime, double LastRestingTime,
             double AccelerationTime, double SamplingTime);

    // Virtual Function
    virtual YMatrix AccDecProfiling() = 0;

    // Utility Function
    void DisplayData(const YMatrix& outputData);
};

// NRS Profiler Class (Corresponding the NRS_acc_dec_profiling)
class YAccProfiler : public Profiler {
public:
    // Constructor
    YAccProfiler(const std::vector<double>& data, 
                double StartingTime, double LastRestingTime, 
                double AccelerationTime, double SamplingTime);

    // NRS Acc-Dec Profiling
    YMatrix AccDecProfiling() override;
};


