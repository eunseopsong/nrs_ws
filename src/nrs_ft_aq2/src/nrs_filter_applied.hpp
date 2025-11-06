#ifndef NRS_FILTER_APPLIED_HPP
#define NRS_FILTER_APPLIED_HPP

#include "nrs_filter_core.hpp"

class NRS_ACC_calculation : public NRS_KalmanFilter
{
public:
    NRS_ACC_calculation();
    ~NRS_ACC_calculation() = default;

    double vel2Acc(double velocity_input);

    double dt = 0.01;
    double vel_pre = 0.0;
};

#endif  // NRS_FILTER_APPLIED_HPP
