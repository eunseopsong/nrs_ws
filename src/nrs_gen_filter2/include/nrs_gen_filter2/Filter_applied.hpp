#ifndef FILTER_APPLIED_HPP
#define FILTER_APPLIED_HPP

#include "Gen_filter.hpp"

class NRS_ACC_calculation : public NRS_KalmanFilter
{
    public:
        /* Functions */
        NRS_ACC_calculation(); // Default constructor
        ~NRS_ACC_calculation() {};

        double vel2Acc(double velocity_input);

        /* Parameters used in vel2Acc function */
        double dt = 0.01;
        double vel_pre = 0; 

    private:
};

#endif  // FILTER_APPLIED_HPP