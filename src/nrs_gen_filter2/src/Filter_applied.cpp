#include "nrs_gen_filter2/Filter_applied.hpp" //// #include "Filter_applied.hpp"

NRS_ACC_calculation::NRS_ACC_calculation() 
{
    x_pre = 0;
    p_pre = 0;
}

double NRS_ACC_calculation::vel2Acc(double velocity_input)
{
    double cal_acc;

    cal_acc = (velocity_input - vel_pre) / dt;

    cal_acc = KalmanFilter1D(cal_acc);
    vel_pre = velocity_input;

    return cal_acc;
}