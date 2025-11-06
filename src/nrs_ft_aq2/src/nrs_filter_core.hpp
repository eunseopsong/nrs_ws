#ifndef NRS_FILTER_CORE_HPP
#define NRS_FILTER_CORE_HPP

#include <vector>

/*** Kalman Filter Class ***/
class NRS_KalmanFilter
{
public:
    /* General Kalman Filter Parameters */

    // Transition matrix: 2x2
    double Phi_matrix[4];
    // Q covariance plant noise matrix: 2x2
    double Q_matrix[4];
    // Sensitivity matrix: 1x2
    double H_matrix[2];
    // Observation noise: R covariance matrix 1x1
    double R_matrix;
    // P plus current covariance matrix 2x2: estimate error
    double P_plus[4];
    // x plus current state vector 2x1: value, speed
    double x_plus[2];

    // 1D Kalman filter parameters
    double x_pre, p_pre;
    double Q, R;

    NRS_KalmanFilter() = default;
    ~NRS_KalmanFilter() = default;

    double KalmanFilter(double input);
    double KalmanFilter1D(double input);
};


/*** Moving Average Filter Class ***/
class NRS_MovFilter
{
public:
    int mv_num = 0;
    int counter = 0;
    std::vector<float> saved_data;

    NRS_MovFilter(int mv_num_);
    NRS_MovFilter() : NRS_MovFilter(0) {}
    ~NRS_MovFilter() = default;

    float MovFilter(float input);

private:
    float output = 0.0f;
};


/*** Frequency Filter Class ***/
class NRS_FreqFilter
{
public:
    // Common
    double Ts = 0.0;

    // High Pass Filter
    double HFP_timeZone[3] = {0.0, 0.0, 0.0};
    double HPF_cutF = 5;     // Hz
    double HPF_zeta = 0.7;   // damping ratio

    // Low Pass Filter
    double PastInput = 0.0;
    double PastOutput = 0.0;
    double LPF_cutF = 5;     // Hz

    // Band Stop Filter
    double BSF_timeZone[3] = {0.0, 0.0, 0.0};
    double BSF_cutF = 5;     // Hz
    double BSF_BW   = 5;     // Hz

    NRS_FreqFilter(double Ts_);
    NRS_FreqFilter() : NRS_FreqFilter(0.0) {}
    ~NRS_FreqFilter() = default;

    float HPF(float input);
    float LPF(float input);
    float BSF(float input);
};

#endif  // NRS_FILTER_CORE_HPP
