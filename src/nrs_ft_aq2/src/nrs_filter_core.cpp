#include "nrs_filter_core.hpp"
#include <numeric>
#include <algorithm>
#include <cmath>

/*** NRS_KalmanFilter ***/

double NRS_KalmanFilter::KalmanFilter(double input)
{
    double P_minus[4];
    double x_minus[2];
    double K_gain[2];
    double temp_help;

    // Prediction
    x_minus[0] = Phi_matrix[0] * x_plus[0] + Phi_matrix[1] * x_plus[1];
    x_minus[1] = Phi_matrix[2] * x_plus[0] + Phi_matrix[3] * x_plus[1];

    P_minus[0] = (Phi_matrix[0]*P_plus[0] + Phi_matrix[1]*P_plus[2]) * Phi_matrix[0];
    P_minus[0] += (Phi_matrix[0]*P_plus[1] + Phi_matrix[1]*P_plus[3]) * Phi_matrix[1];
    P_minus[0] += Q_matrix[0];

    P_minus[1] = (Phi_matrix[0]*P_plus[0] + Phi_matrix[1]*P_plus[2]) * Phi_matrix[2];
    P_minus[1] += (Phi_matrix[0]*P_plus[1] + Phi_matrix[1]*P_plus[3]) * Phi_matrix[3];
    P_minus[1] += Q_matrix[1];

    P_minus[2] = (Phi_matrix[2]*P_plus[0] + Phi_matrix[3]*P_plus[2]) * Phi_matrix[0];
    P_minus[2] += (Phi_matrix[2]*P_plus[1] + Phi_matrix[3]*P_plus[3]) * Phi_matrix[1];
    P_minus[2] += Q_matrix[2];

    P_minus[3] = (Phi_matrix[2]*P_plus[0] + Phi_matrix[3]*P_plus[2]) * Phi_matrix[2];
    P_minus[3] += (Phi_matrix[2]*P_plus[1] + Phi_matrix[3]*P_plus[3]) * Phi_matrix[3];
    P_minus[3] += Q_matrix[3];

    // Gain
    temp_help = (H_matrix[0]*P_minus[0] + H_matrix[1]*P_minus[2]) * H_matrix[0];
    temp_help += (H_matrix[0]*P_minus[1] + H_matrix[1]*P_minus[3]) * H_matrix[1];
    temp_help += R_matrix;

    K_gain[0] = (H_matrix[0]*P_minus[0] + H_matrix[1]*P_minus[1]) / temp_help;
    K_gain[1] = (H_matrix[0]*P_minus[2] + H_matrix[1]*P_minus[3]) / temp_help;

    // Correction
    P_plus[0] = (1.0 - K_gain[0]*H_matrix[0]) * P_minus[0] - K_gain[0]*H_matrix[1] * P_minus[2];
    P_plus[1] = (1.0 - K_gain[0]*H_matrix[0]) * P_minus[1] - K_gain[0]*H_matrix[1] * P_minus[3];
    P_plus[2] = -K_gain[1]*H_matrix[0] * P_minus[0] + (1.0 - K_gain[1]*H_matrix[1]) * P_minus[2];
    P_plus[3] = -K_gain[1]*H_matrix[0] * P_minus[1] + (1.0 - K_gain[1]*H_matrix[1]) * P_minus[3];

    x_plus[0] = x_minus[0] + K_gain[0] * (input - x_minus[0]);
    x_plus[1] = x_minus[1] + K_gain[1] * (input - x_minus[0]);

    return x_plus[0];
}

double NRS_KalmanFilter::KalmanFilter1D(double input)
{
    double x_mi = x_pre;
    double p_mi = p_pre + Q;

    double K = p_mi / (p_mi + R);
    double x = x_mi + K * (input - x_mi);
    double p = (1 - K) * p_mi;

    x_pre = x;
    p_pre = p;

    return x;
}

/*** NRS_MovFilter ***/

NRS_MovFilter::NRS_MovFilter(int mv_num_) : mv_num(mv_num_) {}

float NRS_MovFilter::MovFilter(float input)
{
    if (mv_num > counter)
    {
        saved_data.push_back(input);
        counter++;
    }
    else
    {
        output = std::accumulate(saved_data.begin(), saved_data.end(), 0.0f);
        output /= static_cast<float>(mv_num);

        std::fill(saved_data.begin(), saved_data.end(), 0.0f);
        counter = 0;
    }

    return output;
}

/*** NRS_FreqFilter ***/

NRS_FreqFilter::NRS_FreqFilter(double Ts_) : Ts(Ts_) {}

float NRS_FreqFilter::HPF(float input)
{
    double w0 = 2 * M_PI * HPF_cutF;
    double T  = Ts;
    double Q  = 1.0 / (2.0 * HPF_zeta);

    double a0_ = 4/(T*T) + 2*w0/(Q*T) + (w0*w0);
    double a1_ = -8/(T*T) + 2*(w0*w0);
    double a2_ = 4/(T*T) - 2*w0/(Q*T) + (w0*w0);

    double b0_ = 4/(T*T);
    double b1_ = -8/(T*T);
    double b2_ = 4/(T*T);

    double a1 = a1_ / a0_;
    double a2 = a2_ / a0_;
    double b0 = b0_ / a0_;
    double b1 = b1_ / a0_;
    double b2 = b2_ / a0_;

    double sum0 = -a1 * HFP_timeZone[1] - a2 * HFP_timeZone[0];
    HFP_timeZone[2] = input + sum0;

    double out = b0*HFP_timeZone[2] + b1*HFP_timeZone[1] + b2*HFP_timeZone[0];

    HFP_timeZone[0] = HFP_timeZone[1];
    HFP_timeZone[1] = HFP_timeZone[2];

    return static_cast<float>(out);
}

float NRS_FreqFilter::LPF(float input)
{
    double fs = 1.0 / Ts;
    double w0 = 2.0 * M_PI * LPF_cutF;

    double a1 = (w0 - 2*fs) / (2*fs + w0);
    double b0 = w0 / (2*fs + w0);
    double b1 = b0;

    double out = b0 * input + b1 * PastInput - a1 * PastOutput;
    PastOutput = out;
    PastInput  = input;

    return static_cast<float>(out);
}

float NRS_FreqFilter::BSF(float input)
{
    double w0_peak = 2 * M_PI * BSF_cutF;
    double Q       = BSF_cutF / BSF_BW;

    double b0_ = 4/(Ts*Ts) + (w0_peak*w0_peak);
    double b1_ = -2 * 4/(Ts*Ts) + 2*(w0_peak*w0_peak);
    double b2_ = 4/(Ts*Ts) + (w0_peak*w0_peak);

    double a0_ = 4/(Ts*Ts) + 2*w0_peak/(Q*Ts) + (w0_peak*w0_peak);
    double a1_ = -8/(Ts*Ts) + 2*(w0_peak*w0_peak);
    double a2_ = 4/(Ts*Ts) - 2*w0_peak/(Q*Ts) + (w0_peak*w0_peak);

    double a1 = a1_ / a0_;
    double a2 = a2_ / a0_;
    double b0 = b0_ / a0_;
    double b1 = b1_ / a0_;
    double b2 = b2_ / a0_;

    double sum0 = -a1 * BSF_timeZone[1] - a2 * BSF_timeZone[0];
    BSF_timeZone[2] = input + sum0;

    double out = b0*BSF_timeZone[2] + b1*BSF_timeZone[1] + b2*BSF_timeZone[0];

    BSF_timeZone[0] = BSF_timeZone[1];
    BSF_timeZone[1] = BSF_timeZone[2];

    return static_cast<float>(out);
}
