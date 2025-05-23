#include "Yoon_filters.h"

float Yoon_filters::KalmanFilter(float input)
{
    float P_minus[4]; /* matrix 2x2 */
    float x_minus[2]; /* vector 2x1 */
    float K_gain[2];  /* matrix 2x1 */
    float temp_help;
    
    /* Prediction Step */
    x_minus[0] = this->KF_par.Phi_matrix[0]*this->KF_par.x_plus[0] + this->KF_par.Phi_matrix[1]*this->KF_par.x_plus[1];
    x_minus[1] = this->KF_par.Phi_matrix[2]*this->KF_par.x_plus[0] + this->KF_par.Phi_matrix[3]*this->KF_par.x_plus[1];
    P_minus[0] = (this->KF_par.Phi_matrix[0]*this->KF_par.P_plus[0] + this->KF_par.Phi_matrix[1]*this->KF_par.P_plus[2])*this->KF_par.Phi_matrix[0];
    P_minus[0] += (this->KF_par.Phi_matrix[0]*this->KF_par.P_plus[1] + this->KF_par.Phi_matrix[1]*this->KF_par.P_plus[3])*this->KF_par.Phi_matrix[1];
    P_minus[0] += this->KF_par.Q_matrix[0];
    P_minus[1] = (this->KF_par.Phi_matrix[0]*this->KF_par.P_plus[0] + this->KF_par.Phi_matrix[1]*this->KF_par.P_plus[2])*this->KF_par.Phi_matrix[2];
    P_minus[1] += (this->KF_par.Phi_matrix[0]*this->KF_par.P_plus[1] + this->KF_par.Phi_matrix[1]*this->KF_par.P_plus[3])*this->KF_par.Phi_matrix[3];
    P_minus[1] += this->KF_par.Q_matrix[1];
    P_minus[2] = (this->KF_par.Phi_matrix[2]*this->KF_par.P_plus[0] + this->KF_par.Phi_matrix[3]*this->KF_par.P_plus[2])*this->KF_par.Phi_matrix[0];
    P_minus[2] += (this->KF_par.Phi_matrix[2]*this->KF_par.P_plus[1] + this->KF_par.Phi_matrix[3]*this->KF_par.P_plus[3])*this->KF_par.Phi_matrix[1];
    P_minus[2] += this->KF_par.Q_matrix[2];
    P_minus[3] = (this->KF_par.Phi_matrix[2]*this->KF_par.P_plus[0] + this->KF_par.Phi_matrix[3]*this->KF_par.P_plus[2])*this->KF_par.Phi_matrix[2];
    P_minus[3] += (this->KF_par.Phi_matrix[2]*this->KF_par.P_plus[1] + this->KF_par.Phi_matrix[3]*this->KF_par.P_plus[3])*this->KF_par.Phi_matrix[3];
    P_minus[3] += this->KF_par.Q_matrix[3];
    /* Kalman Gain */
    temp_help = (this->KF_par.H_matrix[0]*P_minus[0] + this->KF_par.H_matrix[1]*P_minus[2])*this->KF_par.H_matrix[0];
    temp_help += (this->KF_par.H_matrix[0]*P_minus[1] + this->KF_par.H_matrix[1]*P_minus[3])*this->KF_par.H_matrix[1];
    temp_help += this->KF_par.R_matrix;
    K_gain[0] = (this->KF_par.H_matrix[0]*P_minus[0] + this->KF_par.H_matrix[1]*P_minus[1])/temp_help; /* temp_help shall be !=0 */
    K_gain[1] = (this->KF_par.H_matrix[0]*P_minus[2] + this->KF_par.H_matrix[1]*P_minus[3])/temp_help;
    /* Correction Step */
    this->KF_par.P_plus[0] = (1.0 - K_gain[0]*this->KF_par.H_matrix[0])*P_minus[0] - K_gain[0]*this->KF_par.H_matrix[1]*P_minus[2];
    this->KF_par.P_plus[1] = (1.0 - K_gain[0]*this->KF_par.H_matrix[0])*P_minus[1] - K_gain[0]*this->KF_par.H_matrix[1]*P_minus[3];
    this->KF_par.P_plus[2] = -K_gain[1]*this->KF_par.H_matrix[0]*P_minus[0] + (1.0 - K_gain[1]*this->KF_par.H_matrix[1])*P_minus[2];
    this->KF_par.P_plus[3] = -K_gain[1]*this->KF_par.H_matrix[0]*P_minus[1] + (1.0 - K_gain[1]*this->KF_par.H_matrix[1])*P_minus[3];
    this->KF_par.x_plus[0] = x_minus[0] + K_gain[0]*(input - x_minus[0]);
    this->KF_par.x_plus[1] = x_minus[1] + K_gain[1]*(input - x_minus[0]);
    
    return this->KF_par.x_plus[0];
}

float Yoon_filters::KalmanFilter1D(float input)
{
    float x_mi, p_mi, K, x, p;

    x_mi = this->KF1D_par.x_pre;
    p_mi = this->KF1D_par.p_pre + this->KF1D_par.Q;
    
    K = p_mi/(p_mi+this->KF1D_par.R);
    x = x_mi + K*(input-x_mi);
    p = (1-K)*p_mi;
    
    this->KF1D_par.x_pre = x;
    this->KF1D_par.p_pre = p;

    return x;
}

float Yoon_filters::MovingAvgFilter(float input)
{
    float output = 0;

    if(this->MV_par.mv_num <= this->MV_par.counter)
    {
        this->MV_par.counter = 0;
    }

    if(this->MV_par.mv_num > this->MV_par.counter)
    {
        this->MV_par.saved_data[this->MV_par.counter] = input;
        this->MV_par.counter++;

        for(int i = 0;i<this->MV_par.mv_num;i++)
        {
            output += this->MV_par.saved_data[i];
        }
        output /= (float)this->MV_par.mv_num;
    }

    return output;
}

float Yoon_filters::HighPassFilter(float input)
{
    double HPF_out = 0;

    double w0, T, Q;
    double a0_, a1_, a2_, b0_, b1_, b2_;
    double a1, a2, b0, b1, b2;
    double H0 = 1;
    double sum0;
    
    w0 = 2*(3.141592)*this->HPF_par.f_cut;
    T = this->HPF_par.ts; // sampling time
    Q = 1/(2*this->HPF_par.zeta);
    
    a0_ = 4/(T*T) + 2*w0/(Q*T) + (w0*w0);
    a1_ = -8/(T*T) + 2*(w0*w0);
    a2_ = 4/(T*T) - 2*w0/(Q*T) + (w0*w0);
    
    b0_ = 4*H0/(T*T);
    b1_ = -8*H0/(T*T);
    b2_ = 4*H0/(T*T);
    
    a1 = a1_/a0_;
    a2 = a2_/a0_;
    b0 = b0_/a0_;
    b1 = b1_/a0_;
    b2 = b2_/a0_;
    
    
    sum0 = -a1*this->HPF_par.timeZone[1] - a2*this->HPF_par.timeZone[0];
    this->HPF_par.timeZone[2] = input + sum0;
    HPF_out = b0*this->HPF_par.timeZone[2] + b1*this->HPF_par.timeZone[1] + b2*this->HPF_par.timeZone[0];

    this->HPF_par.timeZone[0] = this->HPF_par.timeZone[1];
    this->HPF_par.timeZone[1] = this->HPF_par.timeZone[2];

    return HPF_out;
}

float Yoon_filters::LowPassFilter(float input)
{
    double a1,b0,b1,w0;
    double LPF_out;
    w0 = 2*3.14*this->LPF_par.CutOffFrequency;
    a1 = (w0 - 2*this->LPF_par.SamplingFrequency)/(2*this->LPF_par.SamplingFrequency + w0);
    b0 = w0/(2*this->LPF_par.SamplingFrequency + w0);
    b1 = b0;

    LPF_out = b0*(input) + b1*(this->LPF_par.PastInput) - a1*(this->LPF_par.PastOutput);
    this->LPF_par.PastOutput = LPF_out;
    this->LPF_par.PastInput = input;

    return LPF_out;
}

float Yoon_filters::BandStopFilter(float input)
{
    double BSF_out = 0;

    double w0_peak, Ts, Q;
    double a0_, a1_, a2_, b0_, b1_, b2_;
    double a1, a2, b0, b1, b2;
    double H0 = 1;
    double sum0;

    Ts =  this->BSF_par.ts;//
    w0_peak = 2*(3.141592)*this->BSF_par.f_peak;//
    Q = this->BSF_par.f_peak/this->BSF_par.bandWidth;//
    
    b0_ = H0*4/(Ts*Ts) + H0*(w0_peak*w0_peak);
    b1_ = -2*H0*4/(Ts*Ts) + 2*(w0_peak*w0_peak);
    b2_ = H0*4/(Ts*Ts) + H0*(w0_peak*w0_peak);
    
    a0_ = 4/(Ts*Ts)+2*w0_peak/(Q*Ts)+(w0_peak*w0_peak);
    a1_ = -8/(Ts*Ts)+2*(w0_peak*w0_peak);
    a2_ = 4/(Ts*Ts)-2*w0_peak/(Q*Ts)+(w0_peak*w0_peak);
    
    a1 = a1_/a0_;
    a2 = a2_/a0_;
    b0 = b0_/a0_;
    b1 = b1_/a0_;
    b2 = b2_/a0_;
    
    
    sum0 = -a1*this->BSF_par.timeZone[1] - a2*this->BSF_par.timeZone[0];
    this->BSF_par.timeZone[2] = input + sum0;
    BSF_out = b0*this->BSF_par.timeZone[2] + b1*this->BSF_par.timeZone[1] + b2*this->BSF_par.timeZone[0];

    this->BSF_par.timeZone[0] = this->BSF_par.timeZone[1];
    this->BSF_par.timeZone[1] = this->BSF_par.timeZone[2];

    return BSF_out;
}