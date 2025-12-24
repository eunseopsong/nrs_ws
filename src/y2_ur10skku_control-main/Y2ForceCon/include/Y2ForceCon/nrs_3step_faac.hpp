#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include "Y2ForceCon/nrs_3stepfaac_map.hpp"


template <typename T>
T clamp(T value, T low, T high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

class SimpleKalmanFilter {
    public:
        SimpleKalmanFilter(double dt, std::vector<double>& process_noise, std::vector<double>& measurement_noise);
        ~SimpleKalmanFilter() {}
    
        void predict();  // 예측 단계
        void update(double z);  // 업데이트 단계 (위치만 사용)

        double getAcceleration() { return a; } // 가속도 추정값 반환
        double getPosition() { return x; } // 위치 추정값 반환
        double getVelocity() { return v; } // 속도 추정값 반환
    
    private:
        double dt;  // 시간 간격
        double x;   // 위치 추정값
        double v;   // 속도 추정값
        double a;   // 가속도 추정값
        double z_v; // 측정값 (속도)
        double z_a; // 측정값 (가속도)
        double z_xpre, z_vpre;
        double P_x,P_v,P_a;   // 공분산 (1x1)
        double Q_x,Q_v,Q_a;   // 프로세스 노이즈 공분산 (스칼라)
        double R_x,R_v,R_a;   // 측정 노이즈 공분산 (스칼라)
};

struct Nrs3StepFAAC_MDK
{
    double Mass; // Mass
    double Damping; // Damping
    double Stiffness; // Stiffness
};

class Nrs3StepFAAC
{
    public:
        Nrs3StepFAAC(double _dt, std::vector<double> _process_noise, std::vector<double> _measurement_noise);
        ~Nrs3StepFAAC() {}

        SimpleKalmanFilter kf; // 칼만 필터 객체
        nrs_3stepfaac_map map; // FAAC maps
        double dt = 0.002;
        double Updated_MDR = 1.0; // Updated MDR
        double Init_md,Init_dd,Init_kd, Init_MDR; // Initial Mass, Damping, Stiffness, MassDamper Ratio
        double Updated_md,Updated_dd,Updated_kd; // Updated Mass, Damping, Stiffness
        double FEDMM, FEMM, EpsilonMM, XcMSTDMM; // Input Normalization limitations
        double DMMM, DMDRatio_MM, DMdMM; // Output Normalization limitations
        double FAAC_MinMax; // Fuzzy Min Max for Normalization
        std::vector<double> FAAC_Md_Limit,FAAC_MDR_Limit,FAAC_MdMin_limit; // Desired Md, MDR, Md Min. Limit (Very important)
        std::vector<double> STD_window; // Window for STD calculation
        std::vector<double> Tank_MM; // Tank Min Max (for MDK recovery)
        unsigned int window_size; // Window size for STD calculation
        double Pre_Ferr = 0; // Force Error, Force Error Dot
        bool contact_flag = false; // Contact flag (To monitoring the contact status)
        double tau_basic, tau_max; // Basic,Max. recovery time (for MDK recovery)
        double gamma_d; // Weight for dampling ratio effect (for MDK recovery)
        double Zeta_min; // zeta min. at MDK recovery
        int FAAC_array_interval; // Interval for FAAC array
        double contact_threshold; // Contact threshold used for contact detection (Unit: N)
        double FAAC_Ferr,FAAC_FDot_Err,FAAC_Epsilon,FAAC_XeSTD; // Fuzzy inputs
        
        void FAAC_Init(const double Init_md_, const double Init_dd_, const double Init_kd_);
        double FAAC_Normalize(double Input,double Input_Min,double Input_Max,double FAAC_Min,double FAAC_Max);
        double FAAC_Epsilon_Calc(double xc, double fext, double fd, double md, double dd);
        double FAAC_XeSTD_Calc(const double Xe);

        double FAAC_DelM_Calc(double F_Err, double FDot_Err);
        double FAAC_DelMDRatio_Calc(double F_Err, double FDot_Err);
        double FAAC_DMdM_Calc(double epsilon, double Xc_X_std);
        void FAAC_MDKTransi_Calc(const double Tank_energy);
        
        Nrs3StepFAAC_MDK FAAC_MDKob_RUN(
            const double Tank_Energy, const double F_ext, const double Fd, const double Xc, const double X);

    private:
};