#include "rtde_handarm2/ForceControl/admittance_control.hpp"

Yadmittance_control::Yadmittance_control(double sampling_time)
{
    adm_1D_dt = sampling_time;
}

bool Yadmittance_control::adm_1D_MDK(double Mass, double Damping, double Stiffness)
{
    adm_1D_M = Mass;
    adm_1D_D = Damping;
    adm_1D_K = Stiffness;
    return true;
}

double Yadmittance_control::adm_MDK_monitor(int select)
{
    if (select == 0) return adm_1D_M;
    if (select == 1) return adm_1D_D;
    if (select == 2) return adm_1D_K;
    return 0.0;
}

/* Semi-implicit Euler 2차 상태공간 근사 */
double Yadmittance_control::adm_1D_control(double xd, double Fd, double Fext)
{
    const double dt = (adm_1D_dt > 0.0) ? adm_1D_dt : 0.01;

    // 이전 오차 시프트
    adm_1D_Perror[2] = adm_1D_Perror[1];
    adm_1D_Perror[1] = adm_1D_Perror[0];
    adm_1D_Ferror[2] = adm_1D_Ferror[1];
    adm_1D_Ferror[1] = adm_1D_Ferror[0];

    // 현재 힘 오차
    adm_1D_Ferror[0] = Fd - Fext;

    // 단자유도 admittance: M xdd + D xd + K (x - xd) = Fd - Fext
    // 상태( x, v )에 대해 반-암시 오일러
    const double rhs = (Fd - Fext) - adm_1D_D * v - adm_1D_K * (x - xd);
    const double a   = rhs / adm_1D_M;        // 가속도
    v += a * dt;                               // 속도 적분
    x += v * dt;                               // 위치 적분

    xc = x;    // 내부 상태 보관(옵션)
    adm_1D_Perror[0] = xd - x;

    return x;
}
