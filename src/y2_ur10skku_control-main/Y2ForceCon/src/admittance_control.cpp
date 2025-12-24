#include "Y2ForceCon/admittance_control.hpp"
#include <cmath>


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
    /* 0:M, 1:D, 2:K */
    if(select == 0){return adm_1D_M;}
    else if(select == 1){return adm_1D_D;}
    else if(select == 2){return adm_1D_K;}
    else{printf("Wrong MDK monitor was selected \n");}
}

double Yadmittance_control::adm_1D_control(double xd, double Fd, double Fext)
{
    adm_1D_Ferror[0] = Fd - Fext;

    adm_1D_A = 4*adm_1D_M - 2*adm_1D_dt*adm_1D_D + pow(adm_1D_dt,2)*adm_1D_K;
    adm_1D_B = 2*pow(adm_1D_dt,2)*adm_1D_K - 8*adm_1D_M;
    adm_1D_C = 4*adm_1D_M + 2*adm_1D_dt*adm_1D_D + pow(adm_1D_dt,2)*adm_1D_K;

    xc = xd + (1/adm_1D_C)*(adm_1D_B*adm_1D_Perror[1] + adm_1D_A*adm_1D_Perror[2]
        - pow(adm_1D_dt,2)*(adm_1D_Ferror[0] + 2*adm_1D_Ferror[1] + adm_1D_Ferror[2]));

    adm_1D_Perror[0] = xd - xc;
    adm_1D_Perror[2] = adm_1D_Perror[1];
    adm_1D_Perror[1] = adm_1D_Perror[0];

    adm_1D_Ferror[2] = adm_1D_Ferror[1];
    adm_1D_Ferror[1] = adm_1D_Ferror[0];

    return xc;
}