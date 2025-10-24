#pragma once
#include <array>
#include <cmath>

class Yadmittance_control {
public:
    explicit Yadmittance_control(double sampling_time);

    bool   adm_1D_MDK(double Mass, double Damping, double Stiffness);
    double adm_MDK_monitor(int select);   // 0:M, 1:D, 2:K
    double adm_1D_control(double xd, double Fd, double Fext);

private:
    // params
    double adm_1D_dt{0.002};
    double adm_1D_M{1.0}, adm_1D_D{10.0}, adm_1D_K{0.0};

    // discrete internal states
    double xc{0.0};     // last commanded position (internal)
    double x{0.0};      // last output position
    double v{0.0};      // last output velocity

    std::array<double,3> adm_1D_Perror{0.0,0.0,0.0};
    std::array<double,3> adm_1D_Ferror{0.0,0.0,0.0};
};
