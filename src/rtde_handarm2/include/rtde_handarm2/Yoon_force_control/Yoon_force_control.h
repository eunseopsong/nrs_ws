#include <stdio.h>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include "Kinematics.h"

#define PI 3.141592
using namespace Eigen;


class Yoon_force_control
{
    private:
         /* For 1D_admittance control parameters */
        double adm_1D_Perror[3] = {0,}; // Position error
        double adm_1D_Ferror[3] = {0,}; // Force error
        double adm_1D_M = 1;
        double adm_1D_D = 1;
        double adm_1D_K = 0;
        double adm_1D_A,adm_1D_B,adm_1D_C;
        double adm_1D_dt;
        double xc; // output of admittance control
    
    public:
        Yoon_force_control() {}
        ~Yoon_force_control() {}
      
        bool adm_1D_MDK(double Mass, double Damping, double Stiffness);
        bool adm_1D_init(double Perror_init,double Ferror_init,double Sampling_time);
        double adm_1D_control(double xd, double Fd, double Fext);

};

typedef class E_tank_Hand_Guiding
{
    
    private:

        double dampE_sum = 0; // damping energy summation
        double massCE_sum = 0; // mass change energy summation
        double whi; // To prevent overflow of energy
        double gamma;
    
    public:
        E_tank_Hand_Guiding() {}
        ~E_tank_Hand_Guiding() {}

        double init_energy = 2; // initial energy (unit : J)
        double tank_Ulim = 5; // tank upper limit (unit : J)
        double tank_Llim = 0.1; // tank lower limit (unit : J)

        double init_Md;
        double init_Dd;

        double tank_energy; // energy stored in tank (unit : J)
    
        // should be updated at the every loop
        double Md_dot; // admittance control mass derivative
        double Dd; // admittance control damping
        double current_vel; // current vel (m/s or rad/s) 

        double Energy_tank();
     
} Etank_HG;

struct DS_power_PB_RTinput
{
    // Class DS_power_playback's Real time inputs
    /* Position */
    double PFd = 0;
    Eigen::Vector3d PXr,PX,PFext,PRtz,PRtx;
    Eigen::Matrix3d P_Tool_Rot;
    /* Orientation */
    double OFd = 0;
    Eigen::Vector3d OXr,OX,OFext,ORtz,ORtx;
};
class DS_power_playback // Dynamical system based force control
{
    // Real-time inputs : 
    // [position]
    // PXr,PX,PFd,PFext
    // [Orientation]
    // OXr,OX,OFd,OFext

    // Real-time outputs : 
    // PXc_0, OXc_0
    private:

    public:
    /* Parameters start */

    /* STEP1) Task generation */
    
    /* Position paramters */
    Eigen::Vector3d PXr,PXr_pre,PX,PX_pre,PXr_dot,PX_dot,PXd_dot,Pfx,Pfr,Pfn,PZ_ori,PU1,PU1_pre,PU2,PU3,PU3_pre,PU1_rot_axis,PRtz,PRtx; 
    Eigen::Vector3d PFd = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d PFd_hat = Eigen::Vector3d::Zero(3);
    Eigen::MatrixXd P_Tool_Rot = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd PKd = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd PU = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd PRamK_mrx = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd PRamM_mrx = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd PU1_rot_mrx = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd PU3_rot_mrx = Eigen::MatrixXd::Zero(3,3);
    double PRamK[3] = {1,2,2};
    double PRamD[3] = {1,1,1};
    double PRamM[3] = {1,1,1};
    /* Orientation paramters */
    Eigen::Vector3d OXr,OXr_pre,OX,OX_pre,OXr_dot,OX_dot,OXd_dot,Ofx,Ofr,Ofn,OZ_ori,OU1,OU2,OU3,OU1_rot_axis,ORtz;
    Eigen::Vector3d OFd = Eigen::Vector3d::Zero(3); 
    Eigen::MatrixXd OKd = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd OU = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd ORamK_mrx = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd ORamM_mrx = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd OU1_rot_mrx = Eigen::MatrixXd::Zero(3,3);
    double ORamK[3] = {1,2,2};
    double ORamD[3] = {1,1,1};
    double ORamM[3] = {1,1,1};

    AKfun AA_ref; //Kinematics function class instance


    /* STEP2) Calculate state-varying damping matrix */

    /* Position paramters */
    Vector3d Pe1,Pe1_pre,Pe2,Pe3,Pe3_pre,Pe1_rot_axis;
    Eigen::MatrixXd PRamD_mrx = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd PDd = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd Pe1_rot_mrx = Eigen::MatrixXd::Zero(3,3);
    /* Orientation paramters */
    Vector3d Oe1,Oe2,Oe3,Oe1_rot_axis;
    Eigen::MatrixXd OQ = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd ORamD_mrx = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd ODd = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd Oe1_rot_mrx = Eigen::MatrixXd::Zero(3,3);


    /* STEP3) Calculate energy tank */

    /* Position paramters */
    double PTankE = 0; 
    double PTiniE = 2;// Initial energy 2J, Unit: J 
    double PTankMin = 0.1; // Min. energy 0.1J, Unit: J 
    double PTankMax = 3; // Max. energy 5J, Unit: J 
    double Ppr_integ = 0;
    double Ppn_integ = 0;
    double Ppd_integ = 0;
    double Ppr, Ppn, Ppd;
    double PTAl, PTBr, PTBn;

    /* Orientation paramters */
    double OTankE = 0; 
    double OTiniE = 2;// Initial energy 2J, Unit: J 
    double OTankMin = 0.1; // Min. energy 0.1J, Unit: J 
    double OTankMax = 5; // Max. energy 5J, Unit: J 
    double Opr_integ = 0;
    double Opn_integ = 0;
    double Opd_integ = 0;
    double Opr, Opn, Opd;
    double OTAl, OTBr, OTBn;

    
    /* STEP4) Reshape the task */

    /* Position paramters */
    double PTBr_dat, PTBn_dat;
    /* Orientation paramters */
    double OTBr_dat, OTBn_dat;


    /* STEP5) Solve the differential equation with tustin method */

    double Ts = 0.002; // Sampling time
    /* Position paramters */
    Vector3d PXc_0,PXc_1,PXc_2,PA_0,PA_1,PA_2,PFext; // initialization required
    Matrix3d Pdif_U,Pdif_V,Pdif_W,PMd;
    /* Orientation paramters */
    Vector3d OXc_0,OXc_1,OXc_2,OA_0,OA_1,OA_2,OFext; // initialization required
    Matrix3d Odif_U,Odif_V,Odif_W,OMd;

    /* Residure parameters */
    bool PB_init_flag = false;
    double Fext_threshold = 1.5; // (Unit: N)
    double U13_angle_threshold = 5.0; // (Unit: degree)
    Eigen::Vector3d VpreU1, VrotU3, Vapproach,Vapproach_pre;
    /* Parameters end */

    /* Functions start */
    void playback_init(Eigen::Vector3d Current_pos, Eigen::Vector3d Current_ori);
    void PU_calculation();
    int playback_start(DS_power_PB_RTinput RTinput);

};

class Fuzzy_adaptive_k
{
    public:
    Fuzzy_adaptive_k(const char Exper_Env[]);

    int FAAC_array_size[2] = {12,12}; // size - 1
    double FAAC_DelEtha_array[13][13] = {
        {-5.37231, -5.25268, -5.25268, -5.37231, -5.25268, -3, -3, -3, -3, -3, -3, 1.15463e-16, 6.65601e-18},
        {-5.25268, -5.25268, -5.25268, -5.25268, -5.25268, -3, -3, -3, -3, -3, -3, 1.15463e-16, 1.15463e-16},
        {-5.25268, -5.25268, -3, -3, -3, -3, -3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16},
        {-5.37231, -5.25268, -3, -3, -3, -3, -3, -3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 1.15463e-16, 6.65601e-18},
        {-5.25268, -5.25268, -3, -3, -3, -3, -3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16},
        {-3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3},
        {-3, -3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 1.15463e-16, 6.65601e-18, 1.15463e-16, 1.15463e-16, 6.65601e-18, 1.15463e-16, 3, 3},
        {-3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3},
        {-3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3, 3, 3, 3, 3, 5.25268, 5.25268},
        {-3, -3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 3, 3, 3, 3, 3, 3, 5.25268, 5.37231},
        {-3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3, 3, 3, 3, 3, 5.25268, 5.25268},
        {1.15463e-16, 1.15463e-16, 3, 3, 3, 3, 3, 3, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268},
        {6.65601e-18, 1.15463e-16, 3, 3, 3, 3, 3, 3, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231},
    };
    double FAAC_DelK_array[13][13] = {
        {5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 3, 3, 3, 3, 3, 3, 1.15463e-16, 6.65601e-18},
        {5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 3, 3, 3, 3, 3, 3, 1.15463e-16, 1.15463e-16},
        {5.25268, 5.25268, 3, 3, 3, 3, 3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16},
        {5.37231, 5.25268, 3, 3, 3, 3, 3, 3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 1.15463e-16, 6.65601e-18},
        {5.25268, 5.25268, 3, 3, 3, 3, 3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16},
        {3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, -3, -3},
        {3, 3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 1.15463e-16, 6.65601e-18, 1.15463e-16, 1.15463e-16, 6.65601e-18, 1.15463e-16, -3, -3},
        {3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, 1.15463e-16, -3, -3},
        {3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, -3, -3, -3, -3, -3, -3, -5.25268, -5.25268},
        {3, 3, 1.15463e-16, 6.65601e-18, 1.15463e-16, -3, -3, -3, -3, -3, -3, -5.25268, -5.37231},
        {3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, -3, -3, -3, -3, -3, -3, -5.25268, -5.25268},
        {1.15463e-16, 1.15463e-16, -3, -3, -3, -3, -3, -3, -5.25268, -5.25268, -5.25268, -5.25268, -5.25268},
        {6.65601e-18, 1.15463e-16, -3, -3, -3, -3, -3, -3, -5.25268, -5.37231, -5.25268, -5.25268, -5.37231},
    };

    double FAAC_DelK_Cal(double F_Err, double FDot_Err);
    double FAAC_DelSigma_Cal(double F_Err, double FDot_Err);
    double FAAC_Normalize(double Input,double Input_Min,double Input_Max,double FAAC_Min,double FAAC_Max);

    private:
    double FAAC_MinMax = 6;
    double FEDMM,FEMM,DKMM,DEMM;
    double FAAC_array_interval = 1;
};

class Fuzzy_adaptive_md
{
    /* This controller is constructed by yoon */
    public:

    int FAAC_array_size[2] = {12,12}; // size - 1

    double FAAC_DelM_array[13][13] = {{5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 1.15463e-16, 6.65601e-18},
    {5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 1.15463e-16, 1.15463e-16},
    {3, 3, 3, 3, 3, 3, 3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, -5.25268, -5.25268},
    {3, 3, 3, 3, 3, 3, 3, 3, 1.15463e-16, 6.65601e-18, 1.15463e-16, -5.25268, -5.37231},
    {3, 3, 3, 3, 3, 3, 3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, -5.25268, -5.25268},
    {-5.25268, -5.25268, -3, -3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, -3, -3, -3, -5.25268, -5.25268},
    {-5.37231, -5.25268, -3, -3, -3, 1.15463e-16, 6.65601e-18, 1.15463e-16, -3, -3, -3, -5.25268, -5.37231},
    {-5.25268, -5.25268, -3, -3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, -3, -3, -3, -5.25268, -5.25268},
    {-5.25268, -5.25268, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3, 3, 3, 3, 3, 3, 3},
    {-5.37231, -5.25268, 1.15463e-16, 6.65601e-18, 1.15463e-16, 3, 3, 3, 3, 3, 3, 3, 3},
    {-5.25268, -5.25268, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3, 3, 3, 3, 3, 3, 3},
    {1.15463e-16, 1.15463e-16, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268},
    {6.65601e-18, 1.15463e-16, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231},
    };

    double FAAC_DelMDRatio_array[13][13] = {{5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231},
    {5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268},
    {3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3},
    {3, 3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 3, 3, 3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 3, 3},
    {3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3},
    {-5.25268, -5.25268, -3, -3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, -3, -3, -3, -5.25268, -5.25268},
    {-5.37231, -5.25268, -3, -3, -3, 1.15463e-16, 6.65601e-18, 1.15463e-16, -3, -3, -3, -5.25268, -5.37231},
    {-5.25268, -5.25268, -3, -3, -3, 1.15463e-16, 1.15463e-16, 1.15463e-16, -3, -3, -3, -5.25268, -5.25268},
    {3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3},
    {3, 3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 3, 3, 3, 1.15463e-16, 6.65601e-18, 1.15463e-16, 3, 3},
    {3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3, 3, 1.15463e-16, 1.15463e-16, 1.15463e-16, 3, 3},
    {5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268, 5.25268},
    {5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231, 5.25268, 5.25268, 5.37231},
    };



    Fuzzy_adaptive_md(const char Exper_Env[], double Init_Md, double Init_Dd, double Init_Kd, double Ts, double HPF_cf, double MdDdR_threshold);
    double FAAC_DelM_Cal(double F_Err, double FDot_Err);
    double FAAC_DelMDRatio_Cal(double F_Err, double F_H);
    double FAAC_HPFInput_Cal(double F_ext, double HPF_cf, double HPF_Ts, double MdDdR_threshold);
    double FAAC_Normalize(double Input,double Input_Min,double Input_Max,double FAAC_Min,double FAAC_Max);

    void FAAC_Kd_variation(double Fd, double Curr_Kd, Eigen::Vector3d &Curr_posi, double* Output_Kd);
    void FAAC_MD_MainCal(double Fd, double F_ext, double* Cal_Md, double* Cal_Dd); 

    private:
    double FAAC_Md_Limit[2] = {0,};
    double FAAC_MDR_Limit[2] = {0,};
    double FAAC_MinMax = 6;
    double FEDMM,FEMM,HPFFMM,DMMM,DMDRatio_MM;
    double FAAC_array_interval = 1;
    double HPF_Fext_in[2] = {0, 0};
    double HPF_Fext_out[2] = {0, 0};
    double Init_MDR, Updated_Md, Updated_MDR;
    double FAAC_Fe, FAAC_Fe_dot, FAAC_Ts, FAAC_HPF_cf, FAAC_MdDdR_threshold;
    double FAAC_Fe_pre = 0;
    double FAAC_Kd, F_ExtThr, Kd_recov_dist, Init_Kd_;
    Eigen::Vector3d Zh0,Zm;
};