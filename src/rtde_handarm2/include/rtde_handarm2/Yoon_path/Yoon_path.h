#include<stdio.h>
#include<Eigen/Dense>
#include<iostream>


typedef Eigen::VectorXd Dynamic1D;
typedef Eigen::MatrixXd DynamicMatrix;

class Yoon_path
{
    private:

        /* Paramerters for Single_blended_path start*/
        Dynamic1D time_profile;
        DynamicMatrix Interpolated, Final_vel;
        double Travel_time;
        int Y_array_size;
        /* Paramerters for Single_blended_path end*/


    public:

        /* Single_blended_path start */
        DynamicMatrix Final_pos, Final_force;
        double Ts = 0.002; // Sampling time
        double Acc_time = 0.5; // Target Acceleration time
        double Tm = 1.5; //Time margin at the start and end point, recommended twice of Acc_time

        // EX) Relative motion from zero
        // double Tar_pos[] = {100,50}; // Target position array, unit: mm
        // double Tar_vel[] = {10,20}; // Target velocity array, unit: mm/s
        // double Waiting_time[] = {0,0}; // Waiting time, unit : s, 0이 아닌 값을 넣으려면 동일 array 위치의 pos와 vel은 0 

        int Single_blended_path(double Tar_pos[], double Tar_vel[], double Waiting_time[],int Path_length);
        int Single_blended_force(double Exe_time[], double Tar_force[], double Waiting_time[],int Path_length);
        /* Single_blended_path end */


        /* PTP_6D_path start */
        DynamicMatrix Final_px,Final_py,Final_pz,Final_rx,Final_ry,Final_rz;
        bool PTP_6D_path_init_flag = false;
        uint64_t PTP_6D_path_counter = 0;
        int generated_path_num[6] = {-1,-1,-1,-1,-1,-1};
        double starting_pos[6] = {0,};

        bool PTP_6D_path_init(double Init_pos[6], double Tar_pos[6], double travel_time);
        bool PTP_6D_path_exe(double path_out[6]);
        /* PTP_6D_path end */


        /* MultiP_6D_path start */
        DynamicMatrix MFinal_px,MFinal_py,MFinal_pz,MFinal_rx,MFinal_ry,MFinal_rz;
        bool MPTP_6D_path_init_flag = false;
        uint64_t MPTP_6D_path_counter = 0;
        int Mgenerated_path_num[6] = {-1,-1,-1,-1,-1,-1};
        double Mstarting_pos[6] = {0,};
        bool MultiP_6D_path_init(Eigen::MatrixXd Tar_point, double _Tar_vel[], double Waiting_time[],int Point_num);
        bool MultiP_path_exe(double path_out[6]);
        /* MultiP_6D_path end */

        /* PPB_path start */
        // For power(Position + force) playback trajectory
        // It contains posture motion & force blending
        DynamicMatrix PPBFinal_px,PPBFinal_py,PPBFinal_pz,PPBFinal_rx,PPBFinal_ry,PPBFinal_rz,PPBFinal_Force;
        bool PPB_path_init_flag = false;
        uint64_t PPB_path_counter = 0;
        int PPBgenerated_path_num[7] = {-1,-1,-1,-1,-1,-1,-1};
        double PPBstarting_pos[7] = {0,};
        double Defualt_WaitingT = 2.0; // Defualt waiting time (unit: s)
        bool PPB_path_init(Eigen::MatrixXd Tar_point, double _Tar_vel[], double _Des_force[], double Waiting_time[],int Point_num);
        bool PPB_path_exe(double path_out[]);
        /* PPB_path end */


        /* Single_PTP_path using Trapezoidal start */
        // Explain : This code use cpu allocation (that is, smaller calculation and memory is needed)

        double t0,tf; // t0: Init time (Normally set it "0"), tf: End time
        double x0,xf; // x0: Init position, xf: End position
        double tb,xb,ald; // tb: blended time, xb: blended location, ald: calculated
        double tc; // Current time to calculate current position

        // [ald]: Desired acceleration (m/s^2), [vd]: User desired velocity (m/s)
        // [x0]: Initial position (m), [xf]: Final position (m)
        bool Trape_1D_path_init(double ald_, double vd, double x0_, double xf_);
        bool Trape_1D_path_exe(double* x_out);

        /* Single_PTP_path using Trapezoidal end */

};

