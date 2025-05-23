/* For formal */
#include <stdio.h>
#include <cmath>

/* To identify the sampling time */
#include <sys/time.h>

/* For TCP/IP */
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#define BUF_SIZE 14

/* Sample time calculation on/off */
#define SamTime_Cal_OnOff 0 // On: 1, Off; 0

/* For Sensor transform */
#define Sen_transform 1 // 0: defualt value, 1: transformed value

// If sensor Fz -> -Fx
// H_Fx 2, H_Fx_sign 1
// Force[0] = (H_Fx_sign-2)*Force[H_Fx];
// If sensor Fy -> Fy
// H_Fy 1, H_Fy_sign 3
// Force[1] = (H_Fy_sign-2)*Force[H_Fy];

#define H_Fx 2 
#define H_Fx_sign 1 // 1: minus, 3: plus
#define H_Fy 0
#define H_Fy_sign 1
#define H_Fz 1
#define H_Fz_sign 1

#define C_Fx 1
#define C_Fx_sign 1
#define C_Fy 0
#define C_Fy_sign 1
#define C_Fz 2
#define C_Fz_sign 3

class Yoon_FT_sensor
{   
    /* Informations */

    // char ip; // ECAN IP (URE ECAN : 192.168.111.44)
    // int port; // ECAN PORT (URE ECAN : 4001)

    private:

        // parameters for setting
        char sendbuf[BUF_SIZE] = { 0x04,0x00,0x00,0x01,0x02,0x06,0x01,0x03,0x01,0x00,0x00,0x00,0x00,0x00 };
        char Contact_sendbuf[BUF_SIZE] = { 0x04,0x00,0x00,0x01,0x02,0x06,0x0b,0x03,0x01,0x00,0x00,0x00,0x00,0x00 };
        // char Contact_sendbuf[BUF_SIZE] = { 0x04,0x00,0x00,0x01,0x02,0x06,0x11,0x03,0x01,0x00,0x00,0x00,0x00,0x00 };  
        bool init_flag = false;
        uint16_t return_out = 0;

        // parameters that do not touch
        int clnt_sock;
        int readstrlen = 0;

        struct sockaddr_in st_serv_addr; // sockaddr_in 구조체 변수 선언
        unsigned char recvmsg[BUF_SIZE]; // CAN to Ethernet must be set the buffer size 16

        // Calculation parameters
        double inter_force[3] = {0,}; // for force value transform
        double inter_moment[3] = {0,}; // for moment value transform
        double Cinter_force[3] = {0,}; // for force value transform
        double Cinter_moment[3] = {0,}; // for moment value transform
        double inter_posAcc[3] = {0,}; // for linear acceleration value transform
        double inter_angAcc[3] = {0,}; // for angular acceleration value transform

        
    public:
        Yoon_FT_sensor() {}
        ~Yoon_FT_sensor() {}

        /* To identify the sampling time */
        timeval Htval_start, Htval_end, Ctval_start, Ctval_end; // 시간 저장용 구조체
        int Htval_microseconds; // Handle sensor sampling time  
        int Ctval_microseconds; // Contact sensor sampling time

        double CAN_sampling = 1/50; // period(s)
        double Force_val[3], Moment_val[3], Contact_Force_val[3], Contact_Moment_val[3], Pos_acc_val[3], Ang_acc_val[3];
        
        int sensor_init_counter = 0;
        int init_average_num = 100; // To use average force value for force initialization
        double init_Force[3] = {0,}; // initial values
        double init_Moment[3] = {0,}; // initial values
        double init_Contact_Force[3] = {0,}; // initial values
        double init_Contact_Moment[3] = {0,}; // initial values

        double Ang_vel_val[3] = {0,};
        double Ang_pvel_val[3] = {0,};
        

        void TCP_init(char* IP, int port);
        uint16_t TCP_start();
        bool Sensor_value_init();
        void errhandle(const char *errmsg);



};
