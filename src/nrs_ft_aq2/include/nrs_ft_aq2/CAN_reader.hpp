/* For formal */
#include <stdio.h>
#include <cmath>

#include <iostream>
#include <vector>
#include <numeric>

/* For TCP/IP */
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#define BUF_SIZE 14
#define SYSTEM_VERSION 0 // 0: SKKU, 1: AIDIN, 2: Workpiece

/* For Sensor transform */
#define Sen_transform 1 // 0: defualt value, 1: transformed value

class NRS_FTSensor
{
    /* Informations */

    // char ip; // ECAN IP (URE ECAN : 192.168.111.44)
    // int port; // ECAN PORT (URE ECAN : 4001)

private:

    unsigned char Handle_ID,Contact_ID;

#if (SYSTEM_VERSION == 0) // SKKU version - Robot side
    // CAN ID setting
    unsigned char Handle_ForceID = 0x01;  // SKKU 0x01
    unsigned char Handle_MomentID = 0x02; // SKKU 0x02
    unsigned char Handle_LAccID = 0x03;
    unsigned char Handle_AAccID = 0x04;
    unsigned char Contact_ForceID = 0x0b;  // SKKU 0x0b
    unsigned char Contact_MomentID = 0xc; // SKKU 0xc

    // parameters for sensor setting
    char sendbuf[BUF_SIZE] = {0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    char Contact_sendbuf[BUF_SIZE] = {0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x0b, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif

#if (SYSTEM_VERSION == 1) // AIDIN version
    // CAN ID setting
    unsigned char Handle_ForceID = 0x0b;  // SKKU 0x01
    unsigned char Handle_MomentID = 0x0c; // SKKU 0x02
    unsigned char Handle_LAccID = 0x03;
    unsigned char Handle_AAccID = 0x04;
    unsigned char Contact_ForceID = 0x01;  // SKKU 0x0b
    unsigned char Contact_MomentID = 0x02; // SKKU 0xc

    // parameters for sensor setting
    char sendbuf[BUF_SIZE] = {0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x0b, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    char Contact_sendbuf[BUF_SIZE] = {0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif

#if (SYSTEM_VERSION == 2) // SKKU version - Workpiece side
    // CAN ID setting
    unsigned char Handle_ForceID = 0x01;  // SKKU 0x01
    unsigned char Handle_MomentID = 0x02; // SKKU 0x02
    unsigned char Handle_LAccID = 0x03;
    unsigned char Handle_AAccID = 0x04;
    unsigned char Contact_ForceID = 0x11;  // SKKU 0x0b
    unsigned char Contact_MomentID = 0x12; // SKKU 0xc

    // parameters for sensor setting
    char sendbuf[BUF_SIZE] = {0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    char Contact_sendbuf[BUF_SIZE] = {0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x11, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif

    // char Contact_sendbuf[BUF_SIZE] = { 0x04,0x00,0x00,0x01,0x02,0x06,0x11,0x03,0x01,0x00,0x00,0x00,0x00,0x00 };

    uint16_t return_out = 0;

    // parameters that do not touch
    bool init_flag = false;
    int clnt_sock;
    int readstrlen = 0;

    struct sockaddr_in st_serv_addr; // sockaddr_in 구조체 변수 선언
    unsigned char recvmsg[BUF_SIZE]; // CAN to Ethernet must be set the buffer size 16

    /* For FT Sensor Value Transform */
    std::vector<double> inter_force, inter_moment;
    /* For Contact FT Sensor Value Transform */
    std::vector<double> Cinter_force, Cinter_moment;
    /* For ACC Value Transform */
    std::vector<double> inter_posAcc, inter_angAcc;
    /* For Contact ACC Value Transform */
    std::vector<double> Cinter_posAcc, Cinter_angAcc;


public:
    NRS_FTSensor(unsigned char Handle_ID_, unsigned char Contact_ID_, bool HaccSwitch_, bool CaccSwitch_);
    ~NRS_FTSensor() {}

    double CAN_sampling = 1 / 50; // period(s)
    std::vector<double> Force_val, Moment_val, Contact_Force_val, Contact_Moment_val, Pos_acc_val, Ang_acc_val, CPos_acc_val, CAng_acc_val;

    int init_average_num = 1000; // To use average force value for force initialization

    int sensor_init_counter = 0;

    /* Init values */
    std::vector<double> init_Force, init_Moment, init_Contact_Force, init_Contact_Moment, Ang_vel_val, Ang_pvel_val, CAng_vel_val, CAng_pvel_val;

    /* Sensor Coordinate set */
    std::vector<int> H_sen_order, H_sen_sign, C_sen_order, C_sen_sign;

    // int H_Fx = 2;
    // int H_Fx_sign = 1; // 1: minus, 3: plus
    // int H_Fy = 0;
    // int H_Fy_sign = 1;
    // int H_Fz = 1;
    // int H_Fz_sign = 1;

    // int C_Fx = 1;      // SKKU : 1
    // int C_Fx_sign = 1; // SKKU : 1
    // int C_Fy = 0;      // SKKU : 0
    // int C_Fy_sign = 1; // SKKU : 1
    // int C_Fz = 2;      // SKKU : 2
    // int C_Fz_sign = 3; // SKKU : 3


    void TCP_init(char *IP, int port);
    uint16_t TCP_start();
    bool Sensor_value_init();
    void errhandle(const char *errmsg);
};
