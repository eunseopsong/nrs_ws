#pragma once

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <vector>
#include <chrono>

#include "Y2Matrix/YMatrix.hpp"
#include "Y2FT_AQ/FT_EtherGet.hpp"   // FTData 정의 포함

#define BUF_SIZE 14

class FT_eCANGet
{
public:
    FT_eCANGet(const std::string& IP, const int PORT);
    ~FT_eCANGet();

    FTData FTGet();
    bool FT_init(const unsigned int init_count_num);

private:
    std::string  IP_;
    unsigned int PORT_;

    std::vector<double> init_force;   // Fx, Fy, Fz offset
    std::vector<double> init_moment;  // Mx, My, Mz offset

    int  clnt_sock = -1;
    bool init_flag = false;

    // CAN frame setting
    double        CAN_sampling = 1.0 / 500.0;  // 500 Hz
    unsigned char Sensor_ID    = 0x01; // skku: 0x01, aiding: 0x01

    char sendbuf[BUF_SIZE] = {
        0x04,0x00,0x00,0x01,
        0x02,0x06,0x01,0x03,
        0x01,0x00,0x00,0x00,
        0x00,0x00
    };

    float unpackFloat(unsigned char* recvmsg, int i=0, int data_type=0);

    // State variables for Angular acceleration - DEPRECATED (causes accumulation)
    // Kept for backward compatibility but not used
    double Ang_vel_val[4]  = {0,0,0,0};
    double Ang_pvel_val[4] = {0,0,0,0};

    int  readstrlen = 0;
    struct sockaddr_in st_serv_addr;
    unsigned char recvmsg[BUF_SIZE];

    // Sampling frequency measurement (force frame 기준)
    std::chrono::steady_clock::time_point last_stamp_;
    bool         have_last_stamp_ = false;
    unsigned int sample_count_    = 0;
    long long    sum_dt_us_       = 0;

    // 5초 대기용
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_wait_msg_time_;
    bool wait_5sec_passed_ = false;

    // 초기화 카운트
    unsigned int init_count  = 0;

    // FT_get 내부 연산용 변수
    FTData FTGet_ftdata;
};
