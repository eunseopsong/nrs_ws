#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
/* To get time interval */
#include <sys/time.h> 
/* For TCP/IP */
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#define BUF_SIZE 14

class NRS_CAN_sender
{
    public:
        /* Buffer to send (defualt value) */
        char sendbuf[BUF_SIZE] = {0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

        // parameters that do not touch
        bool init_flag = false;
        int clnt_sock;
        int readstrlen = 0;

        struct sockaddr_in st_serv_addr; // sockaddr_in 구조체 변수 선언
        unsigned char recvmsg[BUF_SIZE]; // CAN to Ethernet must be set the buffer size 16

        /* To get sampling time of sensor */
        timeval tval_start, tval_end;
        int tval_microseconds;
        struct timeval last_time[256] = {0};

        NRS_CAN_sender() {};
        ~NRS_CAN_sender();


        void TCP_connect(char *IP, int port);
        void CAN_sender_input();
        void CAN_send();
        void CAN_start();

        void errhandle(const char *errmsg);

    private:

        int pre_ID = 0;
};
