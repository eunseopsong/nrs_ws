#include <iostream>
#include <stdio.h>
#include <memory>

/* ROS2 */
#include "rclcpp/rclcpp.hpp"

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
    /* Buffer to send (default value) */
    unsigned char sendbuf[BUF_SIZE] = {
        0x04, 0x00, 0x00, 0x01,
        0x02, 0x06, 0x01, 0x03,
        0x01, 0x00, 0x00, 0x00,
        0x00, 0x00
    };

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

    NRS_CAN_sender() {}
    ~NRS_CAN_sender();

    void TCP_connect(char *IP, int port);
    void CAN_sender_input();
    void CAN_send();
    void CAN_start();

    void errhandle(const char *errmsg);

private:
    int pre_ID = 0;
};

NRS_CAN_sender::~NRS_CAN_sender()
{
    close(clnt_sock);
}

void NRS_CAN_sender::TCP_connect(char *IP, int port)
{
    // 클라이언트 소켓 TCP/IP 프로토콜 생성
    clnt_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (clnt_sock == -1)
        errhandle("socket() ERR!");

    // serv_sock에 bind로 주소 넣기 위한 밑작업
    memset(&st_serv_addr, 0, sizeof(st_serv_addr));
    st_serv_addr.sin_family = AF_INET;
    st_serv_addr.sin_addr.s_addr = inet_addr(IP);
    st_serv_addr.sin_port = htons(port);

    // connect()으로 서버소켓에 연결요청
    int connret = connect(clnt_sock, (struct sockaddr *)&st_serv_addr, sizeof(st_serv_addr));
    if (connret == -1) {
        errhandle("connect() ERR!");
    } else {
        printf("TCP/IP was connected: %s \n", IP);
    }
}

void NRS_CAN_sender::CAN_sender_input()
{
    int input;

data_sel:
    std::cout << "Select the CAN ID(DEC): ";
    std::cin >> input;

    sendbuf[6] = input;

    std::cout << "Type the data to send(DEC): ";
    for (int i = 0; i < 7; i++)
    {
        std::cin >> input;
        sendbuf[7 + i] = input;
        printf("ID:%u, Data(%u): %u, %u, %u, %u, %u, %u, %u \n",
               sendbuf[6], i,
               sendbuf[7], sendbuf[8], sendbuf[9], sendbuf[10],
               sendbuf[11], sendbuf[12], sendbuf[13]);
    }
    printf("Data was fully selected. Send?(yes:1, no:0) -> ");
    std::cin >> input;
    if (input == 0)
        goto data_sel;
}

void NRS_CAN_sender::CAN_send()
{
    int iResult = send(clnt_sock, sendbuf, sizeof(sendbuf), 0);
    printf("Bytes Sent: %d\n", iResult);

    printf("ID:%02x : %02x, %02x, %02x, %02x, %02x, %02x, %02x \n",
        sendbuf[6],
        sendbuf[7], sendbuf[8], sendbuf[9], sendbuf[10],
        sendbuf[11], sendbuf[12], sendbuf[13]);
}

void NRS_CAN_sender::CAN_start()
{
    readstrlen = read(clnt_sock, (char *)&recvmsg, sizeof(recvmsg));
    if (readstrlen == -1) {
        errhandle("read() ERR!");
    } else {
        int id = (int)recvmsg[4];
        struct timeval current_time;
        gettimeofday(&current_time, NULL);  // 현재 시간

        if (last_time[id].tv_sec != 0) {  // 이전 타임스탬프가 있으면
            long sec_diff  = current_time.tv_sec  - last_time[id].tv_sec;
            long usec_diff = current_time.tv_usec - last_time[id].tv_usec;
            double elapsed_ms = (double)(sec_diff * 1000000 + usec_diff) / 1000.0;

            printf("Sampling period for ID %d: %.3f ms\n", id, elapsed_ms);
        }

        // Update last time for this ID
        last_time[id] = current_time;

        printf("ID: %d, Data: %d, %d, %d, %d, %d, %d \n",
               id,
               (int)recvmsg[6], (int)recvmsg[7], (int)recvmsg[8],
               (int)recvmsg[9], (int)recvmsg[10], (int)recvmsg[11]);
    }
}

void NRS_CAN_sender::errhandle(const char *errmsg)
{
    fputs(errmsg, stderr);
    fputc('\n', stderr);
    close(clnt_sock);
    exit(1);
}

// =====================
// ROS2 main
// =====================
int main(int argc, char *argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("can_sender");

    NRS_CAN_sender NRS_CAN;

    // IP / Port 설정
    NRS_CAN.TCP_connect((char *)"192.168.0.44", 4001);
    NRS_CAN.CAN_sender_input();
    NRS_CAN.CAN_send();

    double frequency = 2000.0; // Hz
    rclcpp::Rate loop_rate(frequency);

    while (rclcpp::ok()) {
        NRS_CAN.CAN_start();
        rclcpp::spin_some(node);  // 콜백 있을 때 대비 (지금은 없음)
        loop_rate.sleep();        // 루프 주기 유지
    }

    rclcpp::shutdown();
    return 0;
}
