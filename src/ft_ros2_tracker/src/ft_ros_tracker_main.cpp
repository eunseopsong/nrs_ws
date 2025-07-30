// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>
// #include <arpa/inet.h>
// #include <sys/socket.h>
// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <chrono>
// #include <signal.h>
// #include <csignal>

// #include "ros/ros.h"
// #include "ft_ros_tracker/vive_ft_msg.h"

// #define BUF_SIZE 14
// #define INIT_STAYING_TIME 500 // # of FT data
// #define INIT_TIME 500 // # of FT data

// void mySigintHandler(int sig)
// {
//     ros::shutdown();
//     exit(1);
//     printf("Program was terminated \n");
// }

// void errhandle(const char *errmsg);
// void shutdown(){
//     ros::shutdown();
// }

// int clnt_sock;

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "ft_ros_tracker");
//     ros::NodeHandle nh;

//     ros::Publisher Vive_Force_pub = nh.advertise<ft_ros_tracker::vive_ft_msg>("vive_force", 10);
//     ros::Publisher Vive_Moment_pub = nh.advertise<ft_ros_tracker::vive_ft_msg>("vive_moment", 10);

//     ft_ros_tracker::vive_ft_msg Vive_Force_data;
//     ft_ros_tracker::vive_ft_msg Vive_Moment_data;


//     // sockaddr_in 구조체 변수 선언
//     struct sockaddr_in st_serv_addr;
//     // 보내고 받을 버퍼 정의
//     double Force_val[3], Moment_val[3];
//     unsigned char recvmsg[BUF_SIZE]; // CAN to Ethernet must be set the buffer size 14
//     char sendbuf[14] = { 0x04,0x00,0x00,0x01,0x02,0x06,0x01,0x03,0x01,0x00,0x00,0x00,0x00,0x00 }; 
//     // ip, port 정의
//     char ip[] = "192.168.0.55";
//     int port = 4001;

//     // 클라이언트 소켓 TCP/IP 프로토콜 생성
//     clnt_sock = socket(PF_INET, SOCK_STREAM, 0);
//     if(clnt_sock == -1) errhandle("socket() ERR!");

//     // serv_sock에 bind로 주소 넣기 위한 밑작업
//     memset(&st_serv_addr,0,sizeof(st_serv_addr));
//     st_serv_addr.sin_family = AF_INET;
//     st_serv_addr.sin_addr.s_addr = inet_addr(ip);
//     st_serv_addr.sin_port = htons(port);

//     // connect()으로 서버소켓에 연결요청
//     int connret = connect(clnt_sock,(struct sockaddr*) &st_serv_addr, sizeof(st_serv_addr));
//     if(connret == -1) errhandle("connect() ERR!");


//     int iResult = send(clnt_sock, sendbuf, sizeof(sendbuf), 0);

//     ROS_INFO("Bytes Sent: %d\n", iResult);

//     int readstrlen = 0;

//     auto start_time = std::chrono::steady_clock::now();

//     double F_init[3] = {0,};
//     double M_init[3] = {0,};

//     uint64_t F_Init_counter = 0;
//     uint64_t M_Init_counter = 0;

//     while(1)
//     {
//         readstrlen = read(clnt_sock, (char*)&recvmsg, sizeof(recvmsg));
//         if(readstrlen == -1) errhandle("read() ERR!");

//         // --------------- Sensor data calculation --------------- //
//         if (recvmsg[4] == 1) // if ID is 1
//         { 
//             for (int i = 0; i < 3; i++){
//                 Force_val[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 100 - 300;

//                 if((F_Init_counter >= INIT_STAYING_TIME)&&(F_Init_counter < INIT_STAYING_TIME + INIT_TIME))
//                 {
//                     F_init[i] += Force_val[i];
//                     // test_counter[i] ++;
//                 }          
//             }
//                 // printf("F_Init_counter: %d\n", F_Init_counter);
//                 // printf("%d, %d, %d \n",test_counter[0],test_counter[1],test_counter[2]);
        
//             if(F_Init_counter > INIT_STAYING_TIME + INIT_TIME)
//             {
//                 Vive_Force_data.Fx = Force_val[0] - F_init[0]/((double)INIT_TIME);
//                 Vive_Force_data.Fy = Force_val[1] - F_init[1]/((double)INIT_TIME);
//                 Vive_Force_data.Fz = Force_val[2] - F_init[2]/((double)INIT_TIME);

//                 printf("Fx: %5.3f, Fy: %5.3f, Fz: %5.3f \n", Vive_Force_data.Fx, Vive_Force_data.Fy, Vive_Force_data.Fz);

//                 Vive_Force_pub.publish(Vive_Force_data);
//             }
//             else{F_Init_counter++;}     
//         }

//         else if (recvmsg[4] == 2) // if ID is 2
//         {
//             for (int i = 0; i < 3; i++){
//                 Moment_val[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 500 - 50;

//                 if((M_Init_counter >= INIT_STAYING_TIME)&&(M_Init_counter < INIT_STAYING_TIME + INIT_TIME))
//                 {
//                     M_init[i] += Moment_val[i];
//                 }
//             }
            
//             if(F_Init_counter > INIT_STAYING_TIME + INIT_TIME)
//             {
//                 Vive_Moment_data.Mx = Moment_val[0];
//                 Vive_Moment_data.My = Moment_val[1];
//                 Vive_Moment_data.Mz = Moment_val[2];

//                 printf("Mx: %5.3f, My: %5.3f, Mz: %5.3f \n", Vive_Moment_data.Mx, Vive_Moment_data.My, Vive_Moment_data.Mz);
//                 Vive_Moment_pub.publish(Vive_Moment_data);
//             }
//             else{M_Init_counter++;}
//         }
//          ros::spinOnce();
//     }

//     signal(SIGINT, mySigintHandler);
//     shutdown();
//     close(clnt_sock);
//     return 0;
// }

// void errhandle(const char *errmsg){
//   ROS_ERROR("%s", errmsg);
//   close(clnt_sock);
//   exit(1);
// }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <signal.h>
#include <csignal>

#include <rclcpp/rclcpp.hpp>                   ///// #include "ros/ros.h"
#include "ft_ros2_tracker/msg/vive_ft_msg.hpp" //// #include "ft_ros_tracker/vive_ft_msg.h"

#define BUF_SIZE 14
#define INIT_STAYING_TIME 500 // # of FT data
#define INIT_TIME 500 // # of FT data

int clnt_sock;

// Signal handler function to handle Ctrl+C
void mySigintHandler(int sig)
{
    printf("Shutting down...\n");
    ros::shutdown();
    close(clnt_sock); // Ensure the socket is closed
    exit(0);
}

void errhandle(const char *errmsg);

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "ft_ros_tracker", ros::init_options::NoSigintHandler); // Disable default SIGINT handler
    ros::NodeHandle nh;

    // Setup signal handling
    signal(SIGINT, mySigintHandler);

    ros::Publisher Vive_Force_pub = nh.advertise<ft_ros_tracker::vive_ft_msg>("vive_force", 10);
    ros::Publisher Vive_Moment_pub = nh.advertise<ft_ros_tracker::vive_ft_msg>("vive_moment", 10);

    ft_ros_tracker::vive_ft_msg Vive_Force_data;
    ft_ros_tracker::vive_ft_msg Vive_Moment_data;

    struct sockaddr_in st_serv_addr;
    double Force_val[3], Moment_val[3];
    unsigned char recvmsg[BUF_SIZE];
    char sendbuf[14] = { 0x04,0x00,0x00,0x01,0x02,0x06,0x01,0x03,0x01,0x00,0x00,0x00,0x00,0x00 }; 
    // char ip[] = "192.168.0.55";
    char ip[] = "192.168.0.42";
    int port = 4001;

    clnt_sock = socket(PF_INET, SOCK_STREAM, 0);
    if(clnt_sock == -1) errhandle("socket() ERR!");

    memset(&st_serv_addr,0,sizeof(st_serv_addr));
    st_serv_addr.sin_family = AF_INET;
    st_serv_addr.sin_addr.s_addr = inet_addr(ip);
    st_serv_addr.sin_port = htons(port);

    int connret = connect(clnt_sock, (struct sockaddr*) &st_serv_addr, sizeof(st_serv_addr));
    if(connret == -1) errhandle("connect() ERR!");

    int iResult = send(clnt_sock, sendbuf, sizeof(sendbuf), 0);
    ROS_INFO("Bytes Sent: %d\n", iResult);

    int readstrlen = 0;
    auto start_time = std::chrono::steady_clock::now();

    double F_init[3] = {0,};
    double M_init[3] = {0,};

    uint64_t F_Init_counter = 0;
    uint64_t M_Init_counter = 0;

    while (ros::ok())
    {
        readstrlen = read(clnt_sock, (char*)&recvmsg, sizeof(recvmsg));
        if(readstrlen == -1) errhandle("read() ERR!");

        if (recvmsg[4] == 1)
        { 
            for (int i = 0; i < 3; i++){
                Force_val[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 100 - 300;

                if((F_Init_counter >= INIT_STAYING_TIME)&&(F_Init_counter < INIT_STAYING_TIME + INIT_TIME))
                {
                    F_init[i] += Force_val[i];
                }          
            }
        
            if(F_Init_counter > INIT_STAYING_TIME + INIT_TIME)
            {
                Vive_Force_data.Fx = Force_val[0] - F_init[0]/((double)INIT_TIME);
                Vive_Force_data.Fy = Force_val[1] - F_init[1]/((double)INIT_TIME);
                Vive_Force_data.Fz = Force_val[2] - F_init[2]/((double)INIT_TIME);

                printf("Fx: %5.3f, Fy: %5.3f, Fz: %5.3f \n", Vive_Force_data.Fx, Vive_Force_data.Fy, Vive_Force_data.Fz);

                Vive_Force_pub.publish(Vive_Force_data);
            }
            else { F_Init_counter++; }     
        }

        else if (recvmsg[4] == 2)
        {
            for (int i = 0; i < 3; i++){
                Moment_val[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 500 - 50;

                if((M_Init_counter >= INIT_STAYING_TIME)&&(M_Init_counter < INIT_STAYING_TIME + INIT_TIME))
                {
                    M_init[i] += Moment_val[i];
                }
            }
            
            if(F_Init_counter > INIT_STAYING_TIME + INIT_TIME)
            {
                Vive_Moment_data.Mx = Moment_val[0];
                Vive_Moment_data.My = Moment_val[1];
                Vive_Moment_data.Mz = Moment_val[2];

                printf("Mx: %5.3f, My: %5.3f, Mz: %5.3f \n", Vive_Moment_data.Mx, Vive_Moment_data.My, Vive_Moment_data.Mz);
                Vive_Moment_pub.publish(Vive_Moment_data);
            }
            else { M_Init_counter++; }
        }
        ros::spinOnce();
    }

    close(clnt_sock);
    return 0;
}

void errhandle(const char *errmsg){
    ROS_ERROR("%s", errmsg);
    close(clnt_sock);
    exit(1);
}
