#include "nrs_ft_aq2/CAN_sender.hpp"

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
    if (connret == -1)
    {errhandle("connect() ERR!");}
    else
    {printf("TCP/IP was connected: %s \n",IP);}
}
void NRS_CAN_sender::CAN_sender_input()
{
    int input;

    data_sel:
    std::cout << "Select the CAN ID(DEC): ";
    std::cin >> input;

    sendbuf[6] = input;

    std::cout << "Type the data to send(DEC): ";
    for(int i=0;i<7;i++)
    {
        std::cin >> input;
        sendbuf[7+i] = input;
        printf("ID:%d, Data(%d): %d, %d, %d, %d, %d, %d, %d \n",
        sendbuf[6], i, sendbuf[7],sendbuf[8],sendbuf[9],sendbuf[10],
        sendbuf[11],sendbuf[12],sendbuf[13]);
    }
    printf("Data was fully selected. Send?(yes:1, no:0) -> ");
    std::cin >> input;
    if(input == 0)
    {goto data_sel;}
}
void NRS_CAN_sender::CAN_send()
{
    int iResult = send(clnt_sock, sendbuf, sizeof(sendbuf), 0);
    printf("Bytes Sent: %d\n", iResult);
}
void NRS_CAN_sender::CAN_start()
{
    readstrlen = read(clnt_sock, (char *)&recvmsg, sizeof(recvmsg));
    if (readstrlen == -1) {
        errhandle("read() ERR!");
    } else {
        int id = (int)recvmsg[4];
        struct timeval current_time;
        gettimeofday(&current_time, NULL);  // Get current time

        if (last_time[id].tv_sec != 0) {  // If we have a previous timestamp
            long sec_diff = current_time.tv_sec - last_time[id].tv_sec;
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
{ // for FT_communication
    fputs(errmsg, stderr);
    fputc('\n', stderr);
    close(clnt_sock);
    exit(1);
}



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_sender");
	ros::NodeHandle nh;

    NRS_CAN_sender NRS_CAN;

    NRS_CAN.TCP_connect("192.168.0.42",4001);
    NRS_CAN.CAN_sender_input();
    NRS_CAN.CAN_send();

    double frequency = 2000.0; // Hz
    ros::Rate loop_rate(frequency);
    
    while(ros::ok())
    {
        NRS_CAN.CAN_start();
        loop_rate.sleep();    // sleep to maintain loop rate
    }

    return 0;
}