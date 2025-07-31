#include "CAN_reader.hpp"

NRS_FTSensor::NRS_FTSensor(unsigned char Handle_ID_, unsigned char Contact_ID_, bool HaccSwitch_, bool CaccSwitch_)
:inter_force(3,0), inter_moment(3), Cinter_force(3), Cinter_moment(3), inter_posAcc(3), inter_angAcc(3), Cinter_posAcc(3), Cinter_angAcc(3),
init_Force(3), init_Moment(3), init_Contact_Force(3), init_Contact_Moment(3), Ang_vel_val(3), Ang_pvel_val(3), CAng_vel_val(3), CAng_pvel_val(3),
Force_val(3), Moment_val(3), Contact_Force_val(3), Contact_Moment_val(3), Pos_acc_val(3), Ang_acc_val(3), CPos_acc_val(3), CAng_acc_val(3),
Handle_ID(Handle_ID_), Contact_ID(Contact_ID_)
{
    sendbuf[6] = Handle_ID;
    Contact_sendbuf[6] = Contact_ID;
    if(HaccSwitch_) {sendbuf[7] = 0x06;} // Acceleration On
    else {sendbuf[7] = 0x03;} // Acceleration Off
    if(CaccSwitch_) {Contact_sendbuf[7] = 0x06;} // Acceleration On
    else {Contact_sendbuf[7] = 0x03;} // Acceleration Off
    H_sen_order= {2,0,1};
    H_sen_sign = {1,1,1};
    C_sen_order = {1,0,2};
    C_sen_sign = {1,1,3};
}

void NRS_FTSensor::TCP_init(char *IP, int port)
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
        errhandle("connect() ERR!");

    // To Hand-guiding sensor
    int iResult = send(clnt_sock, sendbuf, sizeof(sendbuf), 0);
    printf("Bytes Sent: %d\n", iResult);
    // To Contact force sensor
    iResult = send(clnt_sock, Contact_sendbuf, sizeof(Contact_sendbuf), 0);
    printf("Bytes Sent: %d\n", iResult);

    init_flag = true;
}

uint16_t NRS_FTSensor::TCP_start()
{
    if (init_flag == true)
    {
        readstrlen = read(clnt_sock, (char *)&recvmsg, sizeof(recvmsg));
        if (readstrlen == -1)
            errhandle("read() ERR!");

        // --------------- Sensor data calculation --------------- //
        if (recvmsg[4] == Handle_ID) // if ID is 1 -> Sensor1 force (Handle part)
        {

            #if 0 // To measure the sensor's sampling time
            gettimeofday(&tval_start, NULL);  // Get current time
            tval_microseconds = tval_end.tv_usec - tval_start.tv_usec;
            memcpy(&tval_end,&tval_start,sizeof(tval_start));
            printf("Force time interval: %d micros\n",tval_microseconds);
            #endif

            for (int i = 0; i < 3; i++)
            {
                inter_force[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 100 - 300;
            }

            // FT sensor transform according to its assembly configuration
            // original configuration
            #if (Sen_transform == 0)
            Force_val[0] = -inter_force[0] + init_Force[0];
            Force_val[1] = -inter_force[1] + init_Force[1];
            Force_val[2] = -inter_force[2] + init_Force[2];
            #endif
            // vertical configuration
            #if (Sen_transform == 1)
            Force_val[0] = -(H_sen_sign[0] - 2) * inter_force[H_sen_order[0]] + (H_sen_sign[0] - 2) * init_Force[H_sen_order[0]];
            Force_val[1] = -(H_sen_sign[1] - 2) * inter_force[H_sen_order[1]] + (H_sen_sign[1] - 2) * init_Force[H_sen_order[1]];
            Force_val[2] = -(H_sen_sign[2] - 2) * inter_force[H_sen_order[2]] + (H_sen_sign[2] - 2) * init_Force[H_sen_order[2]];
            #endif

            return_out = 1;
        }
        else if (recvmsg[4] == Handle_ID+1) // if ID is 2 -> Sensor1 moment (Handle part)
        {
            for (int i = 0; i < 3; i++)
            {
                inter_moment[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 500 - 50;
            }

            // FT sensor transform according to its assembly configuration

            // original configuration
            #if (Sen_transform == 0)
            Moment_val[0] = -inter_moment[0] + init_Moment[0]; // -y value -> x value
            Moment_val[1] = -inter_moment[1] + init_Moment[1]; // -x value -> y value
            Moment_val[2] = -inter_moment[2] + init_Moment[2]; // -z value -> z value
            #endif
            // vertical configuration
            #if (Sen_transform == 1)
            Moment_val[0] = -(H_sen_sign[0] - 2) * inter_moment[H_sen_order[0]] + (H_sen_sign[0] - 2) * init_Moment[H_sen_order[0]];
            Moment_val[1] = -(H_sen_sign[1] - 2) * inter_moment[H_sen_order[1]] + (H_sen_sign[1] - 2) * init_Moment[H_sen_order[1]];
            Moment_val[2] = -(H_sen_sign[2] - 2) * inter_moment[H_sen_order[2]] + (H_sen_sign[2] - 2) * init_Moment[H_sen_order[2]];
            #endif

            return_out = 2;
        }
        else if (recvmsg[4] == Handle_ID+2) // if ID is 3 -> Sensor1 linear acceleration (LAx,LAy,LAz)
        {
            for (int i = 0; i < 3; i++)
            {
                inter_posAcc[i] = ((double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i])) * 2 - 65535;
                inter_posAcc[i] = (inter_posAcc[i] / 16384) * 9.81; // m/s^2
            }

            // 45degree rotation
            Pos_acc_val[0] = -(cos(45 * 3.141592 / 180) * inter_posAcc[0] - sin(45 * 3.141592 / 180) * inter_posAcc[1]);
            Pos_acc_val[1] = -(sin(45 * 3.141592 / 180) * inter_posAcc[0] + cos(45 * 3.141592 / 180) * inter_posAcc[1]);
            Pos_acc_val[2] = -inter_posAcc[2];

            // FT sensor transform according to its assembly configuration
            Pos_acc_val[0] = -(H_sen_sign[0] - 2) * Pos_acc_val[H_sen_order[0]] + (H_sen_sign[0] - 2) * Pos_acc_val[H_sen_order[0]];
            Pos_acc_val[1] = -(H_sen_sign[1] - 2) * Pos_acc_val[H_sen_order[1]] + (H_sen_sign[1] - 2) * Pos_acc_val[H_sen_order[1]];
            Pos_acc_val[2] = -(H_sen_sign[2] - 2) * Pos_acc_val[H_sen_order[2]] + (H_sen_sign[2] - 2) * Pos_acc_val[H_sen_order[2]];

            return_out = 3;
        }
        else if (recvmsg[4] == Handle_ID+3) // if ID is 4 -> Sensor1 angular acceleration (AAx,AAy,AAz)
        {
            for (int i = 0; i < 3; i++)
            {
                Ang_vel_val[i] = ((double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i])) * 2 - 65535;
                Ang_vel_val[i] = (Ang_vel_val[i] * 250 / 32768) * (3.141592 / 180); // rad/s

                inter_angAcc[i] = (Ang_vel_val[i] - Ang_pvel_val[i]) / CAN_sampling; // rad/s^2

                Ang_pvel_val[i] = Ang_vel_val[i];
            }

            // 45degree rotation
            Ang_acc_val[0] = -(cos(45 * 3.141592 / 180) * inter_angAcc[0] - sin(45 * 3.141592 / 180) * inter_angAcc[1]);
            Ang_acc_val[1] = -(sin(45 * 3.141592 / 180) * inter_angAcc[0] + cos(45 * 3.141592 / 180) * inter_angAcc[1]);
            Ang_acc_val[2] = -inter_angAcc[2];

            // FT sensor transform according to its assembly configuration
            Ang_acc_val[0] = -(H_sen_sign[0] - 2) * Ang_acc_val[H_sen_order[0]] + (H_sen_sign[0] - 2) * Ang_acc_val[H_sen_order[0]];
            Ang_acc_val[1] = -(H_sen_sign[1] - 2) * Ang_acc_val[H_sen_order[1]] + (H_sen_sign[1] - 2) * Ang_acc_val[H_sen_order[1]];
            Ang_acc_val[2] = -(H_sen_sign[2] - 2) * Ang_acc_val[H_sen_order[2]] + (H_sen_sign[2] - 2) * Ang_acc_val[H_sen_order[2]];

            return_out = 4;
        }
        else if (recvmsg[4] == Contact_ID) // if ID is 11 -> Sensor2 force (Contact part) 0b
        {
            for (int i = 0; i < 3; i++)
            {
                Cinter_force[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 100 - 300;
            }

            // FT sensor transform according to its assembly configuration

            // original configuration
            #if (Sen_transform == 0)
            Contact_Force_val[0] = -Cinter_force[0] + init_Contact_Force[0];
            Contact_Force_val[1] = -Cinter_force[1] + init_Contact_Force[1];
            Contact_Force_val[2] = -Cinter_force[2] + init_Contact_Force[2];
            #endif
            // vertical configuration
            #if (Sen_transform == 1)
            Contact_Force_val[0] = -(C_sen_sign[0] - 2) * Cinter_force[C_sen_order[0]] + (C_sen_sign[0] - 2) * init_Contact_Force[C_sen_order[0]];
            Contact_Force_val[1] = -(C_sen_sign[1] - 2) * Cinter_force[C_sen_order[1]] + (C_sen_sign[1] - 2) * init_Contact_Force[C_sen_order[1]];
            Contact_Force_val[2] = -(C_sen_sign[2] - 2) * Cinter_force[C_sen_order[2]] + (C_sen_sign[2] - 2) * init_Contact_Force[C_sen_order[2]];
            #endif

            return_out = 11;
        }
        else if (recvmsg[4] == Contact_ID+1) // if ID is 12 -> Sensor2 moment (Contact part) 0c
        {
            for (int i = 0; i < 3; i++)
            {
                Cinter_moment[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 500 - 50;
            }

            // FT sensor transform according to its assembly configuration

            // original configuration
            #if (Sen_transform == 0)
            Contact_Moment_val[0] = -Cinter_moment[0] + init_Contact_Moment[0]; // -y value -> x value
            Contact_Moment_val[1] = -Cinter_moment[1] + init_Contact_Moment[1]; // -x value -> y value
            Contact_Moment_val[2] = -Cinter_moment[2] + init_Contact_Moment[2]; // -z value -> z value
            #endif
            // vertical configuration
            #if (Sen_transform == 1)
            Contact_Moment_val[0] = -(C_sen_sign[0] - 2) * Cinter_moment[C_sen_order[0]] + (C_sen_sign[0] - 2) * init_Contact_Moment[C_sen_order[0]];
            Contact_Moment_val[1] = -(C_sen_sign[1] - 2) * Cinter_moment[C_sen_order[1]] + (C_sen_sign[1] - 2) * init_Contact_Moment[C_sen_order[1]];
            Contact_Moment_val[2] = -(C_sen_sign[2] - 2) * Cinter_moment[C_sen_order[2]] + (C_sen_sign[2] - 2) * init_Contact_Moment[C_sen_order[2]];
            #endif

            return_out = 12;
        }
        else if (recvmsg[4] == Contact_ID+2) // if ID is 13 -> Sensor2 linear acceleration (LAx,LAy,LAz)
        {
            for (int i = 0; i < 3; i++)
            {
                Cinter_posAcc[i] = ((double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i])) * 2 - 65535;
                Cinter_posAcc[i] = (Cinter_posAcc[i] / 16384) * 9.81; // m/s^2
            }

            // 45degree rotation
            CPos_acc_val[0] = -(cos(45 * 3.141592 / 180) * Cinter_posAcc[0] - sin(45 * 3.141592 / 180) * Cinter_posAcc[1]);
            CPos_acc_val[1] = -(sin(45 * 3.141592 / 180) * Cinter_posAcc[0] + cos(45 * 3.141592 / 180) * Cinter_posAcc[1]);
            CPos_acc_val[2] = -Cinter_posAcc[2];

            // FT sensor transform according to its assembly configuration
            CPos_acc_val[0] = -(C_sen_sign[0] - 2) * CPos_acc_val[C_sen_order[0]] + (C_sen_sign[0] - 2) * CPos_acc_val[C_sen_order[0]];
            CPos_acc_val[1] = -(C_sen_sign[1] - 2) * CPos_acc_val[C_sen_order[1]] + (C_sen_sign[1] - 2) * CPos_acc_val[C_sen_order[1]];
            CPos_acc_val[2] = -(C_sen_sign[2] - 2) * CPos_acc_val[C_sen_order[2]] + (C_sen_sign[2] - 2) * CPos_acc_val[C_sen_order[2]];

            return_out = 13;
        }
        else if (recvmsg[4] == Contact_ID+3) // if ID is 14 -> Sensor2 angular acceleration (AAx,AAy,AAz)
        {
            for (int i = 0; i < 3; i++)
            {
                CAng_vel_val[i] = ((double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i])) * 2 - 65535;
                CAng_vel_val[i] = (CAng_vel_val[i] * 250 / 32768) * (3.141592 / 180); // rad/s

                Cinter_angAcc[i] = (CAng_vel_val[i] - CAng_pvel_val[i]) / CAN_sampling; // rad/s^2

                CAng_pvel_val[i] = CAng_vel_val[i];
            }

            // 45degree rotation
            CAng_acc_val[0] = -(cos(45 * 3.141592 / 180) * Cinter_angAcc[0] - sin(45 * 3.141592 / 180) * Cinter_angAcc[1]);
            CAng_acc_val[1] = -(sin(45 * 3.141592 / 180) * Cinter_angAcc[0] + cos(45 * 3.141592 / 180) * Cinter_angAcc[1]);
            CAng_acc_val[2] = -Cinter_angAcc[2];

            // FT sensor transform according to its assembly configuration
            CAng_acc_val[0] = -(C_sen_sign[0] - 2) * CAng_acc_val[C_sen_order[0]] + (C_sen_sign[0] - 2) * CAng_acc_val[C_sen_order[0]];
            CAng_acc_val[1] = -(C_sen_sign[1] - 2) * CAng_acc_val[C_sen_order[1]] + (C_sen_sign[1] - 2) * CAng_acc_val[C_sen_order[1]];
            CAng_acc_val[2] = -(C_sen_sign[2] - 2) * CAng_acc_val[C_sen_order[2]] + (C_sen_sign[2] - 2) * CAng_acc_val[C_sen_order[2]];

            return_out = 14;
        }
        else
        {
            return_out = 0;
        }
    }
    else
    {
        errhandle("Connection initialization was failed, please run it again");
        return_out = 0;
    }

    return return_out;
}
bool NRS_FTSensor::Sensor_value_init()
{
    if (sensor_init_counter == 0)
    {
        for (int i = 0; i < 3; i++)
        {
            init_Force[i] = 0;
            init_Moment[i] = 0;

            init_Contact_Force[i] = 0;
            init_Contact_Moment[i] = 0;
        }
        printf("Sensor is on zeroset \n");
    }
    if (sensor_init_counter < init_average_num)
    {
        for (int i = 0; i < 3; i++)
        {
            init_Force[i] += inter_force[i] / init_average_num;
            init_Moment[i] += inter_moment[i] / init_average_num;

            init_Contact_Force[i] += Cinter_force[i] / init_average_num;
            init_Contact_Moment[i] += Cinter_moment[i] / init_average_num;
        }

        sensor_init_counter++;
    }

    if (sensor_init_counter < init_average_num)
    { return false; }
    else
    {
        // printf("%.2f, %.2f, %.2f \n", init_Force[0],init_Force[1],init_Force[2]);
        return true;
    }
        
}

void NRS_FTSensor::errhandle(const char *errmsg)
{ // for FT_communication
    fputs(errmsg, stderr);
    fputc('\n', stderr);
    close(clnt_sock);
    exit(1);
}