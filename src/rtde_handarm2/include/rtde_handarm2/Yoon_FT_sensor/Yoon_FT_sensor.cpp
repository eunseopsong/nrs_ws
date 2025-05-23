#include "Yoon_FT_sensor.h"

void Yoon_FT_sensor::TCP_init(char* IP, int port)
{
    // 클라이언트 소켓 TCP/IP 프로토콜 생성
    clnt_sock = socket(PF_INET, SOCK_STREAM, 0);
    if(clnt_sock == -1) errhandle("socket() ERR!");

    // serv_sock에 bind로 주소 넣기 위한 밑작업
    memset(&st_serv_addr,0,sizeof(st_serv_addr));
    st_serv_addr.sin_family = AF_INET;
    st_serv_addr.sin_addr.s_addr = inet_addr(IP);
    st_serv_addr.sin_port = htons(port);

    // connect()으로 서버소켓에 연결요청
    int connret = connect(clnt_sock,(struct sockaddr*) &st_serv_addr, sizeof(st_serv_addr));
    if(connret == -1) errhandle("connect() ERR!");

    
    // To Hand-guiding sensor
    int iResult = send(clnt_sock, sendbuf, sizeof(sendbuf), 0);
    printf("Bytes Sent: %d\n", iResult);
    // To Contact force sensor
    iResult = send(clnt_sock, Contact_sendbuf, sizeof(Contact_sendbuf), 0);
    printf("Bytes Sent: %d\n", iResult);

    init_flag = true;
}

uint16_t Yoon_FT_sensor::TCP_start()
{
    if(init_flag == true)
    {
        readstrlen = read(clnt_sock, (char*)&recvmsg, sizeof(recvmsg));
        if(readstrlen == -1) errhandle("read() ERR!");

        /**** Sensor data calculation ****/

        /* if ID is 1 -> Sensor1 force (Handle part) */
        if (recvmsg[4] == 0x01)
        {
            /* To measure the sensor's sampling time */
            #if SamTime_Cal_OnOff // 
            gettimeofday(&Htval_start, NULL);  // Get current time
            Htval_microseconds = Htval_end.tv_usec - Htval_start.tv_usec;
            memcpy(&Htval_end,&Htval_start,sizeof(Htval_start));
            #endif

            for (int i = 0; i < 3; i++)
            {
                inter_force[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 100 - 300;
            }

            /* FT sensor transform according to its assembly configuration */

            /* original configuration */
            #if(Sen_transform == 0)
            Force_val[0] = -inter_force[0] + init_Force[0]; 
            Force_val[1] = -inter_force[1] + init_Force[1]; 
            Force_val[2] = -inter_force[2] + init_Force[2]; 
            #endif
            /* vertical configuration */
            #if(Sen_transform == 1)
            Force_val[0] = - (H_Fx_sign-2)*inter_force[H_Fx] + (H_Fx_sign-2)*init_Force[H_Fx];
            Force_val[1] = - (H_Fy_sign-2)*inter_force[H_Fy] + (H_Fy_sign-2)*init_Force[H_Fy];
            Force_val[2] = - (H_Fz_sign-2)*inter_force[H_Fz] + (H_Fz_sign-2)*init_Force[H_Fz];
            #endif

            return_out = 1;
        }
        /* if ID is 2 -> Sensor1 moment (Handle part) */
        else if (recvmsg[4] == 0x02)
        {
            for (int i = 0; i < 3; i++)
            {inter_moment[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 500 - 50;}

            /* FT sensor transform according to its assembly configuration */

            /* original configuration */ 
            #if(Sen_transform == 0)
            Moment_val[0] = -inter_moment[0] + init_Moment[0]; // -y value -> x value
            Moment_val[1] = -inter_moment[1] + init_Moment[1]; // -x value -> y value
            Moment_val[2] = -inter_moment[2] + init_Moment[2]; // -z value -> z value
            #endif
            /* vertical configuration */ 
            #if(Sen_transform == 1)
            Moment_val[0] = - (H_Fx_sign-2)*inter_moment[H_Fx] + (H_Fx_sign-2)*init_Moment[H_Fx];
            Moment_val[1] = - (H_Fy_sign-2)*inter_moment[H_Fy] + (H_Fy_sign-2)*init_Moment[H_Fy];
            Moment_val[2] = - (H_Fz_sign-2)*inter_moment[H_Fz] + (H_Fz_sign-2)*init_Moment[H_Fz];
            #endif

            return_out = 2;
        }
        /* if ID is 3 -> Sensor1 linear acceleration (LAx,LAy,LAz) */
        else if (recvmsg[4] == 0x03)
        {
            for (int i = 0; i < 3; i++)
            {
                inter_posAcc[i] = ((double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]))*2-65535;
                inter_posAcc[i] = (inter_posAcc[i]/16384)*9.81; // m/s^2
            }
            
            /* 45degree rotation */
            Pos_acc_val[0] = -(cos(45*3.141592/180)*inter_posAcc[0] - sin(45*3.141592/180)*inter_posAcc[1]);
            Pos_acc_val[1] = -(sin(45*3.141592/180)*inter_posAcc[0] + cos(45*3.141592/180)*inter_posAcc[1]);
            Pos_acc_val[2] = -inter_posAcc[2];

            return_out = 3;

        }
        /* if ID is 4 -> Sensor1 angular acceleration (AAx,AAy,AAz) */
        else if (recvmsg[4] == 0x04)
        {
            for (int i = 0; i < 3; i++)
            {
                Ang_vel_val[i] = ((double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]))*2-65535;
                Ang_vel_val[i] = (Ang_vel_val[i]*250/32768)*(3.141592/180); // rad/s

                inter_angAcc[i] = (Ang_vel_val[i] - Ang_pvel_val[i])/CAN_sampling; // rad/s^2

                Ang_pvel_val[i] = Ang_vel_val[i];
            }

            /* 45degree rotation */
            Ang_acc_val[0] = -(cos(45*3.141592/180)*inter_angAcc[0] - sin(45*3.141592/180)*inter_angAcc[1]);
            Ang_acc_val[1] = -(sin(45*3.141592/180)*inter_angAcc[0] + cos(45*3.141592/180)*inter_angAcc[1]);
            Ang_acc_val[2] = -inter_angAcc[2];

            return_out = 4;
        }

        /* if ID is 11 -> Sensor2 force (Contact part) 0b */
        else if (recvmsg[4] == 0x0b)
        {
            /* To measure the sensor's sampling time */
            #if SamTime_Cal_OnOff // 
            gettimeofday(&Ctval_start, NULL);  // Get current time
            Ctval_microseconds = Ctval_end.tv_usec - Ctval_start.tv_usec;
            memcpy(&Ctval_end,&Ctval_start,sizeof(Ctval_start));
            #endif

            for (int i = 0; i < 3; i++)
            {Cinter_force[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 100 - 300;}

            /* FT sensor transform according to its assembly configuration */ 

            /* original configuration */ 
            #if(Sen_transform == 0)
            Contact_Force_val[0] = -Cinter_force[0] + init_Contact_Force[0]; 
            Contact_Force_val[1] = -Cinter_force[1] + init_Contact_Force[1]; 
            Contact_Force_val[2] = -Cinter_force[2] + init_Contact_Force[2]; 
            #endif
            /* vertical configuration */ 
            #if(Sen_transform == 1)
            Contact_Force_val[0] = -(C_Fx_sign-2)*Cinter_force[C_Fx] + (C_Fx_sign-2)*init_Contact_Force[C_Fx]; 
            Contact_Force_val[1] = -(C_Fy_sign-2)*Cinter_force[C_Fy] + (C_Fy_sign-2)*init_Contact_Force[C_Fy]; 
            Contact_Force_val[2] = -(C_Fz_sign-2)*Cinter_force[C_Fz] + (C_Fz_sign-2)*init_Contact_Force[C_Fz]; 
            #endif

            return_out = 11;
        }

        /* if ID is 12 -> Sensor2 moment (Contact part) 0c */
        else if (recvmsg[4] == 0x0c) // 
        {
            for (int i = 0; i < 3; i++)
            {Cinter_moment[i] = (double)((int)recvmsg[6 + 2 * i] * 256 + (int)recvmsg[7 + 2 * i]) / 500 - 50;}

            /* FT sensor transform according to its assembly configuration */ 

            /* original configuration */ 
            #if(Sen_transform == 0)
            Contact_Moment_val[0] = -Cinter_moment[0] + init_Contact_Moment[0]; // -y value -> x value
            Contact_Moment_val[1] = -Cinter_moment[1] + init_Contact_Moment[1]; // -x value -> y value
            Contact_Moment_val[2] = -Cinter_moment[2] + init_Contact_Moment[2]; // -z value -> z value
            #endif
            /* vertical configuration */ 
            #if(Sen_transform == 1)
            Contact_Moment_val[0] = -(C_Fx_sign-2)*Cinter_moment[C_Fx] + (C_Fx_sign-2)*init_Contact_Moment[C_Fx]; 
            Contact_Moment_val[1] = -(C_Fy_sign-2)*Cinter_moment[C_Fy] + (C_Fy_sign-2)*init_Contact_Moment[C_Fy]; 
            Contact_Moment_val[2] = -(C_Fz_sign-2)*Cinter_moment[C_Fz] + (C_Fz_sign-2)*init_Contact_Moment[C_Fz]; 
            #endif

            return_out = 12;
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
bool Yoon_FT_sensor::Sensor_value_init()
{
    while(sensor_init_counter < init_average_num)
    {
        for(int i=0;i<3;i++)
        {
            if(sensor_init_counter != 0)
            {
                init_Force[i] += inter_force[i]/init_average_num;
                init_Moment[i] += inter_moment[i]/init_average_num;
    
                init_Contact_Force[i] += Cinter_force[i]/init_average_num;
                init_Contact_Moment[i] += Cinter_moment[i]/init_average_num;
            }
            else
            {
                init_Force[i] = 0;
                init_Moment[i] = 0;
    
                init_Contact_Force[i] = 0;
                init_Contact_Moment[i] = 0;
            }
        }

        sensor_init_counter++;
    }

    if(sensor_init_counter < init_average_num) return false;
    else return true;
}

void Yoon_FT_sensor::errhandle(const char *errmsg){ // for FT_communication
  fputs(errmsg, stderr);
  fputc('\n', stderr);
  close(clnt_sock);
  exit(1);
}