#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "Yoon_UR10e_cmd.h"
#include "ros/ros.h"

// For ROS message
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"

// For ROS service - AIDIN GUI SERVER
#include <std_srvs/Empty.h>

// For trajectory generation
#include "Text_loader.h"
#include "Yoon_path.h"

// For UART communication
#include "Yoon_communi.h"

// Yaml file headers
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "NRS_yaml_location.h"

#define DOF 6
#define PI 3.141592

Yoon_UART Yuart("/dev/ttyACM0",(int)115200); // UART instance (Device_location, Baudrate)
char buffer[1024] = {0};

/* Yaml file load */
std::ifstream fin1(NRS_Record_Printing_loc);
std::ifstream fin2(NRS_Fcon_desired_loc);

YAML::Node NRS_recording = YAML::Load(fin1);
YAML::Node NRS_Fcon_desired = YAML::Load(fin2);

bool SRV1_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Empty1 서비스 호출됨.");
    return true;  // 성공적으로 처리되었음을 나타냄
}

bool SRV3_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Empty3 서비스 호출됨.");
    return true;  // 성공적으로 처리되었음을 나타냄
}

bool SRV4_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Empty4 서비스 호출됨.");
    return true;  // 성공적으로 처리되었음을 나타냄
}


void catch_signal(int sig)
{
    if(sig == 0){printf("Desired posture is under 4 \n");}
    Yuart.YUART_terminate();
    printf("Program was terminated !! \n");
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Yoon_Hbutton_cmd");
    ros::NodeHandle nh;

    /**** ROS instance generation start ****/

    /* ROS Message */
    ros::Publisher yoon_mode_pub = nh.advertise<std_msgs::UInt16>("Yoon_UR10e_mode",20);
    ros::Publisher PbNum_command_pub = nh.advertise<std_msgs::UInt16>("Yoon_PbNum_cmd",20); 

    std_msgs::UInt16 yoon_mode_msg;
    std_msgs::UInt32 PbNum_command_msg;

    /* ROS Service */
    ros::ServiceServer Aidin_gui_srv1 = nh.advertiseService("teaching_mode",SRV1_Handle);
    ros::ServiceServer Aidin_gui_srv3 = nh.advertiseService("save_waypoint",SRV3_Handle);
    ros::ServiceServer Aidin_gui_srv4 = nh.advertiseService("trajectory_generation",SRV4_Handle);

    /**** ROS instance generation end ****/


    char current_status[32]="Stanby mode";
    char mode0[] = "Stanby mode";
    char mode1[] = "Guiding mode";
    char mode2[] = "Playback ready";
    char mode3[] = "Playback execution";
    char mode4[] = "Path generation done";
    char modeErr1[] = "Wrong iter number(1~9)";


    char Hmode0[1024] = "MODE 0"; // Defualt value
    char Hmode1[1024] = "MODE 1"; // Mode change (Stanby mode, guiding mode)
    char Hmode2[1024] = "MODE 2"; // Way point save
    char Hmode3[1024] = "MODE 3"; // Playback start (Ready , execution)
    char Hmode4[1024] = "MODE 4"; // Iteration number
    char Hmode5[1024] = "MODE 5"; // Trajectory generation

    int Mode_val = 0;
    int Pre_Mode_val = 0;

    bool guiding_mode = false; // false: stanby mode, true: guiding mode
    int point_counter = 0; // Saved way points
    int iter_num = 0; // Iteration number
    int PB_exe_counter = 0; // 1: ready status, 2: execution

    signal(SIGTERM, catch_signal);// Termination
	signal(SIGINT, catch_signal);// Active

    ros::Rate loop_rate(100);
    while(1)
    {
        /** Real time monitoring **/
        printf("\n============================================================\n");
        printf("************************************************************\n");
        printf("NOTE : Before start the main node, the screen of handle \n");
        printf("       must be 'STANBY MODE' \n");
        printf("************************************************************\n");
        printf("Current status: %s \n",current_status);
        printf("Selected way points: %d, Iteration number(1~9): %d \n",point_counter,iter_num);
        printf("Mode value: %d \n",Mode_val);
        printf("\n============================================================\n");


        if(Yuart.YUART_start(buffer)) 
        {
            // printf("%s\n",buffer);

            /* Raw mode aquisition */
            if(strcmp(buffer,Hmode0)==0) Mode_val=0;
            else if(strcmp(buffer,Hmode1)==0) Mode_val=1;
            else if(strcmp(buffer,Hmode2)==0) Mode_val=2;
            else if(strcmp(buffer,Hmode3)==0) Mode_val=3;
            else if(strcmp(buffer,Hmode4)==0) Mode_val=4;
            else if(strcmp(buffer,Hmode5)==0) Mode_val=5;


            if((Pre_Mode_val == 0) && (Mode_val == 1)) // Mode change
            {
                if(guiding_mode == false) // change to guiding mode
                {
                    guiding_mode = true;
                    memcpy(current_status,mode1,sizeof(mode1));

                    yoon_mode_msg.data = Hand_guiding_mode_cmd;
                    yoon_mode_pub.publish(yoon_mode_msg);
                    ros::spinOnce();
                }
                else // change to stanby mode
                {
                    guiding_mode = false;
                    memcpy(current_status,mode0,sizeof(mode0));

                    // Stanby mode message publish 
                    yoon_mode_msg.data = Motion_stop_cmd;
                    yoon_mode_pub.publish(yoon_mode_msg);
                    ros::spinOnce();

                    if(point_counter != 0)
                    {
                        // Way point save termination message publish
                        yoon_mode_msg.data = Descrete_recording_save;
                        yoon_mode_pub.publish(yoon_mode_msg);
                        ros::spinOnce();
                    }

                }
                
            }
            if((Pre_Mode_val == 0) && (Mode_val == 2)) // Way point save
            {
                yoon_mode_msg.data = Descrete_reording_start;
                yoon_mode_pub.publish(yoon_mode_msg);
                ros::spinOnce();
                point_counter ++;
            }
            if((Pre_Mode_val == 0) && (Mode_val == 5)) // Generate the trajectory
            {
                /*** STEP0 : Load the desired values from YAML file ***/

                /* Desired force setting */
                double Desired_Fx = NRS_Fcon_desired["AAC_Des_Force"]["Fx"].as<double>();
                double Desired_Fy = NRS_Fcon_desired["AAC_Des_Force"]["Fy"].as<double>();
                double Desired_Fz = NRS_Fcon_desired["AAC_Des_Force"]["Fz"].as<double>();
                
                /* Desired motion velocity setting */
                double Star2Cont_vel = NRS_Fcon_desired["AAC_Des_velocity"]["Start2Contact"].as<double>();
                double Cont2Term_vel = NRS_Fcon_desired["AAC_Des_velocity"]["Contact2Termin"].as<double>();
                double MMotion_vel = NRS_Fcon_desired["AAC_Des_velocity"]["MMotion_vel"].as<double>();

                /* Desired defualt waiting time setting */
                double Des_Waiting_time = NRS_Fcon_desired["AAC_Des_WaitingT"].as<double>();


                /*** STEP1 : Load the waypoints ***/
                // Tot_txt_load txt_loader("/home/gene/catkin_ws/src/rtde_handarm/src/Descre_P_recording.txt");
                auto YamlDescre_P_recording_path = NRS_recording["Descre_P_recording"].as<std::string>();
                char* Descre_P_recording_path = const_cast<char*>(YamlDescre_P_recording_path.c_str());
                Tot_txt_load txt_loader(Descre_P_recording_path);
                Eigen::MatrixXd Decr_RD_points, Decr_RD_points_tem;
                txt_loader.Tot_data_read(Decr_RD_points_tem);

                std::cout << Decr_RD_points_tem << std::endl;

                /*** STEP2 : Path generation ***/
                Yoon_path Descr_RD_blending;
                /* trajectory planning && recording to text */ 

                /* Open the text file */
                // FILE* Hand_G_recording = fopen("/home/gene/catkin_ws/src/rtde_handarm/src/Hand_G_recording.txt","wt");
                auto Hand_G_recording_path = NRS_recording["Hand_G_recording"].as<std::string>();
                FILE* Hand_G_recording = fopen(Hand_G_recording_path.c_str(),"wt");
                
                /**** Points to path profile ****/

                /* 1) Contact points path profile with force */
                int MPoint_num = Decr_RD_points_tem.rows(); // number of points (actal num : MPoint_num)

                /* Desired velocity & force update */
                if(MPoint_num<4) 
                {
                    /* # of points is under 3 */
                    if(MPoint_num < 3) {catch_signal(0);}
                    else
                    {
                        Decr_RD_points = Eigen::MatrixXd::Zero(4,Decr_RD_points_tem.cols());
                        Decr_RD_points.row(0) = Decr_RD_points_tem.row(0);
                        Decr_RD_points.row(1) = Decr_RD_points_tem.row(1);
                        Decr_RD_points.row(2) = Decr_RD_points_tem.row(1);
                        Decr_RD_points.row(3) = Decr_RD_points_tem.row(2);

                        MPoint_num = 4;
                    }
                }
                else {Decr_RD_points = Decr_RD_points_tem;}

                double MTar_vel[MPoint_num-1] = {0,};
                double MWaiting_time[MPoint_num-1] = {0,};
                double PPB_des_force[MPoint_num-1] = {0,};

                for(int i=0;i<MPoint_num-1;i++) // Point number : MPoint_num
                {
                    if(i == 0) 
                    {
                        MTar_vel[i] = Star2Cont_vel;
                        PPB_des_force[i] = (double)0.0;
                    }
                    else if(i == MPoint_num-2) 
                    {
                        MTar_vel[i] = Cont2Term_vel;
                        PPB_des_force[i] = (double)0.0;
                    }
                    else 
                    {
                        MTar_vel[i] = MMotion_vel;
                        PPB_des_force[i] = Desired_Fz;
                    }
                }

                std::cout << Decr_RD_points << std::endl;
                Descr_RD_blending.Defualt_WaitingT = Des_Waiting_time;
                if(Descr_RD_blending.PPB_path_init(Decr_RD_points.block(0,0,MPoint_num,6),MTar_vel,PPB_des_force,MWaiting_time,MPoint_num))
                {
                    double path_out[7] = {0,};
                    while(Descr_RD_blending.PPB_path_exe(path_out))
                    {
                        fprintf(Hand_G_recording,"%10f %10f %10f %10f %10f %10f %10f %10f %10f\n",
                        path_out[0], path_out[1], path_out[2], path_out[3], path_out[4], path_out[5],
                        Desired_Fx, Desired_Fy, path_out[6]);
                    }
                }

                fclose(Hand_G_recording);

                point_counter = 0; // selected way point num init
                memcpy(current_status,mode4,sizeof(mode4));
            }
            if((Pre_Mode_val == 0) && (Mode_val == 4)) // Iteration number setting
            {
                if(PB_exe_counter == 0)
                {
                    if(iter_num<9) iter_num ++;
                    else iter_num = 0;
                }
            }
            if((Pre_Mode_val == 0) && (Mode_val == 3)) // Playback execution
            {   
                if(iter_num > 0 && iter_num < 10)
                {
                    if(PB_exe_counter == 0)
                    {
                        // Playback iteration number publish
                        PbNum_command_msg.data = iter_num;
                        PbNum_command_pub.publish(PbNum_command_msg);
                        ros::spinOnce();
                        memcpy(current_status,mode2,sizeof(mode2));
                        PB_exe_counter++;
                    }
                    else if(PB_exe_counter == 1)
                    {
                        /*** STEP1 : Playback start ***/
                        yoon_mode_msg.data = Playback_mode_cmd;
                        yoon_mode_pub.publish(yoon_mode_msg);
                        ros::spinOnce();
                        memcpy(current_status,mode3,sizeof(mode3));
                        iter_num = 0;
                        PB_exe_counter = 0;
                    }
                    else PB_exe_counter = 0;

                    
                }
                else memcpy(current_status,modeErr1,sizeof(modeErr1));

            }


            Pre_Mode_val = Mode_val; // mode update to past

        }




        // ros::spinOnce();
    }
    exit(0);
    Yuart.YUART_terminate();
    return 0;

}