#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "Yoon_UR10e_cmd.h"
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"

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
#define HG_OnOff 0 // 0: Hand-guiding mode de-activation, 1: Hand-guiding mode activation

Yoon_UART Yuart("/dev/ttyACM0",(int)115200); // UART instance (Device_location, Baudrate)
char buffer[1024] = {0};
bool pre_button_val = false;
bool button_val = false;

/* Yaml file load */
std::ifstream fin1(NRS_Record_Printing_loc);
std::ifstream fin2(NRS_Fcon_desired_loc);

YAML::Node NRS_recording = YAML::Load(fin1);
YAML::Node NRS_Fcon_desired = YAML::Load(fin2);

void catch_signal(int sig)
{
    Yuart.YUART_terminate();
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Yoon_Hbutton_cmd");
    ros::NodeHandle nh;

    /* ROS instance generation start */

    // Publisher instance
    ros::Publisher yoon_mode_pub = nh.advertise<std_msgs::UInt16>("Yoon_UR10e_mode",20);
    ros::Publisher PbNum_command_pub = nh.advertise<std_msgs::UInt16>("Yoon_PbNum_cmd",20);

    // Msg instance
    std_msgs::UInt16 yoon_mode_msg;
    std_msgs::UInt32 PbNum_command_msg;
    /* ROS instance generation end */

    /* Phase status statement start */
    char current_phase[32] = "Unknown";
    char phase0[] = "Calibration phase";
    char phase1[] = "Teaching(Discrete) phase";
    // char phase2[] = "Teaching(Continuous) phase";

    // point 저장, path generation, iteration 횟수 지정, path execution
    /* Mode status statement start */
    char current_mode[32]= "Stanby mode";
    char mode0[] = "Stanby mode";
    char mode1[] = "Guiding mode";
    char mode2[] = "Playback ready";
    char mode3[] = "Playback execution";
    char mode4[] = "Path generation done";
    char mode5[] = "VR teaching mode";
    char mode6[] = "VR calibration mode";
    char mode7[] = "VR calibration was terminated";
    char modeErr1[] = "Wrong iter number(1~9)";
    /* Mode status statement end */

    /* Handle UART signal comparison start */
    char Hmode0[1024] = "MODE 0"; // Defualt value
    char Hmode1[1024] = "MODE 1"; // Mode change (Stanby mode, guiding mode)
    char Hmode2[1024] = "MODE 2"; // Way point save
    char Hmode3[1024] = "MODE 3"; // Playback start (Ready , execution)
    char Hmode4[1024] = "MODE 4"; // Iteration number
    char Hmode5[1024] = "MODE 5"; // Trajectory generation
    /* Handle UART signal comparison end */

    int Phase_selector = 0; // Calibration phase: 0, Teaching(Discrete) phase: 1, Teaching(Continuous) phase:2

    int Mode_val = 0; // Current mode information
    int Pre_Mode_val = 0; // Current previous information


    bool guiding_mode = false; // false: stanby mode, true: guiding mode
    bool VRTeaching_mode = false; // false: stanby mode, true: VR teaching mode
    // bool VRConTeaching_mode = false;
    int point_counter = 0; // Saved way points
    int iter_num = 0; // Iteration number
    int PB_exe_counter = 0; // 1: ready status, 2: execution

    signal(SIGTERM, catch_signal);// Termination
	signal(SIGINT, catch_signal);// Active

    ros::Rate loop_rate(100);

    // point 저장, path generation, iteration 횟수 지정, path execution
    /* Phase selcection start */
    Phase_sel_window:
    std::cout << "\n Select the handle phase (0:Calibration, 1:Teaching(Descrete), 2:Teaching(Continuous)): ";
    std::cin >> Phase_selector;
    if(Phase_selector == 0){memcpy(current_phase,phase0,sizeof(phase0));}
    else if(Phase_selector == 1){memcpy(current_phase,phase1,sizeof(phase1));}
    // else if(Phase_selector == 2){memcpy(current_phase,phase2,sizeof(phase2));}
    else
    {
        printf("Wrong phase was selected");
        goto Phase_sel_window;
    }
    /* Phase selcection end */

    while(1)
    {
        /* Real time monitoring start */
        printf("\n============================================================\n");
        printf("************************************************************\n");
        printf("NOTE : Before start the main node, the screen of handle \n");
        printf("       must be 'STANBY MODE' \n");
        printf("************************************************************\n");
        printf("Current phase: %s, Current mode: %s \n",current_phase,current_mode);
        printf("Selected way points: %d, Iteration number(1~9): %d \n",point_counter,iter_num);
        printf("Mode value: %d \n",Mode_val);
        printf("\n============================================================\n");
        /* Real time monitoring end */

        if(Yuart.YUART_start(buffer))
        {
            /* Handle button data aqusition start */
            if(strcmp(buffer,Hmode0)==0) Mode_val=0;
            else if(strcmp(buffer,Hmode1)==0) Mode_val=1; // RB (Right Button)
            else if(strcmp(buffer,Hmode2)==0) Mode_val=2; // LB (Left Button)
            else if(strcmp(buffer,Hmode3)==0) Mode_val=3; // MLB (Middle Left Button)
            else if(strcmp(buffer,Hmode4)==0) Mode_val=4; // MRB (Middle Right Button)
            else if(strcmp(buffer,Hmode5)==0) Mode_val=5; // LB (Left Button)
            /* Handle button data aqusition end */

            /* If Mode_val: 1 - Guiding(Cali. phase), Teaching(Discrete)(Teac. phase) */
            if((Pre_Mode_val == 0) && (Mode_val == 1))
            {
                // - Calibration phase -
                if(Phase_selector == 0)
                {
                    if(guiding_mode == false) // change to guiding mode
                    {
                        guiding_mode = true;
                        memcpy(current_mode,mode6,sizeof(mode6));

                        #if HG_OnOff
                        yoon_mode_msg.data = Hand_guiding_mode_cmd;
                        yoon_mode_pub.publish(yoon_mode_msg);
                        ros::spinOnce();
                        #endif
                    }
                    else // change to stanby mode
                    {
                        guiding_mode = false;
                        memcpy(current_mode,mode0,sizeof(mode0));

                        // Stanby mode message publish
                        yoon_mode_msg.data = Motion_stop_cmd;
                        yoon_mode_pub.publish(yoon_mode_msg);
                        ros::spinOnce();
                    }
                }
                // - Teaching(Discrete) phase -
                else if(Phase_selector == 1)
                {
                    if(VRTeaching_mode == false) // change to VR teaching mode
                    {
                        VRTeaching_mode = true;
                        memcpy(current_mode,mode5,sizeof(mode5));
                    }
                    else // change to stanby mode
                    {
                        VRTeaching_mode = false;
                        memcpy(current_mode,mode0,sizeof(mode0));

                        if(point_counter != 0)
                        {
                            // Calibration point save termination message publish
                            yoon_mode_msg.data = VRTeac_recording_save;
                            yoon_mode_pub.publish(yoon_mode_msg);
                            ros::spinOnce();
                        }
                    }
                }

                // // - Teaching(Continuous) phase -
                // else if(Phase_selector == 2)
                // {
                //     if(VRConTeaching_mode == false) // change to VR Continuous teaching mode
                //     {
                //         VRConTeaching_mode = true;
                //         memcpy(current_mode,mode5,sizeof(mode5));

                //     }
                //     else // change to stanby mode
                //     {
                //         VRConTeaching_mode = false;
                //         memcpy(current_mode,mode0,sizeof(mode0));

                //         if(point_counter != 0)
                //         {
                //             // Calibration point save termination message publish
                //             // yoon_mode_msg.data = VRConTeac_recording_save;

                //             // yoon_mode_pub.publish(yoon_mode_msg);
                //             ros::spinOnce();
                //         }
                //     }
                // }

            }
            /* If Mode_val: 2 - Nothing(Cali. phase), Way point save(Teac. phase) */
            if((Pre_Mode_val == 0) && (Mode_val == 2))
            {
                // - Calibration phase -
                if(Phase_selector == 0){}
                // - Teaching phase -
                else if(Phase_selector == 1)
                {
                    yoon_mode_msg.data = VRTeac_reording_start;
                    yoon_mode_pub.publish(yoon_mode_msg);
                    ros::spinOnce();
                    point_counter ++;
                }


            }

            /* If Mode_val: 5 - Nothing(Cali. phase), Generate the trajectory(Teac. phase) */
            if((Pre_Mode_val == 0) && (Mode_val == 5))
            {
                // - Calibration phase -
                if(Phase_selector == 0){}
                // - Teaching phase -
                else if(Phase_selector == 1)
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

                    /*** STEP1 : Load the waypoints ***/
                    // Tot_txt_load txt_loader("/home/gene/catkin_ws/src/rtde_handarm/src/Descre_P_recording.txt");
                    auto YamlDescre_P_recording_path = NRS_recording["Descre_P_recording"].as<std::string>();
                    char* Descre_P_recording_path = const_cast<char*>(YamlDescre_P_recording_path.c_str());
                    Tot_txt_load txt_loader(Descre_P_recording_path);
                    Eigen::MatrixXd Decr_RD_points;
                    txt_loader.Tot_data_read(Decr_RD_points);

                    std::cout << Decr_RD_points << std::endl;

                    /*** STEP2 : Path generation ***/
                    Yoon_path Descr_RD_blending;
                    /* trajectory planning && recording to text */

                    /* Open the text file */
                    // FILE* Hand_G_recording = fopen("/home/gene/catkin_ws/src/rtde_handarm/src/Hand_G_recording.txt","wt");
                    auto Hand_G_recording_path = NRS_recording["Hand_G_recording"].as<std::string>();
                    FILE* Hand_G_recording = fopen(Hand_G_recording_path.c_str(),"wt");

                    /**** Points to path profile ****/

                    /* 1) Contact points path profile with force */
                    int MPoint_num = Decr_RD_points.rows(); // number of points (actal num : MPoint_num)
                    double MTar_vel[MPoint_num-1] = {0,};
                    double MWaiting_time[MPoint_num-1] = {0,};
                    double PPB_des_force[MPoint_num-1] = {0,};


                    // Desired velocity & force update
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
                    memcpy(current_mode,mode4,sizeof(mode4));
                }
            }
            /* If Mode_val: 4 - Cali. point save(Cali. phase), Iteration number setting(Teac. phase) */
            if((Pre_Mode_val == 0) && (Mode_val == 4))
            {
                // - Calibration phase -
                if(Phase_selector == 0)
                {
                    yoon_mode_msg.data = VRCali_reording_start;
                    yoon_mode_pub.publish(yoon_mode_msg);
                    ros::spinOnce();
                    point_counter ++;
                }
                // - Teaching(Discrete) phase -
                else if(Phase_selector == 1)
                {
                    if(PB_exe_counter == 0)
                    {
                        if(iter_num<9) iter_num ++;
                        else iter_num = 0;
                    }
                }
            }
            /* If Mode_val: 3 - Cali save termination(Cali. phase), Playback execution(Teac. phase) */
            if((Pre_Mode_val == 0) && (Mode_val == 3))
            {
                // - Calibration phase -
                if(Phase_selector == 0)
                {
                    if(point_counter != 0)
                    {
                        // Calibration point save termination message publish
                        yoon_mode_msg.data = VRCali_recording_save;
                        yoon_mode_pub.publish(yoon_mode_msg);
                        ros::spinOnce();

                        memcpy(current_mode,mode7,sizeof(mode7));
                        point_counter = 0; // selected way point num init
                    }
                }
                // - Teaching(Discrete) phase -
                else if(Phase_selector == 1)
                {
                    if(iter_num > 0 && iter_num < 10)
                    {
                        if(PB_exe_counter == 0)
                        {
                            // Playback iteration number publish
                            PbNum_command_msg.data = iter_num;
                            PbNum_command_pub.publish(PbNum_command_msg);
                            ros::spinOnce();
                            memcpy(current_mode,mode2,sizeof(mode2));
                            PB_exe_counter++;
                        }
                        else if(PB_exe_counter == 1)
                        {
                            /*** STEP1 : Playback start ***/
                            yoon_mode_msg.data = Playback_mode_cmd;
                            yoon_mode_pub.publish(yoon_mode_msg);
                            ros::spinOnce();
                            memcpy(current_mode,mode3,sizeof(mode3));
                            iter_num = 0;
                            PB_exe_counter = 0;
                        }
                        else PB_exe_counter = 0;


                    }
                    else memcpy(current_mode,modeErr1,sizeof(modeErr1));
                    }
            }
            Pre_Mode_val = Mode_val; // mode update to past
        }

        // ros::spinOnce();
    }
    exit(0);
    Yuart.YUART_terminate();
    return 0;

}
