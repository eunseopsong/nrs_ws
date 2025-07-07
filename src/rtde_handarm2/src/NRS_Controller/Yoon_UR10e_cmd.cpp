#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "Yoon_UR10e_cmd.h"
#include "rclcpp/rclcpp.hpp" //// #include "ros/ros.h"

#include "std_msgs/msg/multi_array_layout.hpp"    //// #include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/msg/multi_array_dimension.hpp" //// #include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/msg/float64_multi_array.hpp"   //// #include "std_msgs/Float64MultiArray.h"
#include "std_msgs/msg/float64.hpp"               //// #include "std_msgs/Float64.h"
#include "std_msgs/msg/u_int16.hpp"               //// #include "std_msgs/UInt16.h"
#include "std_msgs/msg/u_int32.hpp"               //// #include "std_msgs/UInt32.h"

// For trajectory generation
#include "Text_loader.h"
#include "Yoon_path.h"

// Yaml file headers
#include <fstream>
#include <yaml-cpp/yaml.h>

#define DOF 6
#define PI 3.141592

/* Yaml file load start */
std::ifstream fin2("/home/nrsur10/catkin_ws/src/rtde_handarm/NRS_yaml/NRS_Record_Printing.yaml");
YAML::Node NRS_recording = YAML::Load(fin2);
/* Yaml file load end */

void catch_signal(int sig)
{
    exit(1);
}

int main(int argc, char *argv[])
{
    //// ros::init(argc,argv,"Yoon_UR10e_cmd");
    //// ros::NodeHandle nh;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yoon_ur10e_cmd");

    //// ros::Publisher yoon_mode_pub = nh.advertise<std_msgs::UInt16>("Yoon_UR10e_mode",20);
    //// ros::Publisher joint_command_pub = nh.advertise<std_msgs::Float64MultiArray>("yoon_UR10e_joint_cmd",20);
    //// ros::Publisher posture_command_pub = nh.advertise<std_msgs::Float64MultiArray>("yoon_UR10e_EEposture_cmd",20);
    //// ros::Publisher PbNum_command_pub = nh.advertise<std_msgs::UInt16>("Yoon_PbNum_cmd",20); 
    auto yoon_mode_pub = node->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 20);
    auto joint_command_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("yoon_UR10e_joint_cmd", 20);
    auto posture_command_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("yoon_UR10e_EEposture_cmd", 20);
    auto PbNum_command_pub = node->create_publisher<std_msgs::msg::UInt16>("Yoon_PbNum_cmd", 20);

    std_msgs::msg::Float64MultiArray joint_command_msg;
    std_msgs::msg::Float64MultiArray posture_command_msg;
    std_msgs::msg::UInt16 yoon_mode_msg;
    std_msgs::msg::UInt32 PbNum_command_msg;

    uint16_t control_mode = 0;
    double joint_command[2]={0,}; // which joint(1~6), cmd angle(relative, unit: rad)
    double posture_command[6+1]={0,};

    // Status message setting
    char current_status[32];
    char mode0[] = "Motion stop";
    char mode1[] = "-------------";
    char mode2[] = "Joint control(cmd1)";
    char mode3[] = "EE Posture control(cmd1)";
    char mode4[] = "Hand-guiding(cmd2)";
    char mode5[] = "Data recording on";
    char mode6[] = "Data recording off";
    char mode7[] = "Posture playback";

    signal(SIGTERM, catch_signal);// Termination
	signal(SIGINT, catch_signal);// Active

    //// ros::Rate loop_rate(100);
    rclcpp::Rate loop_rate(100);  // 100 Hz

    while(1)
    {
        if(control_mode==0) memcpy(current_status,mode0,sizeof(mode0));
        else if(control_mode==1) memcpy(current_status,mode1,sizeof(mode1));
        else if(control_mode==2) memcpy(current_status,mode2,sizeof(mode2));
        else if(control_mode==3) memcpy(current_status,mode3,sizeof(mode3));
        else if(control_mode==4) memcpy(current_status,mode4,sizeof(mode4));
        else if(control_mode==5) memcpy(current_status,mode5,sizeof(mode5));
        else if(control_mode==6) memcpy(current_status,mode6,sizeof(mode6));
        else if(control_mode==7) memcpy(current_status,mode7,sizeof(mode7));
        else memcpy(current_status,mode0,sizeof(mode0));

        std::cout << "\n Last command for control mode : " << current_status <<std::endl;
        std::cout << "\n [--- Choose the mode ---]"<<std::endl;
        std::cout << " Mode1: ----------------------  , Mode2: Joint control(cmd1)"<<std::endl;
        std::cout << " Mode3: EE Posture control(cmd1), Mode4: Hand-guiding(cmd2)"<<std::endl;
        std::cout << " Mode5: Data recording mode     , Mode6: Generate the path in cmd_node"<<std::endl;
        std::cout << " Mode7: Posture playback        , Mode0: Motion stop"<<std::endl;
        std::cin >>control_mode; // x,y,vel

        if(control_mode == -1)
        {
            break;
        }

        std::cout << "The mode "<< control_mode<<" selected"<<std::endl;
        printf("\n");

        if(control_mode == 1) // posture control mode
        {
            // std::cout << "posture command(3:pos, 3:ori, 1:Travel time)?? :";
            // std::cin >> posture_command[0]>>posture_command[1]>>posture_command[2]
            // >>posture_command[3]>>posture_command[4]>>posture_command[5]>>posture_command[6];

            // yoon_mode_msg.data = control_mode;
            // posture_command_msg.data.clear();
            // for(int i=0;i<DOF+1;i++)
            // {
            //     posture_command_msg.data.push_back(posture_command[i]);
            // }
            // yoon_mode_pub.publish(yoon_mode_msg);
            // posture_command_pub.publish(posture_command_msg);

            // printf("Posture -> ");
            // for(int i=0;i<DOF;i++)
            // {
            //     printf("%d :%f, ",i,posture_command[i]);
            // }
            // printf("\n");
            // printf("Time -> %f second",posture_command[6]);

        }
        else if(control_mode == 2) // joint control mode 
        {
            std::cout << "Select the joint to move(1~6) :";
            std::cin >> joint_command[0];
            std::cout << "Set the relative target angle to move (Unit: Degree !!) :";
            std::cin >> joint_command[1];

            joint_command[1] = joint_command[1]*(PI/180);

            yoon_mode_msg.data = Joint_control_mode_cmd;
            joint_command_msg.data.clear();

            for(int i=0;i<2;i++)
            {
                joint_command_msg.data.push_back(joint_command[i]);
            }

            // Do not change the publishing sequence
            joint_command_pub->publish(joint_command_msg);
            yoon_mode_pub->publish(yoon_mode_msg);

            printf("\nSelected joint: %1.0f, Target relative joint angle: %4f \n", joint_command[0],joint_command[1]);

            rclcpp::spin_some(node); //// ros::spinOnce();
        }

        else if(control_mode == 3) // EE_Posture_control
        {
            yoon_mode_msg.data = EE_Posture_control_mode_cmd;
            yoon_mode_pub->publish(yoon_mode_msg);
            rclcpp::spin_some(node); //// ros::spinOnce();
        }
        else if(control_mode == 4) // Hand_guiding
        {
            yoon_mode_msg.data = Hand_guiding_mode_cmd;
            yoon_mode_pub->publish(yoon_mode_msg);
            rclcpp::spin_some(node); //// ros::spinOnce();
        }
        else if(control_mode == 5) // Data recording mode
        {
            int recording_mode = -1;
            std::cout << "\n[ -- Recording mode selection -- ]" << std::endl;
            std::cout << "1 : Continuous reording start, 2 : Continuous reording end" << std::endl;
            std::cout << "3 : Descrete point rececording start " << std::endl;
            std::cout << "0 : quit" << std::endl;
            std::cin >> recording_mode;

            if(recording_mode == 1)
            {
                yoon_mode_msg.data = Continuous_reording_start;
                yoon_mode_pub->publish(yoon_mode_msg);
                rclcpp::spin_some(node); //// ros::spinOnce();
                std::cout << "To terminate the recording press '2'" << std::endl;
                std::cin >> recording_mode;
                if(recording_mode == 2)
                {
                    yoon_mode_msg.data = Continusous_recording_end;
                    yoon_mode_pub->publish(yoon_mode_msg);
                    rclcpp::spin_some(node); //// ros::spinOnce();
                }
            }
            else if(recording_mode == 2)
            {
                yoon_mode_msg.data = Continusous_recording_end;
                yoon_mode_pub->publish(yoon_mode_msg);
                rclcpp::spin_some(node);     //// ros::spinOnce();
            }
            else if(recording_mode == 3) 
            {
                int point_counter = 0;
                while(1)
                {
                    std::cout << "\nSelect the point("<< point_counter <<"),(Select: 1, Terminate the recording & Save the recorded data: 2) "<< std::endl;
                    std::cout << "First & Last selected point is starting & ending point with Non-contact"<< std::endl;
                    std::cin >> recording_mode;
                    if(recording_mode == 1) // Point save
                    {
                        // std::cout << Desired_XYZ(0), Desired_XYZ(1), Desired_XYZ(2), Desired_RPY(0), Desired_RPY(1), Desired_RPY(2) << std::endl;
                        yoon_mode_msg.data = Descrete_reording_start;
                        yoon_mode_pub->publish(yoon_mode_msg);
                        rclcpp::spin_some(node); //// ros::spinOnce();
                        point_counter ++;
                    }
                    else if(recording_mode == 2) // Terminate the point save & Save the selected point to text file
                    {
                        yoon_mode_msg.data = Descrete_recording_save;
                        yoon_mode_pub->publish(yoon_mode_msg);
                        rclcpp::spin_some(node); //// ros::spinOnce();

                        printf("Point recording was terminated \n");
                        printf("& The recorded point was saved to text file 'Descre_P_recording.txt'\n");
                    }

                    if (recording_mode==2)
                    {
                        break;
                    }

                    
                }

            }

        }
        else if(control_mode == 6) // Select the iteration number & Generate the path in cmd node
        {
            /*** STEP1 : Load the waypoints ***/
            // Tot_txt_load txt_loader("/home/gene/catkin_ws/src/rtde_handarm/src/Descre_P_recording.txt");
            auto YamlDescre_P_recording_path = NRS_recording["Descre_P_recording"].as<std::string>();
            char* Descre_P_recording_path = const_cast<char*>(YamlDescre_P_recording_path.c_str());
            Tot_txt_load txt_loader(Descre_P_recording_path);
            Eigen::MatrixXd Decr_RD_points;
            txt_loader.Tot_data_read(Decr_RD_points);

            std::cout << Decr_RD_points << std::endl;

            /*** STEP2 : Path generation ***/
            #if 1
            Yoon_path Descr_RD_blending1,Descr_RD_blending2,Descr_RD_blending3;
            /* trajectory planning && recording to text */ 

            /* Open the text file */
            // FILE* Hand_G_recording = fopen("/home/gene/catkin_ws/src/rtde_handarm/src/Hand_G_recording.txt","wt");
            auto Hand_G_recording_path = NRS_recording["Hand_G_recording"].as<std::string>();
            FILE* Hand_G_recording = fopen(Hand_G_recording_path.c_str(),"wt");
            
            /**** Points to path profile ****/

            /* 1) Start to Contact path profile */
            double Linear_travel_time = 5.0;
            double Tar_pos[6] = {Decr_RD_points(1,0),Decr_RD_points(1,1),Decr_RD_points(1,2),
            Decr_RD_points(1,3),Decr_RD_points(1,4),Decr_RD_points(1,5)};
            double Init_pos[6] = {Decr_RD_points(0,0),Decr_RD_points(0,1),Decr_RD_points(0,2),
            Decr_RD_points(0,3),Decr_RD_points(0,4),Decr_RD_points(0,5)};

            if(Descr_RD_blending1.PTP_6D_path_init(Init_pos, Tar_pos, Linear_travel_time))
            {
                double path_out[6] = {0,};
                while(Descr_RD_blending1.PTP_6D_path_exe(path_out))
                {
                    fprintf(Hand_G_recording,"%10f %10f %10f %10f %10f %10f %10f %10f %10f \n",
                    path_out[0], path_out[1], path_out[2], path_out[3], path_out[4], path_out[5],
                    0.0, 0.0, 0.0);
                }
            }

            /* 2) Contact points path profile */
            int MPoint_num = Decr_RD_points.rows(); // number of points
            double MTar_vel[MPoint_num-1 -2] = {0,}; // -2 is to delete the start and end point
            double MWaiting_time[MPoint_num-1 -2] = {0,}; // -2 is to delete the start and end point
            double MMotion_vel = 0.02; // m/s

            for(int i=0;i<MPoint_num-1 -2;i++) {MTar_vel[i] = MMotion_vel;}// -2 is to delete the start and end point

            if(Descr_RD_blending2.MultiP_6D_path_init(Decr_RD_points.block(1,0,MPoint_num-2,6), MTar_vel, MWaiting_time,MPoint_num-2))// -2 is to delete the start and end point
            {
                double path_out[6] = {0,};
                while(Descr_RD_blending2.MultiP_path_exe(path_out))
                {
                    fprintf(Hand_G_recording,"%10f %10f %10f %10f %10f %10f %10f %10f %10f \n",
                    path_out[0], path_out[1], path_out[2], path_out[3], path_out[4], path_out[5],
                    0.0, 0.0, 10.0);
                }
            }

            /* 3) Contact to end path profile */
            Linear_travel_time = 3.0;

            for(int i=0;i<6;i++)
            {
                Tar_pos[i] = Decr_RD_points(MPoint_num-1,i);
                Init_pos[i] = Decr_RD_points(MPoint_num-2,i);
            }

            if(Descr_RD_blending3.PTP_6D_path_init(Init_pos, Tar_pos, Linear_travel_time))
            {
                double path_out[6] = {0,};
                while(Descr_RD_blending3.PTP_6D_path_exe(path_out))
                {
                    fprintf(Hand_G_recording,"%10f %10f %10f %10f %10f %10f %10f %10f %10f \n",
                    path_out[0], path_out[1], path_out[2], path_out[3], path_out[4], path_out[5],
                    0.0, 0.0, 0.0);
                }
            }

            fclose(Hand_G_recording);

            #endif
            printf("\n Descrete points to path profile done \n");


        }
        else if(control_mode == 7) // Posture playback
        {
            /*** STEP0 : Iteration number setting ***/
            int iter_num = -1;
            std::cout << "\n Select the iteration num(within 1~9): " << std::endl;
            std::cin >> iter_num;
            if(iter_num > 0 && iter_num < 10)
            {
                PbNum_command_msg.data = iter_num;
                PbNum_command_pub->publish(PbNum_command_msg);
                rclcpp::spin_some(node); //// ros::spinOnce();

                /*** STEP1 : Playback start ***/
                int PB_exe_confirm = 0;
                std::cout << "\n 1: Start the playback, 0: quit" << std::endl;
                std::cin >> PB_exe_confirm;
                if(PB_exe_confirm == 1)
                {
                    yoon_mode_msg.data = Playback_mode_cmd;
                    yoon_mode_pub->publish(yoon_mode_msg);
                    rclcpp::spin_some(node); //// ros::spinOnce();
                }
            }
            else
            {
                printf("\n The wrong interation number was selected (%d)",iter_num);
            }


        }
        else if(control_mode == 0) // Motion stop
        {
            yoon_mode_msg.data = Motion_stop_cmd;
            yoon_mode_pub->publish(yoon_mode_msg);
            rclcpp::spin_some(node); //// ros::spinOnce();
        }

        

    }
    exit(0);
    return 0;

}