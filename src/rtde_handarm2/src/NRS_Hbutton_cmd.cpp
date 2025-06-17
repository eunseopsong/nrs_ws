#include "NRS_Hbutton_cmd.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "Yoon_UR10e_cmd.h"
#include <rclcpp/rclcpp.hpp> //// #include "ros/ros.h"

// For ROS message
#include "std_msgs/msg/multi_array_layout.hpp"    //// #include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/msg/multi_array_dimension.hpp" //// #include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/msg/float64_multi_array.hpp"   //// #include "std_msgs/Float64MultiArray.h"
#include "std_msgs/msg/float64.hpp"               //// #include "std_msgs/Float64.h"
#include "std_msgs/msg/u_int16.hpp"               //// #include "std_msgs/UInt16.h"
#include "std_msgs/msg/u_int32.hpp"               //// #include "std_msgs/UInt32.h"

#include "geometry_msgs/msg/point_stamped.hpp"   //// #include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/msg/pose_stamped.hpp"    //// #include "geometry_msgs/PoseStamped.h"
// #include "rtde_handarm2/VrPosRtMsgQua.msg"         //// #include "rtde_handarm/VRposRtMsg_Qua.h" // 보류

// For ROS service - AIDIN GUI SERVER
#include "std_srvs/srv/empty.hpp"                //// #include <std_srvs/Empty.h>

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
#define TEACHING_MODE 0 // Direct Teaching: 0, VR-Vision Teaching: 1 (Danang Demo)
#define Handle_OnOff 1 // Handle on : 1, Handle off : 0


rclcpp::Node::SharedPtr node_;
rclcpp::Duration loop_period_ = rclcpp::Duration::from_seconds(0.01);  // 예: 10ms
// rclcpp::Duration loop_period_;
rclcpp::Time last_time_;



/* Main calss */
class NRS_Hbutton_cmd
{
    public:
        NRS_Hbutton_cmd(const rclcpp::Node::SharedPtr &node, int loop_rate); //// NRS_Hbutton_cmd(ros::NodeHandle &nh, int Loop_rate);
        ~NRS_Hbutton_cmd();

        /*** Functions definition ***/

        /* ROS_MSG Callback functions */
        void VRPose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg); 
        //// void VRPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        /* Service handles */
        //// bool SRV1_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool SRV1_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                 std::shared_ptr<std_srvs::srv::Empty::Response> response);

        //// bool SRV3_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool SRV3_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);

        //// bool SRV4_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool SRV4_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);

        ////  bool SRV11_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool SRV11_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);

        //// bool SRV12_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool SRV12_Handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);


        /* Main function */
        void HButton_main();

        /* Mode functions */
        void Mode_chage();
        void VR_mode_change();
        void Way_point_save();
        void VR_point_save();
        void Trajectory_gen();
        void Iter_num_set();
        void Playback_exe();
        void catch_signal(int sig);

        /*** Parameters setting ***/
        /* ROS setting */
        rclcpp::Rate loop_rate(10);  //// ros::Rate loop_rate;

        /* UART instance */
        Yoon_UART* Yuart;

        /* Yaml-file instance */
        std::ifstream fin1;
        std::ifstream fin2;

        YAML::Node NRS_recording;
        YAML::Node NRS_Fcon_desired;

        /* ROS Message instance */
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr yoon_mode_pub;           //// ros::Publisher yoon_mode_pub;
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr PbNum_command_pub;       //// ros::Publisher PbNum_command_pub;
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr Clicked_pub;             //// ros::Publisher Clicked_pub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr VRPose_sub; //// ros::Subscriber VRPose_sub;

        geometry_msgs::msg::PointStamped Clicked_msg; //// geometry_msgs::PointStamped Clicked_msg;
        std_msgs::msg::UInt16 yoon_mode_msg;          //// std_msgs::UInt16 yoon_mode_msg;
        std_msgs::msg::UInt32 PbNum_command_msg;      //// std_msgs::UInt32 PbNum_command_msg;

        /* ROS Service instance */
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv1;  //// ros::ServiceServer Aidin_gui_srv1;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv3;  //// ros::ServiceServer Aidin_gui_srv3;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv4;  //// ros::ServiceServer Aidin_gui_srv4;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv11; //// ros::ServiceServer Aidin_gui_srv11;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr Aidin_gui_srv12; //// os::ServiceServer Aidin_gui_srv12;

        /* Normal parameters */
        geometry_msgs::msg::Point VRPose_point; //// geometry_msgs::Point VRPose_point;
        char buffer[1024] = {0};
        bool pre_button_val = false;
        bool button_val = false;

        int Mode_val = 0;
        int Pre_Mode_val = 0;

        bool guiding_mode = false; // false: stanby mode, true: guiding mode
        int point_counter = 0; // Saved way points
        int iter_num = 0; // Iteration number
        int PB_exe_counter = 0; // 1: ready status, 2: execution

        /* State & mode */
        std::string current_status;
        std::string mode0,mode1,mode2,mode3,mode4;
        std::string modeErr1;

        std::string Hmode0,Hmode1,Hmode2,Hmode3,Hmode4,Hmode5;
    private:

};

// NRS_Hbutton_cmd::NRS_Hbutton_cmd(ros::NodeHandle &nh, int Loop_rate)
// : loop_rate(Loop_rate), fin1(NRS_Record_Printing_loc), fin2(NRS_Fcon_desired_loc)
NRS_Hbutton_cmd::NRS_Hbutton_cmd(const rclcpp::Node::SharedPtr &node, int loop_rate_val)
: node_(node), loop_rate_val_(loop_rate_val), fin1(NRS_Record_Printing_loc), fin2(NRS_Fcon_desired_loc)
{
    /* UART init */
    #if(Handle_OnOff == 1)
    Yuart = new Yoon_UART("/dev/ttyACM0",(int)115200);
    #endif

    /* Yaml-file */
    NRS_recording = YAML::Load(fin1);
    NRS_Fcon_desired = YAML::Load(fin2);
    
    /* ROS Message */
    yoon_mode_pub = node_->create_publisher<std_msgs::msg::UInt16>("Yoon_UR10e_mode", 10);
    PbNum_command_pub = node_->create_publisher<std_msgs::msg::UInt16>("Yoon_PbNum_cmd", 10);
    Clicked_pub = node_->create_publisher<geometry_msgs::msg::PointStamped>("/clicked_point", 10);
    //// yoon_mode_pub = nh.advertise<std_msgs::UInt16>("Yoon_UR10e_mode",20);
    //// PbNum_command_pub = nh.advertise<std_msgs::UInt16>("Yoon_PbNum_cmd",20);
    //// Clicked_pub = nh.advertise<geometry_msgs::PointStamped>("/clicked_point",20);

    VRPose_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/pos_cal_rviz", 10,
    std::bind(&NRS_Hbutton_cmd::VRPose_Callback, this, std::placeholders::_1)
    );
    //// VRPose_sub = nh.subscribe("/pos_cal_rviz",20,&NRS_Hbutton_cmd::VRPose_Callback,this);

    /* ROS Service */

    // Concerning for waypoint save & trajectory generation
    //// Aidin_gui_srv1 = nh.advertiseService("teaching_mode",&NRS_Hbutton_cmd::SRV1_Handle,this);
    Aidin_gui_srv1 = node_->create_service<std_srvs::srv::Empty>(
        "teaching_mode",
        std::bind(&NRS_Hbutton_cmd::SRV1_Handle, this, std::placeholders::_1, std::placeholders::_2)
    );

    //// Aidin_gui_srv3 = nh.advertiseService("save_waypoint",&NRS_Hbutton_cmd::SRV3_Handle,this);
    Aidin_gui_srv3 = node_->create_service<std_srvs::srv::Empty>(
        "save_waypoint",
        std::bind(&NRS_Hbutton_cmd::SRV3_Handle, this, std::placeholders::_1, std::placeholders::_2)
    );

    //// Aidin_gui_srv4 = nh.advertiseService("trajectory_generation",&NRS_Hbutton_cmd::SRV4_Handle,this);
    Aidin_gui_srv4 = node_->create_service<std_srvs::srv::Empty>(
        "trajectory_generation",
        std::bind(&NRS_Hbutton_cmd::SRV4_Handle, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Concerning for execution
    //// Aidin_gui_srv11 = nh.advertiseService("Iteration_set",&NRS_Hbutton_cmd::SRV11_Handle,this);
    Aidin_gui_srv11 = node_->create_service<std_srvs::srv::Empty>(
        "Iteration_set",
        std::bind(&NRS_Hbutton_cmd::SRV11_Handle, this, std::placeholders::_1, std::placeholders::_2)
    );

    //// Aidin_gui_srv12 = nh.advertiseService("Playback_execution",&NRS_Hbutton_cmd::SRV12_Handle,this);
    Aidin_gui_srv12 = node_->create_service<std_srvs::srv::Empty>(
        "Playback_execution",
        std::bind(&NRS_Hbutton_cmd::SRV12_Handle, this, std::placeholders::_1, std::placeholders::_2)
    );

    /* State & mode */

    #if(TEACHING_MODE == 0)
        #if(Handle_OnOff == 0)
        current_status = "Stanby mode - No Handle";
        mode0 = "Stanby mode - No Handle";
        mode1 = "Teaching mode - No Handle";
        #elif(Handle_OnOff == 1)
        current_status = "Handle stanby mode";
        mode0 = "Handle stanby mode";
        mode1 = "Handle-guiding mode";
        #endif
    #elif(TEACHING_MODE == 1)
        current_status = "VR stanby mode";
        mode0 = "VR stanby mode";
        mode1 = "VR-teaching mode";
    #endif
    mode2 = "Playback ready";
    mode3 = "Playback execution";
    mode4 = "Path generation done";
    modeErr1 = "Wrong iter number(1~9)";

    Hmode0 = "MODE 0"; // Defualt value
    Hmode1 = "MODE 1"; // Mode change (Stanby mode, guiding mode)
    Hmode2 = "MODE 2"; // Way point save
    Hmode3 = "MODE 3"; // Playback start (Ready , execution)
    Hmode4 = "MODE 4"; // Iteration number
    Hmode5 = "MODE 5"; // Trajectory generation
}

NRS_Hbutton_cmd::~NRS_Hbutton_cmd()
{
    #if(Handle_OnOff == 1)
    Yuart->YUART_terminate();
    delete Yuart;
    #endif
}

void NRS_Hbutton_cmd::catch_signal(int sig)
{
    if(sig == 0){printf("Desired posture is under 4 \n");}
    #if(Handle_OnOff == 1)
    Yuart->YUART_terminate();
    #endif
    printf("Program was terminated !! \n");
    exit(1);
}

/* ROS_MSG functions */
// void NRS_Hbutton_cmd::VRPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
void NRS_Hbutton_cmd::VRPose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    VRPose_point.x = msg->pose.position.x;
    VRPose_point.y = msg->pose.position.y; 
    VRPose_point.z = msg->pose.position.z;

    // std::cout << msg->pose.position.x << msg->pose.position.y << msg->pose.position.z << std::endl;
}

/* Service functions */
// bool NRS_Hbutton_cmd::SRV1_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
bool NRS_Hbutton_cmd::SRV1_Handle(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    #if(TEACHING_MODE == 0)
    Mode_chage();
    #elif(TEACHING_MODE == 1)
    VR_mode_change();
    #endif
    return true;
}

// bool NRS_Hbutton_cmd::SRV3_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
bool NRS_Hbutton_cmd::SRV3_Handle(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    #if(TEACHING_MODE == 0)
    Way_point_save();
    #elif(TEACHING_MODE == 1)
    VR_point_save();
    #endif
    return true;
}

// bool NRS_Hbutton_cmd::SRV4_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
bool NRS_Hbutton_cmd::SRV4_Handle(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    #if(TEACHING_MODE == 0)
    Trajectory_gen();
    #endif
    return true;
}

/* Iteration number set */
// bool NRS_Hbutton_cmd::SRV11_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
bool NRS_Hbutton_cmd::SRV11_Handle(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    #if(TEACHING_MODE == 0)
    Iter_num_set();
    #endif
    return true;
}

/* Playback execution */
// bool NRS_Hbutton_cmd::SRV12_Handle(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
bool NRS_Hbutton_cmd::SRV12_Handle(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    #if(TEACHING_MODE == 0)
    Playback_exe();
    #endif
    return true;
}

void NRS_Hbutton_cmd::Mode_chage()
{
    if(guiding_mode == false) // change to guiding mode
    {
        guiding_mode = true;
        current_status = mode1;

        #if(Handle_OnOff == 1)
        yoon_mode_msg.data = Hand_guiding_mode_cmd;
        yoon_mode_pub->publish(yoon_mode_msg);  //// yoon_mode_pub.publish(yoon_mode_msg);
        rclcpp::spin_some(node_);               //// ros::spinOnce();
        #endif
    }
    else // change to stanby mode
    {
        guiding_mode = false;
        current_status = mode0;

        #if(Handle_OnOff == 1)
        // Stanby mode message publish 
        yoon_mode_msg.data = Motion_stop_cmd;
        yoon_mode_pub->publish(yoon_mode_msg); //// yoon_mode_pub.publish(yoon_mode_msg);
        rclcpp::spin_some(node_);              //// ros::spinOnce();
        #endif

        if(point_counter != 0)
        {
            // Way point save termination message publish
            yoon_mode_msg.data = Descrete_recording_save;
            yoon_mode_pub->publish(yoon_mode_msg); //// yoon_mode_pub.publish(yoon_mode_msg);
            rclcpp::spin_some(node_);              //// ros::spinOnce();
        }
    }
}
void NRS_Hbutton_cmd::VR_mode_change()
{
    if(guiding_mode == false) // change to guiding mode
    {
        guiding_mode = true;
        current_status = mode1;
    }
    else // change to stanby mode
    {
        guiding_mode = false;
        current_status = mode0;
        point_counter = 0;
    }
}

void NRS_Hbutton_cmd::Way_point_save()
{
    yoon_mode_msg.data = Descrete_reording_start;
    yoon_mode_pub->publish(yoon_mode_msg); //// yoon_mode_pub.publish(yoon_mode_msg);
    rclcpp::spin_some(node_);              //// ros::spinOnce();
    point_counter ++;
}

void NRS_Hbutton_cmd::VR_point_save()
{
    Clicked_msg.point.x = VRPose_point.x;
    Clicked_msg.point.y = VRPose_point.y;
    Clicked_msg.point.z = VRPose_point.z;

    Clicked_pub->publish(Clicked_msg);  //// Clicked_pub.publish(Clicked_msg);
    rclcpp::spin_some(node_);           //// ros::spinOnce();

    point_counter++;
}

void NRS_Hbutton_cmd::Trajectory_gen()
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
    current_status = mode4;
}

void NRS_Hbutton_cmd::Iter_num_set()
{
    if(PB_exe_counter == 0)
    {
        if(iter_num<9) iter_num ++;
        else iter_num = 0;
    }
}

void NRS_Hbutton_cmd::Playback_exe()
{
    if(iter_num > 0 && iter_num < 10)
    {
        if(PB_exe_counter == 0)
        {
            // Playback iteration number publish
            PbNum_command_msg.data = iter_num;
            PbNum_command_pub->publish(PbNum_command_msg); //// PbNum_command_pub.publish(PbNum_command_msg);
            rclcpp::spin_some(node_);                      //// ros::spinOnce();
            current_status = mode2;
            PB_exe_counter++;
        }
        else if(PB_exe_counter == 1)
        {
            /*** STEP1 : Playback start ***/
            yoon_mode_msg.data = Playback_mode_cmd;
            yoon_mode_pub->publish(yoon_mode_msg); //// yoon_mode_pub.publish(yoon_mode_msg);
            rclcpp::spin_some(node_);              //// ros::spinOnce();
            current_status = mode3;
            iter_num = 0;
            PB_exe_counter = 0;
        }
        else PB_exe_counter = 0;

        
    }
    else current_status = modeErr1;
}

void NRS_Hbutton_cmd::HButton_main()
{
    while(1)
    {
        /** Real time monitoring **/
        printf("\n============================================================\n");
        printf("************************************************************\n");
        printf("NOTE : Before start the main node, the screen of handle \n");
        printf("       must be 'STANBY MODE' \n");
        printf("************************************************************\n");
        printf("Current status: %s \n",current_status.c_str());
        printf("Selected way points: %d, Iteration number(1~9): %d \n",point_counter,iter_num);
        printf("Mode value: %d \n",Mode_val);
        printf("\n============================================================\n");

        #if(Handle_OnOff == 1)
        if(Yuart->YUART_start(buffer)) 
        {
            // printf("%s\n",buffer);

            /* Raw mode aquisition */
            if(strcmp(buffer,Hmode0.c_str())==0) Mode_val=0;
            else if(strcmp(buffer,Hmode1.c_str())==0) Mode_val=1;
            else if(strcmp(buffer,Hmode2.c_str())==0) Mode_val=2;
            else if(strcmp(buffer,Hmode3.c_str())==0) Mode_val=3;
            else if(strcmp(buffer,Hmode4.c_str())==0) Mode_val=4;
            else if(strcmp(buffer,Hmode5.c_str())==0) Mode_val=5;


            if((Pre_Mode_val == 0) && (Mode_val == 1)) // Mode change
            {
                #if(TEACHING_MODE == 0)
                Mode_chage();
                #elif(TEACHING_MODE == 1)
                VR_mode_change();
                #endif
            }
            if((Pre_Mode_val == 0) && (Mode_val == 2)) // Way point save
            {
                #if(TEACHING_MODE == 0)
                Way_point_save();
                #elif(TEACHING_MODE == 1)
                VR_point_save();
                #endif
            }
            if((Pre_Mode_val == 0) && (Mode_val == 5)) // Generate the trajectory
            {
                #if(TEACHING_MODE == 0)
                Trajectory_gen();
                #endif
            }
            if((Pre_Mode_val == 0) && (Mode_val == 4)) // Iteration number setting
            {
                Iter_num_set();
            }
            if((Pre_Mode_val == 0) && (Mode_val == 3)) // Playback execution
            {   
                Playback_exe();
            }

            Pre_Mode_val = Mode_val; // mode update to past

        }
        #endif
        rclcpp::spin_some(node_);  // For service callback
        // ros::spinOnce();
    }
    exit(0);
    #if(Handle_OnOff == 1)
    Yuart->YUART_terminate();
    #endif
}


// 종료 시그널 처리 함수
void catch_signal(int sig)
{
    (void)sig;  // unused 경고 제거
    printf("Program was terminated\n");
    rclcpp::shutdown();  // ROS 2에서는 shutdown이 안전하게 종료됨
    exit(0);
}

int main(int argc, char **argv)
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);

    // ROS 2 노드 생성
    auto node = rclcpp::Node::make_shared("nrs_hbutton_cmd");

    // 클래스 인스턴스 생성: node와 loop rate 전달
    NRS_Hbutton_cmd NRS_HB_cmd(node, 100);

    // 종료 시그널 핸들러 등록
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    // 메인 함수 실행
    NRS_HB_cmd.HButton_main();

    return 0;
}


// void catch_signal(int sig)
// {
//     printf("Program was terminated \n");
//     exit(1);
// }


// int main(int argc, char **argv)
// {
//     // ros::init(argc,argv,"NRS_Hbutton_cmd");
//     // ros::NodeHandle _nh;
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("nrs_hbutton_cmd");
//     NRS_Hbutton_cmd NRS_HB_cmd(_nh,100);

//     signal(SIGTERM, catch_signal);// Termination
// 	signal(SIGINT, catch_signal);// Active

//     NRS_HB_cmd.HButton_main();

//     return 0;
// }
