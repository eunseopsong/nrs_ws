#include "FT_Processing.hpp"
#include <signal.h>


void catch_signal(int sig) 
{
    ros::shutdown();
    ROS_ERROR("Program was terminated");
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "FT_AQ_main");
	ros::NodeHandle nh;

    signal(SIGTERM, catch_signal); // Termination
	signal(SIGINT, catch_signal);  // Active
    
    int HandleID = 0x01;
    int ContactID = 0x11;
    double Sensor_sampling = 0.001;
    bool HaccSwitch = false;
    bool CaccSwitch = false;

    nh.param("HandleID",HandleID,HandleID);
    nh.param("ContactID",ContactID,ContactID);
    nh.param("Sensor_sampling",Sensor_sampling,Sensor_sampling);
    if(!nh.getParam("HandleACC",HaccSwitch)) {ROS_ERROR("HandleACC not found");}
    if(!nh.getParam("ContactACC",CaccSwitch)) {ROS_ERROR("ContactACC not found");}

    unsigned char Handle_ID = static_cast<unsigned char>(HandleID);
    unsigned char Contact_ID = static_cast<unsigned char>(ContactID);

    FT_processing FTP(nh, Sensor_sampling, Handle_ID, Contact_ID, HaccSwitch, CaccSwitch); // Node Handler, Sampling time(s)
    FTP.FT_run();

    return 0;
}