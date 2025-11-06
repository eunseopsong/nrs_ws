#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <NRS_FT_AQ/vive_ft_msg.h>  // Custom msg

class ViveForceConverter
{
public:
    ViveForceConverter()
    {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("/vive_force", 10, &ViveForceConverter::callback, this);
        pub_ = nh.advertise<geometry_msgs::Vector3>("/force_vector", 10);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    void callback(const NRS_FT_AQ::vive_ft_msg::ConstPtr& msg)
    {
        geometry_msgs::Vector3 force_msg;
        force_msg.x = msg->Fx;
        force_msg.y = msg->Fy;
        force_msg.z = msg->Fz;

        pub_.publish(force_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vive_force_converter");
    ViveForceConverter converter;
    ros::spin();
    return 0;
}
