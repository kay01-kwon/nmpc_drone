#include "eskf_ros/eskf_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eskf_node");
    ros::NodeHandle nh;

    ESKF_ROS eskf_ros(nh);
    
    eskf_ros.run();

    return 0;
}