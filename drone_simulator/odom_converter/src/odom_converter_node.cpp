#include "odom_converter/odom_converter.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odom_converter_node");
    ros::NodeHandle nh;
    
    // Create an instance of the OdomConverter class
    OdomConverter odom_converter(nh);
    
    // Spin to keep the node running
    ros::spin();
    
    return 0;
}