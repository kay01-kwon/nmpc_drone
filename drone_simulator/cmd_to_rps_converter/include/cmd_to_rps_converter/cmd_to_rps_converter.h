#ifndef CMD_TO_RPS_CONVERTER_H
#define CMD_TO_RPS_CONVERTER_H
#include <iostream>
#include <ros/ros.h>
#include <ros_libcanard/hexa_cmd_raw.h>
#include <mav_msgs/Actuators.h>

using std::cout;
using std::endl;

class CmdToRpsConverter
{
    public:

        CmdToRpsConverter(ros::NodeHandle& nh);

        ~CmdToRpsConverter();

        void run();


    private:

        void cmdCallback(const ros_libcanard::hexa_cmd_raw::ConstPtr& msg);
        
        ros::NodeHandle nh_;
        ros::Subscriber cmd_sub_;
        ros::Publisher rps_pub_;

        ros::Rate loop_rate_{100}; // 50 Hz

        double max_bits_{8191};
        double max_rpm_{9800};
        double rpm_to_rps_{2.0* M_PI / 60.0}; // Convert RPM to RPS

        mav_msgs::Actuators rps_msg_;
};



#endif