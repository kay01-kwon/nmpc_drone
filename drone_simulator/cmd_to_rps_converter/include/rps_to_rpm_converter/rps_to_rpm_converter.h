#ifndef RPS_TO_RPM_CONVERTER_H
#define RPS_TO_RPM_CONVERTER_H
#include <iostream>
#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <ros_libcanard/hexa_actual_rpm.h>

class RpsToRpmConverter
{
    public:

        RpsToRpmConverter(ros::NodeHandle& nh);

        ~RpsToRpmConverter();

    private:

        ros::NodeHandle nh_;
        ros::Subscriber rps_sub_;
        ros::Publisher rpm_pub_;

        ros::Rate loop_rate_{100}; // 100 Hz

        double rps_to_rpm_{60.0 / (2.0 * M_PI)}; // Convert RPS to RPM

        ros_libcanard::hexa_actual_rpm rpm_msg_;

        void rpsCallback(const mav_msgs::Actuators::ConstPtr& msg);

};

#endif