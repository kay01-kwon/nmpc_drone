#include "rps_to_rpm_converter.h"

RpsToRpmConverter::RpsToRpmConverter(ros::NodeHandle& nh) : nh_(nh)
{
    ros::TransportHints transport_hints;
    transport_hints = ros::TransportHints().tcpNoDelay(true);
    rps_sub_ = nh_.subscribe("/custom_hexacopter/motor_speed", 10, 
    &RpsToRpmConverter::rpsCallback, this, transport_hints);
    rpm_pub_ = nh_.advertise<ros_libcanard::hexa_actual_rpm>("/uav/actual_rpm", 1);
}

RpsToRpmConverter::~RpsToRpmConverter()
{
    // Destructor can be used for cleanup if needed
}

void RpsToRpmConverter::rpsCallback(const mav_msgs::Actuators::ConstPtr& msg)
{
    rpm_msg_.stamp = msg->header.stamp;

    for (size_t i = 0; i < msg->angular_velocities.size(); ++i) 
    {
        double rps_value = msg->angular_velocities[i];
        double rpm_value = std::abs(rps_value * rps_to_rpm_);
        rpm_msg_.rpm[i] = rpm_value;
    }

    rpm_pub_.publish(rpm_msg_);
}

