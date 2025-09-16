#include "cmd_to_rps_converter.h"

CmdToRpsConverter::CmdToRpsConverter(ros::NodeHandle& nh) 
: nh_(nh)
{
    ros::TransportHints transport_hints;
    transport_hints = ros::TransportHints().tcpNoDelay(true);
    cmd_sub_ = nh_.subscribe("/uav/cmd_raw", 10, &CmdToRpsConverter::cmdCallback, this, transport_hints);
    rps_pub_ = nh_.advertise<mav_msgs::Actuators>("/custom_hexacopter/command/motor_speed", 10);

    rps_msg_.angular_velocities.reserve(6);  // Resize to match the number of motors
    rps_msg_.angular_velocities.assign(6, 0.0); // Initialize all velocities to zero
}

CmdToRpsConverter::~CmdToRpsConverter()
{
    // Destructor can be used for cleanup if needed
}

void CmdToRpsConverter::run()
{
    while(ros::ok())
    {
        rps_pub_.publish(rps_msg_);
        ros::spinOnce();
        loop_rate_.sleep();
    }
}

void CmdToRpsConverter::cmdCallback(const ros_libcanard::hexa_cmd_raw::ConstPtr& msg)
{
    rps_msg_.header.stamp = ros::Time::now();
    rps_msg_.header.frame_id = "base_link";

    for (int i = 0; i < 6; ++i) 
    {
        double cmd_value = double(msg->raw[i]);
        double rps_value = (cmd_value / max_bits_) * max_rpm_ * rpm_to_rps_;
        rps_msg_.angular_velocities.at(i) = rps_value;
    }
}