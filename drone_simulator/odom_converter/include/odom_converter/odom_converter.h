#ifndef ODOM_CONVERTER_H
#define ODOM_CONVERTER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using nav_msgs::Odometry;
using geometry_msgs::PoseStamped;

class OdomConverter{

    public:

    OdomConverter(ros::NodeHandle& nh);

    ~OdomConverter();

    private:

    void callback(const Odometry::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Subscriber odom_sub_;

    Odometry odom_;
    PoseStamped pose_;
};

#endif