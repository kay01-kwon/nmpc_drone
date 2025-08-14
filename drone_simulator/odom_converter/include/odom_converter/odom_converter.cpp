#include "odom_converter.h"

OdomConverter::OdomConverter(ros::NodeHandle &nh)
{
    nh_ = nh;

    std::string subscribe_topic;
    std::string publish_topic;

    nh_.param("subscribe_topic", subscribe_topic, std::string("/custom_hexacopter/ground_truth/odometry"));
    nh_.param("publish_topic", publish_topic, std::string("/mocap"));

    odom_sub_ = nh_.subscribe(subscribe_topic, 1, &OdomConverter::callback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(publish_topic + "/odom", 1);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(publish_topic + "/pose", 1);
}
OdomConverter::~OdomConverter()
{

}
void OdomConverter::callback(const Odometry::ConstPtr &msg)
{
    odom_.header.stamp = msg->header.stamp;
    
    odom_.pose.pose.position.x = msg->pose.pose.position.x;
    odom_.pose.pose.position.y = msg->pose.pose.position.y;
    odom_.pose.pose.position.z = msg->pose.pose.position.z;

    odom_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    odom_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom_.pose.pose.orientation.z = msg->pose.pose.orientation.z;

    pose_.header.stamp = msg->header.stamp;
    pose_.pose.position.x = msg->pose.pose.position.x;
    pose_.pose.position.y = msg->pose.pose.position.y;
    pose_.pose.position.z = msg->pose.pose.position.z;

    pose_.pose.orientation.w = msg->pose.pose.orientation.w;
    pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    pose_.pose.orientation.z = msg->pose.pose.orientation.z;

    odom_pub_.publish(odom_);
    pose_pub_.publish(pose_);
}