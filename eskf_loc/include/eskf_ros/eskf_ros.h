#ifndef ESKF_ROS_H
#define ESKF_ROS_H
#include "eskf_loc/eskf_loc.h"
#include <ros/ros.h>
#include <deque>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>


class ESKF_ROS{
    
    public:

    ESKF_ROS();

    ESKF_ROS(ros::NodeHandle &nh);

    void run();

    ~ESKF_ROS();

    private:

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);

    void imu_interpolate(sensor_msgs::Imu &imu_out, double t_meas);

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void set_param(EskfLocParams &params);

    std::deque<sensor_msgs::Imu> imu_buffer_;
    double t_pose_prev_;
    double t_pose_curr_;

    Control control_;

    Meas z_meas_prev_;
    Vec3 v_mocap_lpf_;

    EskfLoc *eskf_loc_;
    
    ros::NodeHandle nh_;

    ros::Subscriber imu_sub_;
    ros::Subscriber pose_sub_;

    ros::Publisher state_pub_;
    ros::Publisher mocap_pub_;

    bool is_meas_first_{false};
};


#endif