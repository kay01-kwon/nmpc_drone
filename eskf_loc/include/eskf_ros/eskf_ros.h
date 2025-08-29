#ifndef ESKF_ROS_H
#define ESKF_ROS_H
#include "eskf_loc/eskf_loc.h"
#include "utils/circular_buffer.h"
#include <ros/ros.h>
#include <deque>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

struct ImuData{
    double time_stamp;
    Control u;
};

struct PoseData{
    double time_stamp;
    Vec3 p;
    Quat q;
};


class ESKF_ROS{
    
    public:

    ESKF_ROS();

    ESKF_ROS(ros::NodeHandle &nh);

    void run();

    ~ESKF_ROS();

    private:

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void set_param(EskfLocParams &params);

    void imu_interpolate(const Control &u0,
                         const double &t0,
                         const Control &u1,
                         const double &t1,
                         const double &t,
                         Control &u_interp);

    void publish_current_state(const ros::TimerEvent&);

    Control control_;

    // Circular buffer for IMU, pose and estimation data
    CircularBuffer<ImuData> imu_buffer_;
    CircularBuffer<PoseData> pose_buffer_;

    Meas z_meas_prev_;
    Vec3 v_mocap_lpf_;

    // Error state Kalman filter instance
    EskfLoc *eskf_loc_;

    State s_, s_prev_;

    Cov P_, P_prev_;
    
    ros::NodeHandle nh_;

    ros::Subscriber imu_sub_;
    ros::Subscriber pose_sub_;
    ros::Timer pub_timer_;

    nav_msgs::Odometry eskf_msg_;

    ros::Publisher state_pub_;

    tf::Transform transform_;
    
};


#endif