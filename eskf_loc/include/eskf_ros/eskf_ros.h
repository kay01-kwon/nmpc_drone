#ifndef ESKF_ROS_H
#define ESKF_ROS_H
#include "eskf_loc/eskf_loc.h"
#include "utils/interpolation_tool.h"
#include "utils/circular_buffer.h"
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

struct ImuData{
    double time_stamp;
    Vec6d u;
};

struct PoseData{
    double time_stamp;
    Vec3d p;
    Quatd q;
};

struct EskfData{
    State s;
    Mat18x18 P;
};


class ESKF_ROS{
    
    public:

    ESKF_ROS();

    ESKF_ROS(ros::NodeHandle &nh);

    ~ESKF_ROS();

    void run();

    private:

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pose_sub_;
    ros::Timer pub_timer_;

    ros::Publisher state_pub_;
    tf::Transform transform_;

    nav_msgs::Odometry eskf_msg_;

    double dt_imu_debug_{0.0};
    double dt_pose_debug_{0.0};

    Vec6d u_lpf_;

    bool is_first_estimate_{false};
    bool is_time_ready_{false};
    double t_est_now_;
    double t_est_old_;

    // Error state Kalman filter instance
    EskfLoc *eskf_loc_;


    // Circular buffer for IMU, pose and estimation data
    CircularBuffer<ImuData> imu_buffer_;
    CircularBuffer<PoseData> pose_buffer_;

    EskfData eskf_data_;

    bool imu_ready_{false};
    bool pose_ready_{false};

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void find_past_imu_data(size_t &idx0, const double &t_est_old);

    void put_eskf_data_to_msg();

    void publish_current_state(const ros::TimerEvent&);

    void set_param(EskfLocParams &params);

};


#endif