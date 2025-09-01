#ifndef EKF_NODE_CPP
#define EKF_NODE_CPP

#include <ros/ros.h>
#include "dist_est/ekf_dist_est.h"
#include "utils/circular_buffer.h"

#include "ros_libcanard/hexa_actual_rpm.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"

using nav_msgs::Odometry;
using geometry_msgs::Wrench;

class EkfNode
{

    public:

    EkfNode();

    EkfNode(ros::NodeHandle &nh);

    ~EkfNode();

    void run();

    private:

    ros::NodeHandle nh_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber pose_sub_;

    ros::Timer publish_timer_;
    ros::Publisher state_pub_;
    ros::Publisher wrench_pub_;

    nav_msgs::Odometry state_msg_;

    geometry_msgs::Wrench wrench_msg_;

    double ros_t_now_;
    double ros_t_old_;

    EkfDistEst* ekf_dist_est_;

    AugState state_;
    Mat19x19 P_;

    CircularBuffer<nav_msgs::Odometry> state_buffuer_;
    CircularBuffer<ros_libcanard::hexa_actual_rpm> rpm_buffer_;

    void rpmCallback(const ros_libcanard::hexa_actual_rpm &msg);

    void stateCallback(const Odometry &msg);

    void publishCallback(const ros::TimerEvent&);

    void publishState();

    void publishWrench();

    void setParam(const std::string param_name, EKFParams &ekf_params);

    void setParam(const std::string param_name, MavParam &mav_param);


};


#endif