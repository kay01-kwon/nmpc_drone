#ifndef EKF_NODE_CPP
#define EKF_NODE_CPP

#include <ros/ros.h>
#include "dist_est/ekf_dist_est.h"
#include "ros_libcanard/hexa_actual_rpm.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include <deque>

using nav_msgs::Odometry;
using geometry_msgs::Wrench;

class EkfNode
{

    public:

    EkfNode();

    EkfNode(ros::NodeHandle &nh);

    void run();


    private:

    ros::NodeHandle nh_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher state_pub_;
    ros::Publisher wrench_pub_;

    double t_curr_, t_prev_, dt_;
    double t_rotor_;

    EkfDistEst ekf_dist_est_;

    void rpmCallback(const ros_libcanard::hexa_actual_rpm &msg);

    void interpolate_rpm(RotorThrustVector6 &interpolated_rpm, double &t_meas);

    void poseCallback(const Odometry &msg);

    void publishState();

    void publishWrench();

    void setParam(const std::string param_name, EKFParams &ekf_params);

    void setParam(const std::string param_name, MavParam &mav_param);

    rpmVector6 rpm_;
    std::deque<std::pair<double, RotorThrustVector6>> rpm_buffer_;

    ros::Rate loop_rate_{100};  // 100 Hz

    nav_msgs::Odometry state_msg_;

    geometry_msgs::Wrench wrench_msg_;

    Vec3 filtered_disturbance_{0.0, 0.0, 0.0};

    bool is_first_callback_{false};

};


#endif