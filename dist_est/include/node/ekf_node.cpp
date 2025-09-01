#include "ekf_node.h"

EkfNode::EkfNode()
{
    // Default constructor
}

EkfNode::EkfNode(ros::NodeHandle &nh) : nh_(nh)
{
    EKFParams ekf_params;
    MavParam mav_param;

    // Get publish rate from parameter server
    double publish_rate;
    std::string node_name = ros::this_node::getName();
    nh_.getParam(node_name + "/publish_rate", publish_rate);
    double duration = 1.0 / publish_rate;
    ROS_INFO("EKF publish rate: %f Hz", publish_rate);


    // Set EKF parameters from parameter server
    std::string ekf_param_P_name = "/ekf_param/P";
    std::string ekf_param_Q_name = "/ekf_param/Q";
    std::string ekf_param_R_name = "/ekf_param/R";

    setParam(ekf_param_P_name, ekf_params);
    setParam(ekf_param_Q_name, ekf_params);
    setParam(ekf_param_R_name, ekf_params);

    // Get MAV parameters from parameter server
    std::string nominal_param_name = "/nominal_param";
    setParam(nominal_param_name, mav_param);

    ekf_dist_est_ = new EkfDistEst(mav_param, ekf_params);

    // Initialize the EKF node with a NodeHandle
    rpm_sub_ = nh_.subscribe("/uav/actual_rpm", 1, &EkfNode::rpmCallback, this);
    pose_sub_ = nh_.subscribe("/eskf/Odom", 1, &EkfNode::stateCallback, this);

    publish_timer_ = nh_.createTimer(ros::Duration(duration), &EkfNode::publishCallback, this); // 100 Hz
    state_pub_ = nh_.advertise<nav_msgs::Odometry>("ekf/state", 1);
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("ekf/wrench", 1);

    ros_t_now_ = ros::Time::now().toSec();
    ros_t_old_ = ros_t_now_;

}

EkfNode::~EkfNode()
{
    if (ekf_dist_est_ != nullptr)
    {
        delete ekf_dist_est_;
        ekf_dist_est_ = nullptr;
    }
}

void EkfNode::run()
{

    ros::spin();

}

void EkfNode::rpmCallback(const ros_libcanard::hexa_actual_rpm &msg)
{

}


void EkfNode::stateCallback(const Odometry &msg)
{


}

void EkfNode::publishCallback(const ros::TimerEvent&)
{
    publishState();
    publishWrench();
}

void EkfNode::publishState()
{
    state_pub_.publish(state_msg_);
}

void EkfNode::publishWrench()
{
    wrench_pub_.publish(wrench_msg_);
}

void EkfNode::setParam(const std::string param_name, EKFParams &ekf_params)
{

    std::string node_name = ros::this_node::getName();

    std::string param_total_name = node_name + param_name;

    if (param_total_name == (node_name + "/ekf_param/P"))
    {
        std::vector<double> param_vector(19);
        nh_.getParam(param_total_name, param_vector);
        ekf_params.P.setZero();
        ROS_INFO("Setting EKF initial covariance P:");
        for (size_t i = 0; i < param_vector.size(); ++i)
        {
            ekf_params.P(i, i) = param_vector[i];
            ROS_INFO("ekf_params.P(%zu): %e", i, ekf_params.P(i,i));
        }
        ROS_INFO("\n");
    }
    else if (param_total_name == (node_name + "/ekf_param/Q"))
    {
        std::vector<double> param_vector(19);
        nh_.getParam(param_total_name, param_vector);
        ekf_params.Q.setZero();
        ROS_INFO("Setting EKF process noise covariance Q:");
        for (size_t i = 0; i < param_vector.size(); ++i)
        {
            ekf_params.Q(i, i) = param_vector[i];
            ROS_INFO("ekf_params.Q(%zu): %e", i, ekf_params.Q(i,i));
        }
        ROS_INFO("\n");
    }
    else if (param_total_name == (node_name + "/ekf_param/R"))
    {
        std::vector<double> param_vector(13);
        nh_.getParam(param_total_name, param_vector);
        ekf_params.R.setZero();
        ROS_INFO("Setting EKF measurement noise covariance R:");
        for (size_t i = 0; i < param_vector.size(); ++i)
        {
            ekf_params.R(i, i) = param_vector[i];
            ROS_INFO("ekf_params.R(%zu): %e", i, ekf_params.R(i,i));
        }
        ROS_INFO("\n");
    }
    else
    {
        ROS_ERROR("Unknown EKF parameter name: %s", param_total_name.c_str());
        return;
    }
}

void EkfNode::setParam(const std::string param_name, MavParam &mav_param)
{
    std::string node_name = ros::this_node::getName();
    std::string param_total_name = node_name + param_name;

    double m;
    double moi_xx, moi_yy, moi_zz;

    nh_.param(param_total_name + "/m", m, 2.9);
    nh_.param(param_total_name + "/moi/xx", moi_xx, 0.052);
    nh_.param(param_total_name + "/moi/yy", moi_yy, 0.052);
    nh_.param(param_total_name + "/moi/zz", moi_zz, 0.080);
    nh_.param(param_total_name + "/l", mav_param.l, 0.265);
    nh_.param(param_total_name + "/C_T", mav_param.C_T, 1.465e-07);
    nh_.param(param_total_name + "/k_m", mav_param.k_m, 0.01569);

    mav_param.m = m;  // Mass
    mav_param.J << moi_xx, 0.0, 0.0,
                   0.0, moi_yy, 0.0,
                   0.0, 0.0, moi_zz;  // Inertia matrix

    ROS_INFO("MavParam set: m = %f, moi = [%f, %f, %f]", 
             mav_param.m, moi_xx, moi_yy, moi_zz);

    ROS_INFO("MavParam set: l = %f, C_T = %e, k_m = %f", 
             mav_param.l, mav_param.C_T, mav_param.k_m);

}