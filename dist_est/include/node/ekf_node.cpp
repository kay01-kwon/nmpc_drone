#include "ekf_node.h"

EkfNode::EkfNode()
{
    // Default constructor
}

EkfNode::EkfNode(ros::NodeHandle &nh) : nh_(nh)
{
    EKFParams ekf_params;
    ekf_params.P = 0.010 * 0.010 * Mat10x10::Identity();  // Initial covariance
    ekf_params.P.block(7,7,3,3) = 0.01 * Mat3x3::Identity();  // Initial covariance for tau
    ekf_params.Q = 0.001 * 0.001 * Mat10x10::Identity();  // Process noise covariance
    ekf_params.R = 0.001 * 0.001 * Mat7x7::Identity();  // Measurement noise covariance
    

    std::cout << "EKFParams P: " << ekf_params.P << std::endl;
    std::cout << "EKFParams Q: " << ekf_params.Q << std::endl;
    std::cout << "EKFParams R: " << ekf_params.R << std::endl;

    MavParam mav_param;

    mav_param.C_T = 1.465e-07;  // Thrust coefficient
    mav_param.k_m = 0.01569;  // Moment constant (C_M/C_T)
    mav_param.J << 0.052, 0.0, 0.0,
                   0.0, 0.052, 0.0,
                   0.0, 0.0, 0.080;  // Inertia matrix

    ekf_dist_est_.setParam(mav_param, ekf_params);

    // Initialize the EKF node with a NodeHandle
    rpm_sub_ = nh_.subscribe("/uav/actual_rpm", 1, &EkfNode::rpmCallback, this);
    pose_sub_ = nh_.subscribe("/custom_hexacopter/ground_truth/odometry", 1, &EkfNode::poseCallback, this);
    state_pub_ = nh_.advertise<nav_msgs::Odometry>("ekf_state", 1);
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("ekf_wrench", 1);

    t_curr_ = ros::Time::now().toSec();
    t_prev_ = t_curr_;
    t_rotor_ = t_curr_;

    rpm_.setZero();  // Initialize rpm vector to zero
    filtered_disturbance_.setZero();  // Initialize filtered disturbance to zero
    
}

void EkfNode::run()
{
    while(ros::ok())
    {

        ros::spinOnce();

        loop_rate_.sleep();
    }

    // ros::spin();

}

void EkfNode::rpmCallback(const ros_libcanard::hexa_actual_rpm &msg)
{
    t_rotor_ = msg.stamp.toSec();
    for (size_t i = 0; i < 6; i++)
    {
        rpm_[i] = msg.rpm[i];
    }
    
}

void EkfNode::poseCallback(const Odometry &msg)
{
    if (!is_first_callback_)
    {
        // Initialize the previous time
        t_prev_ = ros::Time::now().toSec();
        is_first_callback_ = true;
        return;  // Skip the first callback to avoid zero dt
    }
    t_curr_ = ros::Time::now().toSec();
    dt_ = t_curr_ - t_prev_;

    QuatType q_meas(msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z);

    Vec3 w_meas(msg.twist.twist.angular.x,
           msg.twist.twist.angular.y,
           msg.twist.twist.angular.z);

    MeasVector7 z_meas;
    z_meas << q_meas, w_meas;
    double dt;

    dt = t_curr_-t_rotor_;
    // std::cout << "dt: " << dt << std::endl;

    if (dt < 0.001)
    {
        // If the time difference is too small, skip the update
        // std::cout << "Skipping update due to small dt: " << dt << std::endl;
        return;
    }
    AugStateVector10 s_prior = ekf_dist_est_.predict(rpm_, dt);

    AugStateVector10 s_post = ekf_dist_est_.meas_update(z_meas);

    controlInputVector4 u = ekf_dist_est_.getControlInput(rpm_);
        // Publish the updated state
    state_msg_.header.stamp = ros::Time::now();
    state_msg_.pose.pose.orientation.w = s_post(0);
    state_msg_.pose.pose.orientation.x = s_post(1);
    state_msg_.pose.pose.orientation.y = s_post(2);
    state_msg_.pose.pose.orientation.z = s_post(3);

    state_msg_.twist.twist.angular.x = s_post(4);
    state_msg_.twist.twist.angular.y = s_post(5);
    state_msg_.twist.twist.angular.z = s_post(6);

    Vec3 tau(s_post(7), s_post(8), s_post(9));

    
    wrench_msg_.force.z = u(0);  // Thrust force
    wrench_msg_.torque.x = tau(0);
    wrench_msg_.torque.y = tau(1);
    wrench_msg_.torque.z = tau(2);

    publishWrench();
    publishState();

    t_prev_ = t_curr_;

}

void EkfNode::publishState()
{
    state_pub_.publish(state_msg_);
}

void EkfNode::publishWrench()
{
    wrench_pub_.publish(wrench_msg_);
}
