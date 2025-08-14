#include "ekf_node.h"

EkfNode::EkfNode()
{
    // Default constructor
}

EkfNode::EkfNode(ros::NodeHandle &nh) : nh_(nh)
{
    EKFParams ekf_params;
    MavParam mav_param;

    std::string node_name;

    node_name = ros::this_node::getName();

    ROS_INFO("node_name: %s", node_name.c_str());

    std::vector<double> ekf_param_P(10);
    
    nh_.getParam(node_name + "/ekf_param/P", ekf_param_P);

    std::string ekf_param_P_name = "/ekf_param/P";
    std::string ekf_param_Q_name = "/ekf_param/Q";
    std::string ekf_param_R_name = "/ekf_param/R";

    setParam(ekf_param_P_name, ekf_params);
    setParam(ekf_param_Q_name, ekf_params);
    setParam(ekf_param_R_name, ekf_params);

    std::string nominal_param_name = "/nominal_param";
    
    setParam(nominal_param_name, mav_param);

    // ekf_params.P = 0.010 * 0.010 * Mat10x10::Identity();  // Initial covariance
    // ekf_params.P.block(7,7,3,3) = 0.01 * Mat3x3::Identity();  // Initial covariance for tau
    // ekf_params.Q = 0.001 * 0.001 * Mat10x10::Identity();  // Process noise covariance
    // ekf_params.R = 0.001 * 0.001 * Mat7x7::Identity();  // Measurement noise covariance

    mav_param.C_T = 1.465e-07;  // Thrust coefficient
    mav_param.k_m = 0.01569;  // Moment constant (C_M/C_T)
    mav_param.J << 0.052, 0.0, 0.0,
                   0.0, 0.052, 0.0,
                   0.0, 0.0, 0.080;  // Inertia matrix

    ekf_dist_est_.setParam(mav_param, ekf_params);

    // Initialize the EKF node with a NodeHandle
    rpm_sub_ = nh_.subscribe("/uav/actual_rpm", 10, &EkfNode::rpmCallback, this);
    pose_sub_ = nh_.subscribe("/custom_hexacopter/ground_truth/odometry", 10, &EkfNode::poseCallback, this);
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
    // while(ros::ok())
    // {

    //     ros::spinOnce();

    //     loop_rate_.sleep();
    // }

    ros::spin();

}

void EkfNode::rpmCallback(const ros_libcanard::hexa_actual_rpm &msg)
{
    t_rotor_ = msg.stamp.toSec();
    RotorThrustVector6 rpm_temp;
    for (size_t i = 0; i < 6; i++)
    {
        rpm_temp[i] = double(msg.rpm[i]);
    }
    
    rpm_buffer_.push_back(std::make_pair(t_rotor_, rpm_temp));

    if(rpm_buffer_.size() > 100)
    {
        rpm_buffer_.pop_front();  // Keep the buffer size manageable
    }
}

void EkfNode::interpolate_rpm(RotorThrustVector6 &interpolated_rpm, double &t_meas)
{
    for(size_t i = 0; i < rpm_buffer_.size() - 1; ++i)
    {
        if(t_meas >= rpm_buffer_[i].first && t_meas <= rpm_buffer_[i+1].first)
        {
            double t0 = rpm_buffer_[i].first;
            double t1 = rpm_buffer_[i+1].first;
            RotorThrustVector6 rpm0 = rpm_buffer_[i].second;
            RotorThrustVector6 rpm1 = rpm_buffer_[i+1].second;

            // Linear interpolation
            double alpha = (t_meas - t0) / (t1 - t0);
            interpolated_rpm = (1 - alpha) * rpm0 + alpha * rpm1;
            return;
        }
    }
}

void EkfNode::poseCallback(const Odometry &msg)
{
    if (!is_first_callback_)
    {
        // Initialize the previous time
        is_first_callback_ = true;
        return;  // Skip the first callback to avoid zero dt
    }
    t_curr_ = msg.header.stamp.toSec();

    RotorThrustVector6 rpm_interpolated_double;
    interpolate_rpm(rpm_interpolated_double, t_prev_);
    rpm_ << rpm_interpolated_double(0),
            rpm_interpolated_double(1),
            rpm_interpolated_double(2),
            rpm_interpolated_double(3),
            rpm_interpolated_double(4),
            rpm_interpolated_double(5);

    double dt = t_curr_ - t_prev_;

    if (dt <= 0.0)
    {
        ROS_WARN("Received non-positive dt: %f seconds. Skipping EKF update.", dt);
        // return;  // Skip the update if dt is non-positive
        std::cout << "dt: "<< dt << " seconds";
        // std::cout<< rpm_.transpose() << std::endl; 
    }
    // std::cout<< rpm_.transpose() << std::endl;

    QuatType q_meas(msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z);

    Vec3 w_meas(msg.twist.twist.angular.x,
           msg.twist.twist.angular.y,
           msg.twist.twist.angular.z);

    MeasVector7 z_meas;
    z_meas << q_meas, w_meas;

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

void EkfNode::setParam(const std::string param_name, EKFParams &ekf_params)
{
    size_t num_state;

    std::string node_name = ros::this_node::getName();

    std::string param_total_name = node_name + param_name;

    if (param_total_name == (node_name + "/ekf_param/P"))
    {
        num_state = 10;
        std::vector<double> param_vector;
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
        num_state = 10;
        std::vector<double> param_vector;
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
        num_state = 7;
        std::vector<double> param_vector;
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

    mav_param.m = m;  // Mass
    mav_param.J << moi_xx, 0.0, 0.0,
                   0.0, moi_yy, 0.0,
                   0.0, 0.0, moi_zz;  // Inertia matrix

    ROS_INFO("MavParam set: m = %f, moi = [%f, %f, %f]", 
             mav_param.m, moi_xx, moi_yy, moi_zz);

}