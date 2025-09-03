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
    converter_ = new FDynamics(mav_param);

    // Initialize the EKF node with a NodeHandle
    rpm_sub_ = nh_.subscribe("/uav/actual_rpm", 1, &EkfNode::rpmCallback, this);
    pose_sub_ = nh_.subscribe("/eskf/Odom", 1, &EkfNode::stateCallback, this);

    publish_timer_ = nh_.createTimer(ros::Duration(duration), &EkfNode::publishCallback, this); // 100 Hz
    state_pub_ = nh_.advertise<nav_msgs::Odometry>("ekf/state", 1);
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("ekf/wrench", 1);

    ekf_est_thread_ = thread(&EkfNode::estimate, this);


    // State initialization
    Vec3d p0, v0;
    Quatd q0;
    Vec3d w0;
    Vec3d f_ext0, tau_ext0;

    p0.setZero();
    v0.setZero();
    q0 << 1.0, 0.0, 0.0, 0.0;
    w0.setZero();
    f_ext0.setZero();
    tau_ext0.setZero();

    ekf_data_.s << p0, v0, q0, w0, f_ext0, tau_ext0;

    ekf_data_.P = ekf_params.P;

}

EkfNode::~EkfNode()
{
    if (ekf_dist_est_ != nullptr)
    {
        delete ekf_dist_est_;
        ekf_dist_est_ = nullptr;
    }

    if (converter_ != nullptr)
    {
        delete converter_;
        converter_ = nullptr;
    }
    ekf_est_thread_.join();
}

void EkfNode::run()
{

    ros::spin();

}

void EkfNode::rpmCallback(const ros_libcanard::hexa_actual_rpm &rpm_msg)
{
    std::lock_guard<mutex> lk(mBuf_);
    RpmData rpm_data;
    rpm_data.time_stamp = rpm_msg.stamp.toSec();
    rpm_data.rpm << rpm_msg.rpm[0], rpm_msg.rpm[1], rpm_msg.rpm[2],
                    rpm_msg.rpm[3], rpm_msg.rpm[4], rpm_msg.rpm[5];
    if(rpm_buffer_.full())
    {
        rpm_buffer_.pop();
        rpm_buffer_.push(rpm_data);
    }
    else
    {
        rpm_buffer_.push(rpm_data);
    }

    if(state_buffer_.size() < 2)
    {
        ROS_INFO("Waiting state msg.");
        return;
    }
}

void EkfNode::stateCallback(const Odometry &odom_msg)
{
    std::lock_guard<mutex> lk(mBuf_);

    // ROS_INFO("State callback");

    StateData state_data;
    state_data.time_stamp = odom_msg.header.stamp.toSec();

    double eps = 0.005;

    state_data.p = Vec3d(odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        odom_msg.pose.pose.position.z);

    state_data.v = Vec3d(odom_msg.twist.twist.linear.x,
                        odom_msg.twist.twist.linear.y,
                        odom_msg.twist.twist.linear.z);

    state_data.q = Quatd(odom_msg.pose.pose.orientation.w,
                        odom_msg.pose.pose.orientation.x,
                        odom_msg.pose.pose.orientation.y,
                        odom_msg.pose.pose.orientation.z);
    
    state_data.w = Vec3d(odom_msg.twist.twist.angular.x,
                        odom_msg.twist.twist.angular.y,
                        odom_msg.twist.twist.angular.z);
                        
    if(state_buffer_.full())
    {
        state_buffer_.pop();
        state_buffer_.push(state_data);
    }
    else
    {
        state_buffer_.push(state_data);
    }

    if(rpm_buffer_.size() <= 2)
    {
        ROS_INFO("Waiting rpm msg.");
        return;
    }

    if( t_curr_  + eps <= state_data.time_stamp )
    {
        state_ready_ = true;
    }
}

void EkfNode::estimate()
{
    while(ros::ok())
    {
        if(state_buffer_.size() <= 2 || rpm_buffer_.size() <= 2)
        {
            // ROS_INFO("Waiting for data...");
            continue;
        }

        size_t rpm_head = rpm_buffer_.get_head_idx();
        t_curr_ = rpm_buffer_[rpm_head].time_stamp;

        std::unique_lock<mutex> lock(mBuf_);
        auto dead_line = std::chrono::steady_clock::now() 
        + std::chrono::milliseconds(10);

        if(cvBuf_.wait_until(lock, dead_line, [this]{ return state_ready_; }))
        {
            size_t idx_rpm;

            for(size_t i = rpm_buffer_.get_head_idx(); i > 0; --i)
            {
                if(rpm_buffer_[i].time_stamp == t_curr_)
                {
                    idx_rpm = i;
                    break;
                }
            }

            Vec4d u0;
            Vec4d u1;
            size_t state_head = state_buffer_.get_head_idx();
            double t_meas = state_buffer_[state_head].time_stamp;
            double dt = t_meas - t_curr_;

            double eps = 0.001;

            // ROS_INFO("t_curr_: %f, t_meas: %f, dt: %f", t_curr_, t_meas, dt);

            if(dt <= eps)
            {
                State s_meas;
                s_meas << state_buffer_[state_head].p,
                state_buffer_[state_head].v,
                state_buffer_[state_head].q,
                state_buffer_[state_head].w;

                ekf_dist_est_->correct(s_meas, ekf_data_);
                
            }
            else
            {
                converter_->convert_rpm_to_control_input(rpm_buffer_[idx_rpm-1].rpm, u0);
                converter_->convert_rpm_to_control_input(rpm_buffer_[idx_rpm].rpm, u1);
                Vec4d um = interpolate_vec4(rpm_buffer_[idx_rpm-1].time_stamp, u0,
                                                    rpm_buffer_[idx_rpm].time_stamp, u1,
                                                    t_meas);
                
                ekf_dist_est_->propagate(um, ekf_data_, dt);
                
                State s_meas;
                s_meas << state_buffer_[state_head].p,
                state_buffer_[state_head].v,
                state_buffer_[state_head].q,
                state_buffer_[state_head].w;
                ekf_dist_est_->correct(s_meas, ekf_data_);
            }
            state_ready_ = false;
        }



    }
}

void EkfNode::publishCallback(const ros::TimerEvent&)
{
    publishState();
    publishWrench();
}

void EkfNode::publishState()
{
    state_msg_.header.stamp = ros::Time::now();
    state_msg_.header.frame_id = "world";
    state_msg_.child_frame_id = "EKF";
    state_msg_.pose.pose.position.x = ekf_data_.s(0);
    state_msg_.pose.pose.position.y = ekf_data_.s(1);
    state_msg_.pose.pose.position.z = ekf_data_.s(2);

    state_msg_.twist.twist.linear.x = ekf_data_.s(3);
    state_msg_.twist.twist.linear.y = ekf_data_.s(4);
    state_msg_.twist.twist.linear.z = ekf_data_.s(5);
    
    state_msg_.pose.pose.orientation.w = ekf_data_.s(6);
    state_msg_.pose.pose.orientation.x = ekf_data_.s(7);
    state_msg_.pose.pose.orientation.y = ekf_data_.s(8);
    state_msg_.pose.pose.orientation.z = ekf_data_.s(9);

    state_msg_.twist.twist.angular.x = ekf_data_.s(10);
    state_msg_.twist.twist.angular.y = ekf_data_.s(11);
    state_msg_.twist.twist.angular.z = ekf_data_.s(12);

    state_pub_.publish(state_msg_);
}

void EkfNode::publishWrench()
{
    wrench_msg_.force.x = ekf_data_.s(13);
    wrench_msg_.force.y = ekf_data_.s(14);
    wrench_msg_.force.z = ekf_data_.s(15);

    wrench_msg_.torque.x = ekf_data_.s(16);
    wrench_msg_.torque.y = ekf_data_.s(17);
    wrench_msg_.torque.z = ekf_data_.s(18);

    wrench_pub_.publish(wrench_msg_);
}

Vec4d EkfNode::interpolate_vec4(const double &t0,
                             const Vec4d &v0,
                             const double &t1,
                             const Vec4d &v1,
                             const double &tm)
{
    double alpha = (tm - t0) / (t1 - t0);
    Vec4d vm;
    vm = (1 - alpha) * v0 + alpha * v1;
    return vm;
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