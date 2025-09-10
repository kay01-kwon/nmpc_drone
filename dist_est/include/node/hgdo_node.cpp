#include "hgdo_node.h"
#include "utils/interpolate_tool.h"
HgdoNode::HgdoNode()
{
    // Default constructor
}

HgdoNode::HgdoNode(ros::NodeHandle &nh)
:nh_(nh)
{
    MavParam mav_param;
    HgdoParam hgdo_param;

    // Get publish rate from parameter server
    double publish_rate;
    std::string node_name = ros::this_node::getName();
    std::string publish_rate_param_name = node_name + "/publish_rate";
    nh_.param(publish_rate_param_name, publish_rate, 100.0);
    ROS_INFO("HGDO publish rate: %f Hz", publish_rate);
    double duration = 1.0/publish_rate;

    // Set hgdo parameters
    std::string hgdo_param_name = "/hgdo_param";
    setParam(hgdo_param_name, hgdo_param);

    // Set mav parameters
    std::string nominal_param_name = "/nominal_param";
    setParam(nominal_param_name, mav_param);

    // Initialize converter
    converter_ = new FDynamics(mav_param);

    // Initialize HGDO
    hgdo_dist_est_ = new HGDO(mav_param, hgdo_param);
    hgdo_est_thread_ = thread(&HgdoNode::processState, this);

    ros::TransportHints transport_hint;
    transport_hint = ros::TransportHints()
                    .tcpNoDelay(true);

    rpm_sub_ = nh_.subscribe("/uav/actual_rpm", 100, 
    &HgdoNode::rpmCallback, this, transport_hint);
    odom_sub_ = nh_.subscribe("/mavros/odometry/in", 100, 
    &HgdoNode::stateCallback, this, transport_hint);

    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("/hgdo/wrench", 10);
    publish_timer_ = nh_.createTimer(ros::Duration(duration), &HgdoNode::publishCallback, this);

    state_buffer_ = CircularBuffer<StateData>(20);
    rpm_buffer_ = CircularBuffer<RpmData>(20);

    f_tau_ext_.setZero();

}

HgdoNode::~HgdoNode()
{
    if (hgdo_est_thread_.joinable()) {
        hgdo_est_thread_.join();
    }
    if(hgdo_dist_est_ != nullptr) {
        delete hgdo_dist_est_;
    }
    if(converter_ != nullptr) {
        delete converter_;
    }
}

void HgdoNode::run()
{
    ros::spin();
}

void HgdoNode::rpmCallback(const ros_libcanard::hexa_actual_rpm &rpm_msg)
{
    std::lock_guard<mutex> lock(mBuf_);
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

}

void HgdoNode::stateCallback(const Odometry &state_msg)
{
    std::lock_guard<mutex> lock(mBuf_);

    StateData state_data;
    state_data.time_stamp = state_msg.header.stamp.toSec();

    state_data.p << state_msg.pose.pose.position.x,
                    state_msg.pose.pose.position.y,
                    state_msg.pose.pose.position.z;
                
    state_data.v << state_msg.twist.twist.linear.x,
                    state_msg.twist.twist.linear.y,
                    state_msg.twist.twist.linear.z;

    state_data.w << state_msg.twist.twist.angular.x,
                    state_msg.twist.twist.angular.y,
                    state_msg.twist.twist.angular.z;
    
    state_data.q << state_msg.pose.pose.orientation.w,
                    state_msg.pose.pose.orientation.x,
                    state_msg.pose.pose.orientation.y,
                    state_msg.pose.pose.orientation.z;

    if(state_buffer_.full())
    {
        state_buffer_.pop();
        state_buffer_.push(state_data);
    }
    else
    {
        state_buffer_.push(state_data);
    }
}

void HgdoNode::publishCallback(const ros::TimerEvent&)
{
    publishWrench();
}

void HgdoNode::publishWrench()
{
    wrench_msg_.force.x = f_tau_ext_(0);
    wrench_msg_.force.y = f_tau_ext_(1);
    wrench_msg_.force.z = f_tau_ext_(2);

    wrench_msg_.torque.x = f_tau_ext_(3);
    wrench_msg_.torque.y = f_tau_ext_(4);
    wrench_msg_.torque.z = f_tau_ext_(5);

    wrench_pub_.publish(wrench_msg_);
}


void HgdoNode::processState()
{
    
    while(ros::ok())
    {
        if(state_buffer_.empty() || rpm_buffer_.empty())
        {
            ROS_INFO("Wait for more data...");
            std::chrono::milliseconds duration(10);
            std::this_thread::sleep_for(duration);
            continue;
        }

        if(first_run_)
        {
            first_run_ = false;
            t_est_curr_ = state_buffer_.back().time_stamp;
            t_est_prev_ = -1;
            std::chrono::milliseconds duration(10);
            std::this_thread::sleep_for(duration);
            continue;
        }

        t_est_curr_ = state_buffer_.back().time_stamp;

        if(t_est_curr_ == t_est_prev_)
        {
            std::chrono::milliseconds duration(1);
            std::this_thread::sleep_for(duration);
            continue;
        }

        // ROS_ASSERT(t_est_curr_ > t_est_prev_);

        while(1)
        {
            if(RpmAvailable(t_est_curr_))
            {
                // ROS_INFO("RPM available!");
                break;
            }
            else
            {
                ROS_WARN("Wait for more rpm data...");
                std::chrono::milliseconds duration(2);
                std::this_thread::sleep_for(duration);
            }
        }

        // ROS_ASSERT(state_buffer_.empty() == false);
        // ROS_ASSERT(rpm_buffer_.empty() == false);
        

        mBuf_.lock();
        size_t idx_rpm_curr = 0;
        getRpmInterval(t_est_prev_, t_est_curr_, idx_rpm_curr);

        while(state_buffer_.front().time_stamp < t_est_prev_)
        {
            state_buffer_.pop();
        }

        StateData state_prev;
        state_prev.p = state_buffer_.front().p;
        state_prev.v = state_buffer_.front().v;
        state_prev.q = state_buffer_.front().q;
        state_prev.w = state_buffer_.front().w;

        Vec4d u_est;

        // ROS_INFO("idx_rpm_curr: %zu", idx_rpm_curr);
        // ROS_INFO("rpm buffer size: %zu", rpm_buffer_.size());
        // ROS_INFO("state buffer size: %zu", state_buffer_.size());

        if(idx_rpm_curr >= 1)
        {
            RpmData rpm_prev, rpm_curr;
            rpm_prev.rpm = rpm_buffer_.front().rpm;
            rpm_curr.rpm = rpm_buffer_.back().rpm;
            Vec4d u_prev, u_curr;
            converter_->convert_rpm_to_control_input(rpm_prev.rpm, u_prev);
            converter_->convert_rpm_to_control_input(rpm_curr.rpm, u_curr);
            u_est = (u_prev + u_curr) / 2.0;
        }
        else
        {
            RpmData rpm;
            rpm.rpm = rpm_buffer_.back().rpm;
            converter_->convert_rpm_to_control_input(rpm.rpm, u_est);
        }
        State s_prev;
        s_prev << state_prev.p, state_prev.v, state_prev.q, state_prev.w;
        hgdo_dist_est_->updateStateControlTime(s_prev, 
        u_est, t_est_prev_, t_est_curr_);
        mBuf_.unlock();

        hgdo_dist_est_->getDisturbance(f_tau_ext_);

        t_est_prev_ = t_est_curr_;

    }

}

bool HgdoNode::getRpmInterval(const double &t_prev, const double &t_curr,
                            size_t &idx_curr)
{
    if(rpm_buffer_.empty())
    {
        return false;
    }

    if(rpm_buffer_.back().time_stamp <= t_curr)
    {
        while(rpm_buffer_.front().time_stamp <= t_prev)
        {
            rpm_buffer_.pop();
        }

        for(size_t i = 0; i < rpm_buffer_.size(); i++)
        {
            if(rpm_buffer_[i].time_stamp >= t_curr)
            {
                idx_curr = i;
                return true;
            }
        }
    }
    else
    {
        ROS_WARN("Wait for more rpm data...");
    }
    return false;
}


bool HgdoNode::RpmAvailable(const double &t)
{
    if(rpm_buffer_.empty())
    {
        return false;
    }
    else if(!rpm_buffer_.empty() && rpm_buffer_.back().time_stamp >= t)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void HgdoNode::setParam(const std::string param_name, MavParam &mav_param)
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

void HgdoNode::setParam(const std::string param_name, HgdoParam &hgdo_param)
{
    std::string node_name = ros::this_node::getName();
    std::string param_total_name = node_name + param_name;

    double eps_f, eps_tau;

    nh_.param(param_total_name + "/eps_f", eps_f, 0.01);
    nh_.param(param_total_name + "/eps_tau", eps_tau, 0.01);

    hgdo_param.eps_f = eps_f;
    hgdo_param.eps_tau = eps_tau;

    ROS_INFO("HgdoParam set: eps_f = %f, eps_tau = %f", 
             hgdo_param.eps_f, hgdo_param.eps_tau);
}