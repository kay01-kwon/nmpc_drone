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
    hgdo_est_thread_ = thread(&HgdoNode::estimate, this);

    ros::TransportHints transport_hint;
    transport_hint = ros::TransportHints()
                    .tcpNoDelay(true);

    rpm_sub_ = nh_.subscribe("/uav/actual_rpm", 10, 
    &HgdoNode::rpmCallback, this, transport_hint);
    odom_sub_ = nh_.subscribe("/eskf/Odom", 10, 
    &HgdoNode::stateCallback, this, transport_hint);

    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("/hgdo/wrench", 1);
    publish_timer_ = nh_.createTimer(ros::Duration(duration), &HgdoNode::publishCallback, this);

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

    if(rpm_buffer_.size() <= 2)
    {
        ROS_INFO("Waiting for more rpm data...");
        return;
    }
    
    double eps = 0.005; // 5 ms tolerance for time synchronization
    if(t_input_ + eps <= state_data.time_stamp)
    {
        state_ready_ = true;
    }
}

void HgdoNode::estimate()
{
    while(ros::ok())
    {
        if(state_buffer_.size() <= 5 || rpm_buffer_.size() <= 5)
        {
            continue;
        }

        size_t rpm_head = rpm_buffer_.get_head_idx();
        t_input_ = rpm_buffer_[rpm_head].time_stamp;

        // Lock the buffer for recent state data
        std::unique_lock<mutex> lock(mBuf_);
        auto dead_line = std::chrono::system_clock::now()
        + std::chrono::milliseconds(7);

        if(cvBuf_.wait_until(lock, dead_line,[this]{return state_ready_;}))
        {
            size_t idx_rpm;

            for(size_t i = rpm_buffer_.get_head_idx(); i > 0; --i)
            {
                if(rpm_buffer_[i].time_stamp <= t_input_)
                {
                    idx_rpm = i;
                    break;
                }
            }

            size_t state_head = state_buffer_.get_head_idx();
            double t_new_state = state_buffer_[state_head].time_stamp;
            
            double eps = 1e-6;

            if(t_new_state - t_input_ <= eps && idx_rpm - 1 < 0)
            {
                ROS_WARN("No interpolation needed, t_new_state <= t_input");
                // Do nothing
                continue;
            }
            else
            {
                int idx_state_prev = -1;
                // Find the previous state index
                for(size_t i = state_head; i > 0; --i)
                {
                    if(state_buffer_[i].time_stamp <= t_input_ &&
                        state_buffer_[i].time_stamp >= rpm_buffer_[idx_rpm-1].time_stamp)
                    {
                        idx_state_prev = i;
                        break;
                    }
                }

                if(idx_state_prev == -1)
                {
                    ROS_WARN("No valid previous state found for interpolation.");
                    continue;
                }

                double t_prev_state = state_buffer_[idx_state_prev].time_stamp;
                Vec4d u0, u1, u_interpl;
                converter_->convert_rpm_to_control_input(rpm_buffer_[idx_rpm-1].rpm, u0);
                converter_->convert_rpm_to_control_input(rpm_buffer_[idx_rpm].rpm, u1);
                u_interpl = interpolate_vec4(rpm_buffer_[idx_rpm-1].time_stamp, u0,
                                            rpm_buffer_[idx_rpm].time_stamp, u1,
                                            t_prev_state);
                State s_prev;
                s_prev << state_buffer_[idx_state_prev].p,
                          state_buffer_[idx_state_prev].v,
                          state_buffer_[idx_state_prev].q,
                          state_buffer_[idx_state_prev].w;

                // ROS_INFO("dt: %f", t_new_state - t_prev_state);

                hgdo_dist_est_->updateStateControlTime(s_prev, u_interpl, t_prev_state, t_new_state);
                hgdo_dist_est_->getDisturbance(f_tau_ext_);
            }
            state_ready_ = false;
        }

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