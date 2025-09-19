#include "hgdo_node_v2.h"
#include "utils/interpolate_tool.h"
#include "utils/quaternion_utils.h"

HgdoNode2::HgdoNode2(ros::NodeHandle &nh)
:nh_(nh)
{
    MavParam mav_param;
    HgdoParam hgdo_param;

    // Get publish rate from parameter server
    double publish_rate;
    std::string node_name = ros::this_node::getName();
    std::string publish_rate_param_name = node_name + "/publish_rate";
    nh_.param(publish_rate_param_name, publish_rate, 100.0);

    std::string tf_required_param_name = node_name + "/linear_vel_tf_required";
    nh_.param(tf_required_param_name, linear_vel_transform_required_, false);
    ROS_INFO("HGDO publish rate: %f Hz", publish_rate);
    ROS_INFO("HGDO linear vel transform required: %s", 
             linear_vel_transform_required_ ? "true" : "false");
    double duration = 1.0/publish_rate;
    period_ = duration;

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
    hgdo_est_thread_ = thread(&HgdoNode2::processState, this);

    ros::TransportHints transport_hint;
    transport_hint = ros::TransportHints()
                    .tcpNoDelay(true);

    rpm_sub_ = nh_.subscribe("/uav/actual_rpm", 100, 
    &HgdoNode2::rpmCallback, this, transport_hint);
    odom_sub_ = nh_.subscribe("/mavros/odometry/in", 100, 
    &HgdoNode2::stateCallback, this, transport_hint);

    wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/hgdo/wrench", 10);
    publish_timer_ = nh_.createTimer(ros::Duration(duration), &HgdoNode2::publishCallback, this);

    state_buffer_ = CircularBuffer<StateData>(10);
    rpm_buffer_ = CircularBuffer<RpmData>(10);

    time_latest_[0] = -1e300;    // rpm
    time_latest_[1] = -1e300;    // state

    time_out_[0] = 0.015;  // rpm   15 ms
    time_out_[1] = 0.015; // state  15 ms
    
    last_rx_wall_[0] = 0;   // rpm
    last_rx_wall_[1] = 0;   // state

    f_tau_ext_.setZero();
}

HgdoNode2::~HgdoNode2()
{
    if (hgdo_est_thread_.joinable()) {
        hgdo_est_thread_.join();
    }
    if(hgdo_dist_est_ != nullptr)
        delete hgdo_dist_est_;
    if(converter_ != nullptr)
        delete converter_;
}

void HgdoNode2::run()
{
    ros::spin();
}

void HgdoNode2::rpmCallback(const ros_libcanard::hexa_actual_rpm &msg)
{
    std::lock_guard<mutex> lock(mBuf_);
    RpmData rpm_data;
    rpm_data.time_stamp = msg.stamp.toSec();
    for(int i = 0; i < 6; ++i)
    {
        rpm_data.rpm[i] = msg.rpm[i];
    }

    if(!rpm_buffer_.full())
    {
        rpm_buffer_.push(rpm_data);
    }
    else
    {
        rpm_buffer_.pop();
        rpm_buffer_.push(rpm_data);
    }

    time_latest_[0] = std::max(time_latest_[0], rpm_data.time_stamp);
    last_rx_wall_[0] = ros::WallTime::now().toSec();
    cv_.notify_one();
}

void HgdoNode2::stateCallback(const Odometry &msg)
{
    std::lock_guard<mutex> lock(mBuf_);
    StateData state_data;
    state_data.time_stamp = msg.header.stamp.toSec();

    state_data.p << msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z;


    if(linear_vel_transform_required_)
    {
        Vec3d v_body, v_world;
        v_body << msg.twist.twist.linear.x,
                  msg.twist.twist.linear.y,
                  msg.twist.twist.linear.z;
        Quatd q;
        q << msg.pose.pose.orientation.w,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z;
        Mat3x3 R = quaternion_to_rotm(q);
        v_world = R * v_body;

        state_data.v = v_world;
    }
    else
    {
        state_data.v << msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z;
    }

    state_data.w << msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z;
    
    state_data.q << msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z;

    if(!state_buffer_.full())
    {
        state_buffer_.push(state_data);
    }
    else
    {
        state_buffer_.pop();
        state_buffer_.push(state_data);
    }

    time_latest_[1] = std::max(time_latest_[1], state_data.time_stamp);
    last_rx_wall_[1] = ros::WallTime::now().toSec();
    cv_.notify_one();
}

void HgdoNode2::publishCallback(const ros::TimerEvent& event)
{
    std::lock_guard<mutex> lock(mProc_);
    wrench_msg_.header.stamp = ros::Time(t_curr_);
    wrench_msg_.wrench.force.x = f_tau_ext_[0];
    wrench_msg_.wrench.force.y = f_tau_ext_[1];
    wrench_msg_.wrench.force.z = f_tau_ext_[2];

    wrench_msg_.wrench.torque.x = f_tau_ext_[3];
    wrench_msg_.wrench.torque.y = f_tau_ext_[4];
    wrench_msg_.wrench.torque.z = f_tau_ext_[5];

    wrench_pub_.publish(wrench_msg_);
}

void HgdoNode2::processState()
{
    ROS_INFO("Start Hgdo estimation thread");

    const int MAX_CATCHUP = 3;
    int step = 0;

    t_curr_ = ros::Time::now().toSec();
    t_prev_ = t_curr_;

    ROS_INFO("Initial t_prev: %.3f", t_prev_);

    bool first_run = false;


    while(ros::ok())
    {
        if(state_buffer_.empty() || rpm_buffer_.empty())
        {
            // ROS_INFO("Wait for more data...");
            std::chrono::milliseconds duration(100);
            std::this_thread::sleep_for(duration);
            continue;
        }


        std::unique_lock<mutex> lk(mBuf_);
        cv_.wait_for(lk,
        std::chrono::duration<double>(2*period_),
        [this]
        {
            if(!ros::ok())
                ROS_INFO("Shutting down Hgdo estimation thread...");
            return (watermark_time() - t_prev_ >= period_);
        });

        if(!first_run)
        {
            t_curr_ = state_buffer_.back().time_stamp;
            t_prev_ = t_curr_ - period_;
            first_run = true;
        }

        double w = watermark_time();

        step = 0;

        while((t_prev_ + period_ <= w) && step < MAX_CATCHUP)
        {
            t_curr_ = t_prev_ + period_;

            size_t idx_rpm_curr = 0;
            size_t idx_state_curr = 0;

            bool rpm_interval_available;
            bool state_interval_available;

            rpm_interval_available = getRpmInterval(t_prev_, t_curr_, idx_rpm_curr);
            state_interval_available = getStateInterval(t_prev_, t_curr_, idx_state_curr);


            // ROS_INFO("t_curr: %.3f, rpm_back_time: %.3f, state_back_time: %.3f", 
            //          t_curr_, rpm_buffer_.back().time_stamp, state_buffer_.back().time_stamp);
            // ROS_INFO("rpm idx: %zu, state idx: %zu", idx_rpm_curr, idx_state_curr);
            // ROS_INFO("rpm size: %zu, state size: %zu", rpm_buffer_.size(), state_buffer_.size());
            // ROS_INFO("Hgdo update: t_prev = %.3f, t_curr = %.3f", t_prev_, t_curr_);
            
            // ROS_INFO("rpm available: %s, state available: %s", 
            //          rpm_interval_available ? "true" : "false",
            //          state_interval_available ? "true" : "false");

            if(rpm_interval_available && state_interval_available)
            {
                Vec4d u, u0, u1;
                converter_->convert_rpm_to_control_input(rpm_buffer_.front().rpm, u0);
                converter_->convert_rpm_to_control_input(rpm_buffer_.back().rpm, u1);
                u = 0.5*(u0 + u1);

                State state_curr;

                state_curr << state_buffer_[idx_state_curr].p,
                              state_buffer_[idx_state_curr].v,
                              state_buffer_[idx_state_curr].q,
                              state_buffer_[idx_state_curr].w;

                hgdo_dist_est_->updateStateControlTime(
                state_curr, u, t_prev_, t_curr_    
                );

                {
                    std::lock_guard<mutex> lock(mProc_);
                    hgdo_dist_est_->getDisturbance(f_tau_ext_);
                }

            }

            t_prev_ = t_curr_;
            step++;
        }

        lk.unlock();

    }
}

bool HgdoNode2::getRpmInterval(double &t_start, double &t_end, size_t &idx_end)
{
    if(rpm_buffer_.empty())
    {
        return false;
    }

    while(rpm_buffer_.front().time_stamp <= t_start)
    {
        rpm_buffer_.pop();
        if(rpm_buffer_.empty())
        {
            return false;
        }
    }


    for(int i = 0; i < rpm_buffer_.size(); ++i)
    {
        if(rpm_buffer_[i].time_stamp >= t_end)
        {
            idx_end = i;
            return true;
        }
    }

    return false;
}

bool HgdoNode2::getStateInterval(double &t_start, double &t_end, size_t &idx_end)
{
    if(state_buffer_.empty())
    {
        return false;
    }

    while(state_buffer_.front().time_stamp <= t_start)
    {
        state_buffer_.pop();
        if(state_buffer_.empty())
        {
            return false;
        }
    }

    for(int i = 0; i < state_buffer_.size(); ++i)
    {
        if(state_buffer_[i].time_stamp >= t_end)
        {
            idx_end = i;
            return true;
        }
    }

    return false;
}

double HgdoNode2::watermark_time()
{
    double watermark;

    watermark = time_latest_[0];

    for(int i = 0; i < 2; ++i)
    {
        if(freshByTTL(i, ros::WallTime::now().toSec()))
        {
            watermark = std::min(watermark, time_latest_[i]);
        }
    }
    return watermark;
}

bool HgdoNode2::freshByTTL(int i, double now_wall)
{
    return (now_wall - last_rx_wall_[i] <= time_out_[i]);
}

void HgdoNode2::setParam(const std::string param_name, MavParam &mav_param)
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

void HgdoNode2::setParam(const std::string param_name, HgdoParam &hgdo_param)
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