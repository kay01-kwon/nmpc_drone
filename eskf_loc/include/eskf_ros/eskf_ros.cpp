#include "eskf_ros.h"
#include <iomanip>

ESKF_ROS::ESKF_ROS(ros::NodeHandle &nh) : nh_(nh)
{

    EskfLocParams params;

    set_param(params);

    eskf_loc_ = new EskfLoc(params);

    bool nodelay = true;

    ros::TransportHints transport_hint;
    transport_hint = ros::TransportHints()
                    .tcpNoDelay(nodelay);

    imu_sub_ = nh_.subscribe("/mavros/imu/data_raw", 1000, 
    &ESKF_ROS::imu_callback, this, transport_hint);

    pose_sub_ = nh_.subscribe("/mocap/pose", 1000, 
    &ESKF_ROS::pose_callback, this, transport_hint);

    pub_timer_ = nh_.createTimer(ros::Duration(0.01), 
    &ESKF_ROS::publish_current_state, this);

    state_pub_ = nh_.advertise<nav_msgs::Odometry>("/eskf/Odom", 10);

    State s;

    Vec3d p_init, v_init;
    Quatd q_init;
    Vec3d ab_init, wb_init;
    Vec3d g_init;

    p_init.setZero();
    v_init.setZero();
    q_init.setZero();
    q_init(0) = 1.0;
    ab_init.setZero();
    wb_init.setZero();
    double g = 9.81;
    g_init << 0.0, 0.0, -g;

    // Initialize state and covariance

    eskf_data_.s << p_init, v_init, q_init, ab_init, wb_init, g_init;
    eskf_data_.P = params.P_init;

    eskf_msg_.header.frame_id = "world";
    eskf_msg_.child_frame_id = "eskf_odom";

    put_eskf_data_to_msg();

    // Buffer setup
    imu_buffer_ = CircularBuffer<ImuData>(50);
    pose_buffer_ = CircularBuffer<PoseData>(50);

    u_lpf_ << 0,0,0,0,0,9.81;

}

ESKF_ROS::~ESKF_ROS()
{
    std::cout<<"ESKF_ROS destructor called."<<std::endl;

    delete eskf_loc_;
    
}

void ESKF_ROS::run()
{
    ros::spin();

}

void ESKF_ROS::imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    ImuData imu_data;
    imu_data.time_stamp = imu_msg->header.stamp.toSec();
    imu_data.u.head<3>() = Vec3d(imu_msg->linear_acceleration.x,
                                 imu_msg->linear_acceleration.y,
                                 imu_msg->linear_acceleration.z);

    imu_data.u.tail<3>() = Vec3d(imu_msg->angular_velocity.x,
                                 imu_msg->angular_velocity.y,
                                 imu_msg->angular_velocity.z);

    if(!imu_buffer_.full())
    {
        imu_buffer_.push(imu_data);
    }
    else
    {
        imu_buffer_.pop();
        imu_buffer_.push(imu_data);
    }

    // ROS_INFO("imu callback");

    if(imu_buffer_.size() < 3)
    {
        return;
    }

    size_t imu_head = imu_buffer_.get_head_idx();
    ROS_ASSERT(imu_head > 1);
    dt_imu_debug_ = imu_buffer_[imu_head].time_stamp - imu_buffer_[imu_head-1].time_stamp;
    dt_imu_debug_ = dt_imu_debug_*1000.0;
    imu_ready_ = true;

    // if(is_first_estimate_)
    // {
    //     Vec6d u0, u1, um;
    //     u1 = imu_buffer_.back().u;
    //     u0 = imu_buffer_[imu_head-1].u;
    //     um = 0.5*(u0 + u1);
    //     u_lpf_ = 0.99*u_lpf_ + 0.01*um;
    //     double dt = imu_buffer_.back().time_stamp - imu_buffer_[imu_head-1].time_stamp;
    //     eskf_loc_->propagate(um, eskf_data_.s, eskf_data_.P, dt);
    // }


}

void ESKF_ROS::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    // std::lock_guard<std::mutex> lock(m_buf_);
    PoseData pose_data;

    pose_data.time_stamp = pose_msg->header.stamp.toSec();
    pose_data.p = Eigen::Vector3d(pose_msg->pose.position.x,
                                  pose_msg->pose.position.y,
                                  pose_msg->pose.position.z);
    pose_data.q = Quatd(pose_msg->pose.orientation.w,
                        pose_msg->pose.orientation.x,
                        pose_msg->pose.orientation.y,
                        pose_msg->pose.orientation.z);
    
    if(!pose_buffer_.full())
    {
        pose_buffer_.push(pose_data);
    }
    else
    {
        pose_buffer_.pop();
        pose_buffer_.push(pose_data);
    }

    if(pose_buffer_.size() < 3)
    {
        return;
    }

    size_t pose_head = pose_buffer_.get_head_idx();
    ROS_ASSERT(pose_head > 1);
    dt_pose_debug_ = pose_buffer_[pose_head].time_stamp - pose_buffer_[pose_head-1].time_stamp;
    dt_pose_debug_ = dt_pose_debug_*1000.0;


    // Initialize eskf state with the first pose measurement
    if(!is_first_estimate_)
    {
        eskf_data_.s(0) = pose_data.p(0);
        eskf_data_.s(1) = pose_data.p(1);
        eskf_data_.s(2) = pose_data.p(2);

        eskf_data_.s(6) = pose_data.q(0);
        eskf_data_.s(7) = pose_data.q(1);
        eskf_data_.s(8) = pose_data.q(2);
        eskf_data_.s(9) = pose_data.q(3);
        is_first_estimate_ = true;
    }

    // ROS_INFO("pose callback");

    // pose_ready_ = true;
    // cvBuf_.notify_all();

    // Meas z_meas;
    // z_meas.p_meas = pose_buffer_.back().p;
    // z_meas.q_meas = pose_buffer_.back().q;

    // eskf_loc_->correct(z_meas, eskf_data_.s, eskf_data_.P);

}

void ESKF_ROS::find_past_imu_data(size_t &idx0, const double &t)
{
    double epsilon = 0.002; // 1 ms tolerance
    size_t imu_head = imu_buffer_.get_head_idx();
    
    idx0 = imu_head;

    if(imu_buffer_[imu_head].time_stamp <= t - epsilon)
    {
        idx0 = imu_head;
        ROS_ASSERT(idx0 == imu_head);
        ROS_ASSERT(idx0 >= 0);
        return;
    }

    if(imu_buffer_[0].time_stamp >= t + epsilon)
    {
        idx0 = 0;
        ROS_ASSERT(idx0 == 0);
        return;
    }

    for(size_t i = 0; i <= imu_head; ++i)
    {
        ROS_ASSERT(i >= 0);
        if((imu_buffer_[i].time_stamp <= t + epsilon)
        && (imu_buffer_[i].time_stamp >= t - epsilon))
        {
            idx0 = i;
            // ROS_INFO("Found imu data at idx0: %ld", idx0);
            ROS_ASSERT(idx0 >= 0);
            break;
        }
    }
    return;
}

void ESKF_ROS::put_eskf_data_to_msg()
{

    ros::Time time_stamp;
    time_stamp.fromSec(t_est_now_);
    eskf_msg_.header.stamp = time_stamp;

    eskf_msg_.pose.pose.position.x = eskf_data_.s(0);
    eskf_msg_.pose.pose.position.y = eskf_data_.s(1);
    eskf_msg_.pose.pose.position.z = eskf_data_.s(2);

    eskf_msg_.twist.twist.linear.x = eskf_data_.s(3);
    eskf_msg_.twist.twist.linear.y = eskf_data_.s(4);
    eskf_msg_.twist.twist.linear.z = eskf_data_.s(5);

    eskf_msg_.pose.pose.orientation.w = eskf_data_.s(6);
    eskf_msg_.pose.pose.orientation.x = eskf_data_.s(7);
    eskf_msg_.pose.pose.orientation.y = eskf_data_.s(8);
    eskf_msg_.pose.pose.orientation.z = eskf_data_.s(9);

    if(imu_buffer_.empty())
    {
        eskf_msg_.twist.twist.angular.x = 0.0;
        eskf_msg_.twist.twist.angular.y = 0.0;
        eskf_msg_.twist.twist.angular.z = 0.0;
        return;
    }

    // Subtract bias from angular velocity
    eskf_msg_.twist.twist.angular.x = imu_buffer_.back().u(3) - eskf_data_.s(13);
    eskf_msg_.twist.twist.angular.y = imu_buffer_.back().u(4) - eskf_data_.s(14);
    eskf_msg_.twist.twist.angular.z = imu_buffer_.back().u(5) - eskf_data_.s(15);


}

void ESKF_ROS::publish_current_state(const ros::TimerEvent&)
{
    if(pose_buffer_.empty() || imu_buffer_.empty())
    {
        // ROS_WARN("[ESKF_ROS] Waiting for more data to publish...");
        return;
    }

    double t_pose_latest = pose_buffer_.back().time_stamp;
    double t_imu_latest = imu_buffer_.back().time_stamp;

    // ROS_INFO("dt_imu_debug_: %.2f ms, dt_pose_debug: %.2f ms", 
    // dt_imu_debug_, dt_pose_debug_);

    t_est_now_ = std::max(t_pose_latest, 
    t_imu_latest);

    if(!is_time_ready_)
    {
        t_est_old_ = t_est_now_;
        is_time_ready_ = true;
        return;
    }

    if(t_pose_latest >= t_imu_latest)
    {
        // ROS_INFO("Propagate + Correct");
        size_t idx0;
        find_past_imu_data(idx0, t_est_old_);
        Vec6d u0;
        u0 = imu_buffer_[idx0].u;
        double dt1 = t_pose_latest - t_est_old_;
        double t_diff = t_pose_latest - t_imu_latest;
        // ROS_INFO("idx0: %ld, pose - imu: %.3f ms, dt1: %.3f ms", 
        // idx0, t_diff*1000.0, dt1*1000.0);
        // ROS_INFO("dt1: %.3f ms", dt1*1000.0);
        if(idx0 < imu_buffer_.get_head_idx())
        {
            u0 = 0.5*(imu_buffer_[idx0].u + imu_buffer_[idx0+1].u);
        }
        eskf_loc_->propagate(u0, eskf_data_.s, eskf_data_.P, dt1);
        Meas z_meas;
        z_meas.p_meas = pose_buffer_.back().p;
        z_meas.q_meas = pose_buffer_.back().q;
        eskf_loc_->correct(z_meas, eskf_data_.s, eskf_data_.P);
        ROS_ASSERT(!isnan(eskf_data_.s(9)));
    }
    else
    {
        // ROS_INFO("Only Propagate");
        // ROS_INFO("t_imu_latest - t_pose_latest: %.3f ms",
        // (t_imu_latest - t_pose_latest)*1000.0);
        size_t idx0;
        find_past_imu_data(idx0, t_est_old_);
        Vec6d u0;

        ROS_ASSERT(idx0 >= 0 && idx0 <= imu_buffer_.size());
        u0 = imu_buffer_[idx0].u;
        if(idx0 < imu_buffer_.get_head_idx())
        {
            u0 = 0.5*(imu_buffer_[idx0].u + imu_buffer_[idx0+1].u);
        }
        double dt1 = t_imu_latest - t_est_old_;
        // ROS_INFO("dt: %f3 ms", dt1*1000.0);
        double t_diff = t_imu_latest - t_pose_latest;
        ROS_INFO("idx0: %ld, imu - pose: %.3f ms, dt1: %.3f ms", 
        idx0, t_diff*1000.0, dt1*1000.0);

        eskf_loc_->propagate(u0, eskf_data_.s, eskf_data_.P, dt1);
        ROS_ASSERT(!isnan(eskf_data_.s(9)));
    }
    put_eskf_data_to_msg();
    // ROS_INFO("dt : %.2f ms", dt*1000.0);



    double qw, qx, qy , qz;
    double px, py, pz;

    qw = eskf_msg_.pose.pose.orientation.w;
    qx = eskf_msg_.pose.pose.orientation.x;
    qy = eskf_msg_.pose.pose.orientation.y;
    qz = eskf_msg_.pose.pose.orientation.z;

    px = eskf_msg_.pose.pose.position.x;
    py = eskf_msg_.pose.pose.position.y;
    pz = eskf_msg_.pose.pose.position.z;

    // Broadcasting the transform between "world" and "eskf_odom"
    static tf::TransformBroadcaster br;

    br.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::Quaternion(qx, qy, qz, qw),
                tf::Vector3(px, py, pz)
            ),
            ros::Time::now(),
            "world",
            "eskf_odom"
        )
    );

    state_pub_.publish(eskf_msg_);
    t_est_old_ = t_est_now_;
}



void ESKF_ROS::set_param(EskfLocParams &params)
{
    std::string node_name;
    node_name = ros::this_node::getName();

    std::vector<double> P(18, 0.0);
    nh_.getParam(node_name + "/covariance/P", P);

    std::vector<double> R(7, 0.0);
    nh_.getParam(node_name + "/measurement_noise/R", R);

    double sigma_accel_n;
    double sigma_accel_w;
    double sigma_gyro_n;
    double sigma_gyro_w;

    nh_.param(node_name + "/process_noise/sigma_accel_n", sigma_accel_n, 0.01);
    nh_.param(node_name + "/process_noise/sigma_accel_w", sigma_accel_w, 0.01);

    nh_.param(node_name + "/process_noise/sigma_gyro_n", sigma_gyro_n, 0.01);
    nh_.param(node_name + "/process_noise/sigma_gyro_w", sigma_gyro_w, 0.01);

    std::setprecision(6);

    params.P_init.setZero();
    
    std::cout << "Setting ESKF_ROS parameters:" << std::endl;

    std::cout << "Covariance P_init:" << std::endl;
    for(size_t i = 0; i < P.size(); ++i)
    {
        params.P_init(i, i) = P[i];
        std::cout << "P_init(" << i <<", ";
        std::cout << i << "): " << P[i] << std::endl;
    }

    std::cout << "Measurement noise R:" << std::endl;
    for(size_t i = 0; i < R.size(); ++i)
    {
        params.measurement_noise_cov(i,i) = R[i];
        std::cout << "R(" << i <<", " << i;
        std::cout << "): " << R[i] << std::endl;
    }

    params.sigma_a_n = sigma_accel_n;
    params.sigma_a_w = sigma_accel_w;
    params.sigma_w_n = sigma_gyro_n;
    params.sigma_w_w = sigma_gyro_w;

    std::cout << " IMU Noise parameters:" << std::endl;
    std::cout << "  sigma_a_n: " << params.sigma_a_n << std::endl;
    std::cout << "  sigma_a_w: " << params.sigma_a_w << std::endl;
    std::cout << "  sigma_w_n: " << params.sigma_w_n << std::endl;
    std::cout << "  sigma_w_w: " << params.sigma_w_w << std::endl;

}