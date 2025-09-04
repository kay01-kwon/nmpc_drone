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

    imu_sub_ = nh_.subscribe("/mavros/imu/data_raw", 1, 
    &ESKF_ROS::imu_callback, this, transport_hint);

    pose_sub_ = nh_.subscribe("/mocap/pose", 1, 
    &ESKF_ROS::pose_callback, this, transport_hint);

    pub_timer_ = nh_.createTimer(ros::Duration(0.01), 
    &ESKF_ROS::publish_current_state, this);

    state_pub_ = nh_.advertise<nav_msgs::Odometry>("/eskf/Odom", 1);

    ekf_estimate_thread_ = std::thread(&ESKF_ROS::estimate, this);

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


    // Buffer setup
    imu_buffer_ = CircularBuffer<ImuData>(20);
    pose_buffer_ = CircularBuffer<PoseData>(20);

}

ESKF_ROS::~ESKF_ROS()
{
    std::cout<<"ESKF_ROS destructor called."<<std::endl;
    if(ekf_estimate_thread_.joinable())
    {
        ekf_estimate_thread_.join();
    }

    delete eskf_loc_;
    
}

void ESKF_ROS::run()
{
    ros::spin();

}

void ESKF_ROS::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(m_buf_);
    ImuData imu_data;
    imu_data.time_stamp = msg->header.stamp.toSec();
    imu_data.u.head<3>() = Vec3d(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);

    imu_data.u.tail<3>() = Vec3d(msg->angular_velocity.x,
                                msg->angular_velocity.y,
                                msg->angular_velocity.z);

    if(!imu_buffer_.full())
    {
        imu_buffer_.push(imu_data);
    }
    else
    {
        imu_buffer_.pop();
        imu_buffer_.push(imu_data);
    }

    if(imu_buffer_.size() < 5)
    {
        ROS_WARN("[ESKF_ROS] Waiting for more IMU data...");
        return;
    }

    size_t imu_head = imu_buffer_.get_head_idx();

    dt_imu_debug_ = imu_buffer_[imu_head].time_stamp - imu_buffer_[imu_head-1].time_stamp;
    dt_imu_debug_ = dt_imu_debug_*1000.0;

    imu_ready_ = true;
    cvBuf_.notify_all();

}

void ESKF_ROS::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(m_buf_);
    double t_pose = msg->header.stamp.toSec();
    PoseData pose_data;

    pose_data.time_stamp = t_pose;
    pose_data.p = Eigen::Vector3d(msg->pose.position.x,
                                  msg->pose.position.y,
                                  msg->pose.position.z);
    pose_data.q = Quatd(msg->pose.orientation.w,
                       msg->pose.orientation.x,
                       msg->pose.orientation.y,
                       msg->pose.orientation.z);
    
    if(!pose_buffer_.full())
    {
        pose_buffer_.push(pose_data);
    }
    else
    {
        pose_buffer_.pop();
        pose_buffer_.push(pose_data);
    }

    if(pose_buffer_.size() < 5)
    {
        ROS_WARN("[ESKF_ROS] Waiting for more pose data...");
        return;
    }

    size_t pose_head = pose_buffer_.get_head_idx();
    dt_pose_debug_ = pose_buffer_[pose_head].time_stamp - pose_buffer_[pose_head-1].time_stamp;
    dt_pose_debug_ = dt_pose_debug_*1000.0;

    pose_ready_ = true;
    cvBuf_.notify_all();
}

void ESKF_ROS::estimate()
{
    double t_est_now = ros::Time::now().toSec();
    double t_est_old = t_est_now;
    double dt_est = 0.0;
    ROS_INFO("[ESKF_ROS] ESKF estimation thread started.");
    while(ros::ok())
    {

        std::unique_lock<std::mutex> lock(m_buf_);
        cvBuf_.wait(lock, 
        [this]{
            return (imu_ready_ && pose_ready_);
        }
        );

        if(dt_imu_debug_ > 5.0)
            ROS_INFO("dt_imu: %.2f ms, dt_pose: %.2f ms", dt_imu_debug_, dt_pose_debug_);
        imu_ready_ = false;
        pose_ready_ = false;

        size_t imu_head = imu_buffer_.get_head_idx();
        size_t pose_head = pose_buffer_.get_head_idx();

        double t_imu_latest = imu_buffer_[imu_head].time_stamp;
        double t_pose_latest = pose_buffer_[pose_head].time_stamp;
        double t_diff = t_pose_latest - t_imu_latest;

        double epsilon = 0.010; // 10 ms tolerance

        if(t_diff >= 0.0)
        {
            t_est_now = t_pose_latest;
            dt_est = t_est_now - t_est_old;

            int imu_idx = -1;
            for(size_t i = imu_head; i > 0; --i)
            {
                if(imu_buffer_[i].time_stamp <= t_est_old + epsilon &&
                   imu_buffer_[i].time_stamp >= t_est_old - epsilon)
                {
                    imu_idx = i;
                    break;
                }
            }
            if(imu_idx == -1)
            {
                continue;
            }
            else
            {
                Vec6d u;
                u = imu_buffer_[imu_idx].u;
                eskf_loc_->propagate(u, eskf_data_.s, eskf_data_.P, dt_est);
                Meas z;
                z.p_meas = pose_buffer_[pose_head].p;
                z.q_meas = pose_buffer_[pose_head].q;
                eskf_loc_->correct(z, eskf_data_.s, eskf_data_.P);
            }

        }
        else
        {
            t_est_now = t_imu_latest;

        }

        // ROS_INFO("[ESKF_ROS] t_diff: %.4f s", t_diff*1000.0);

        
        t_est_old = t_est_now;


        // ROS_INFO(" dt_est: %.2f ms", dt_est*1000.0);

        assert(!isnan(eskf_data_.s(9)));

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

        // eskf_msg_.twist.twist.angular.x = 

    }
    
}

void ESKF_ROS::publish_current_state(const ros::TimerEvent&)
{

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

    // br.sendTransform(
    //     tf::StampedTransform(
    //         tf::Transform(
    //             tf::Quaternion(qx, qy, qz, qw),
    //             tf::Vector3(px, py, pz)
    //         ),
    //         ros::Time::now(),
    //         "world",
    //         "eskf_odom"
    //     )
    // );

    state_pub_.publish(eskf_msg_);
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

    nh_.param(node_name + "/process_noise/sigma_accel_n", sigma_accel_n, 0.0);
    nh_.param(node_name + "/process_noise/sigma_accel_w", sigma_accel_w, 0.0);

    nh_.param(node_name + "/process_noise/sigma_gyro_n", sigma_gyro_n, 0.0);
    nh_.param(node_name + "/process_noise/sigma_gyro_w", sigma_gyro_w, 0.0);

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