#include "eskf_ros.h"
#include <iomanip>

ESKF_ROS::ESKF_ROS(ros::NodeHandle &nh) : nh_(nh)
{

    EskfLocParams params;

    set_param(params);

    eskf_loc_ = new EskfLoc(params);

    imu_sub_ = nh_.subscribe("/mavros/imu/data_raw", 1, &ESKF_ROS::imu_callback, this);
    pose_sub_ = nh_.subscribe("/mocap/pose", 1, &ESKF_ROS::pose_callback, this);

    state_pub_ = nh_.advertise<nav_msgs::Odometry>("/eskf/Odom", 1);

    Vec3 p_init, v_init;
    Quat q_init;
    Vec3 ab_init, wb_init;
    Vec3 g_init;

    double g = 9.81;
    g_init << 0.0, 0.0, -g;

    p_init.setZero();
    v_init.setZero();
    q_init.setZero();
    q_init(0) = 1.0;
    ab_init.setZero();
    wb_init.setZero();


    // Initialize state and covariance
    s_ << p_init, v_init, q_init, ab_init, wb_init, g_init;
    P_ = params.P_init;

    s_prev_ = s_;
    P_prev_ = P_;

    // Buffer setup
    imu_buffer_ = CircularBuffer<ImuData>(10);
    pose_buffer_ = CircularBuffer<PoseData>(10);

}

void ESKF_ROS::run()
{
    ros::spin();

}

ESKF_ROS::~ESKF_ROS()
{
    delete eskf_loc_;
}

void ESKF_ROS::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double time_stamp = msg->header.stamp.toSec();
    ImuData imu_data;

    imu_data.time_stamp = time_stamp;
    imu_data.u.head<3>() = Vec3(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);

    imu_data.u.tail<3>() = Vec3(msg->angular_velocity.x,
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

    if(imu_buffer_.size() < 2)
    {
        ROS_WARN("[ESKF_ROS] Waiting for more IMU data...");
        return;
    }

    s_prev_ = s_;
    P_prev_ = P_;


    size_t head_imu = imu_buffer_.size() - 1;

    double eps = 0.005; // 10 ms

    double t_imu = imu_buffer_[head_imu].time_stamp;

    double t0 = imu_buffer_[head_imu - 1].time_stamp;
    Control u0 = imu_buffer_[head_imu - 1].u;
    double t1 = imu_buffer_[head_imu].time_stamp;
    Control u1 = imu_buffer_[head_imu].u;

    Control um = 0.5*(u0 + u1);
    eskf_loc_->propagate(t0, t1, um, s_, P_);
    s_ = eskf_loc_->getState();
    P_ = eskf_loc_->getCovariance();

    if(!pose_buffer_.empty())
    {
        int idx_latest = -1;

        for(size_t i = pose_buffer_.size()-1; i >= 0; --i)
        {
            if( pose_buffer_[i].time_stamp <= t1 + eps)
            {
                idx_latest = i;
                break;
            }
        }

        if(idx_latest >=0 )
        {
            double t_meas;
            Vec3 p_meas;
            Quat q_meas;
            
            t_meas = pose_buffer_[idx_latest].time_stamp;
            p_meas = pose_buffer_[idx_latest].p;
            q_meas = pose_buffer_[idx_latest].q;

            double buffer_start = imu_buffer_[0].time_stamp;
            if(t_meas >= buffer_start - eps)
            {
                if(t_meas >= t0 - eps && t_meas <= t1 + eps)
                {
                    Control u_m;
                    imu_interpolate(u0, t0, u1, t1, t_meas, u_m);
                    Control um0 = 0.5 * (u0 + u_m);
                    Control um1 = 0.5 * (u_m + u1);

                    // Replay from t0 to t_meas
                    eskf_loc_->propagate(t0, t_meas, um0, s_prev_, P_prev_);
                    s_ = eskf_loc_->getState();
                    P_ = eskf_loc_->getCovariance();

                    Meas z_meas;
                    z_meas.head<3>() = p_meas;
                    z_meas.tail<4>() = q_meas;
                    eskf_loc_->correct(z_meas, s_, P_);
                    s_ = eskf_loc_->getState();
                    P_ = eskf_loc_->getCovariance();

                    // Propagate from t_meas to t1
                    eskf_loc_->propagate(t_meas, t1, um1, s_, P_);
                    s_ = eskf_loc_->getState();
                    P_ = eskf_loc_->getCovariance();

                    for(size_t k = 0; k <= idx_latest; ++k)
                        pose_buffer_.pop();

                }
                else if(t_meas < t0 - eps)
                {
                    for(size_t k = 0; k <= idx_latest; ++k)
                        pose_buffer_.pop();
                }
                else
                {
                    // Do nothing, the measurement is too new
                }
            }
        }
        else
        {
            for(size_t k = 0; k < pose_buffer_.size(); ++k)
                pose_buffer_.pop();
        }
    }

    double px, py, pz;
    double vx, vy, vz;
    double qw, qx, qy, qz;
    double wx, wy, wz;

    px = s_(0);
    py = s_(1);
    pz = s_(2);

    vx = s_(3);
    vy = s_(4);
    vz = s_(5);

    qw = s_(6);
    qx = s_(7);
    qy = s_(8);
    qz = s_(9);

    // Angular velocity (from IMU measurement)
    wx = imu_buffer_[head_imu].u(3) - s_(13);
    wy = imu_buffer_[head_imu].u(4) - s_(14);
    wz = imu_buffer_[head_imu].u(5) - s_(15);

    // Publishing the odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "eskf_odom";
    
    // Position
    odom_msg.pose.pose.position.x = px;
    odom_msg.pose.pose.position.y = py;
    odom_msg.pose.pose.position.z = pz;

    // Quaternion
    odom_msg.pose.pose.orientation.w = qw;
    odom_msg.pose.pose.orientation.x = qx;
    odom_msg.pose.pose.orientation.y = qy;
    odom_msg.pose.pose.orientation.z = qz;
    
    // Linear velocity
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = vz;

    // Angular velocity
    odom_msg.twist.twist.angular.x = wx;
    odom_msg.twist.twist.angular.y = wy;
    odom_msg.twist.twist.angular.z = wz;
    
    state_pub_.publish(odom_msg);

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

}

void ESKF_ROS::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double t_pose = msg->header.stamp.toSec();
    PoseData pose_data;

    pose_data.time_stamp = t_pose;
    pose_data.p = Eigen::Vector3d(msg->pose.position.x,
                                  msg->pose.position.y,
                                  msg->pose.position.z);
    pose_data.q = Quat(msg->pose.orientation.w,
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

void ESKF_ROS::imu_interpolate(const Control &u0,
                             const double &t0,
                             const Control &u1,
                             const double &t1,
                             const double &t,
                             Control &u_interp)
{
    double alpha = (t - t0) / (t1 - t0);
    u_interp = (1 - alpha) * u0 + alpha * u1;
}