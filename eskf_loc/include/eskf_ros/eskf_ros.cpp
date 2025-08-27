#include "eskf_ros.h"
#include <iomanip>
#include <tf/transform_broadcaster.h>

ESKF_ROS::ESKF_ROS(ros::NodeHandle &nh) : nh_(nh)
{

    EskfLocParams params;

    set_param(params);

    eskf_loc_ = new EskfLoc(params);

    imu_sub_ = nh_.subscribe("/mavros/imu/data_raw", 10, &ESKF_ROS::imu_callback, this);
    pose_sub_ = nh_.subscribe("/mocap/pose", 10, &ESKF_ROS::pose_callback, this);

    state_pub_ = nh_.advertise<nav_msgs::Odometry>("/eskf/Odom", 10);
    mocap_pub_ = nh_.advertise<nav_msgs::Odometry>("/mocap/Odom", 10);
    t_pose_curr_ = ros::Time::now().toSec();
    t_pose_prev_ = t_pose_curr_;

    z_meas_prev_.setZero();
    z_meas_prev_(3) = 1.0;

    v_mocap_lpf_.setZero();
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
    // Extract the control input from the IMU message

    imu_buffer_.push_back(*msg);

    if(imu_buffer_.size() > 100)
    {
        imu_buffer_.pop_front(); // Keep the buffer size manageable
    }
}

void ESKF_ROS::imu_interpolate(sensor_msgs::Imu &imu_out, double t_meas)
{
    // Interpolate IMU data to match the measurement time
    if (imu_buffer_.empty())
    {
        ROS_WARN("IMU buffer is empty, cannot interpolate.");
        return;
    }

    // Find the closest IMU message before and after the measurement time
    auto it_before = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), t_meas,
                                      [](const sensor_msgs::Imu &imu, double t) {
                                          return imu.header.stamp.toSec() < t;
                                      });

    if (it_before == imu_buffer_.begin())
    {
        imu_out = *it_before; // No interpolation needed, use the first message
        return;
    }

    auto it_after = it_before;
    if (it_after != imu_buffer_.end())
    {
        it_after++; // Move to the next message
    }

    if (it_after == imu_buffer_.end())
    {
        imu_out = *(--it_before); // Use the last message if no after message exists
        return;
    }

    // Perform linear interpolation between the two messages
    double t_before = it_before->header.stamp.toSec();
    double t_after = it_after->header.stamp.toSec();
    
    double alpha = (t_meas - t_before) / (t_after - t_before);

    imu_out.header.stamp = ros::Time(t_meas);
    
    imu_out.linear_acceleration.x = it_before->linear_acceleration.x * (1 - alpha) +
                                     it_after->linear_acceleration.x * alpha;
    
    imu_out.linear_acceleration.y = it_before->linear_acceleration.y * (1 - alpha) +
                                     it_after->linear_acceleration.y * alpha;

    imu_out.linear_acceleration.z = it_before->linear_acceleration.z * (1 - alpha) +
                                     it_after->linear_acceleration.z * alpha;

    imu_out.angular_velocity.x = it_before->angular_velocity.x * (1 - alpha) +
                                  it_after->angular_velocity.x * alpha;

    imu_out.angular_velocity.y = it_before->angular_velocity.y * (1 - alpha) +
                                  it_after->angular_velocity.y * alpha;

    imu_out.angular_velocity.z = it_before->angular_velocity.z * (1 - alpha) +
                                  it_after->angular_velocity.z * alpha;

}

void ESKF_ROS::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!is_meas_first_)
    {
        // Initialize the previous pose time
        t_pose_prev_ = msg->header.stamp.toSec();
        is_meas_first_ = true;
        return; // Skip the first measurement
    }

    // Extract the pose from the message
    t_pose_curr_ = msg->header.stamp.toSec();

    // Interpolate IMU data if necessary
    sensor_msgs::Imu imu_out;
    imu_interpolate(imu_out, t_pose_prev_);

    // Set the control input based on the interpolated IMU data
    control_.head<3>() = Eigen::Vector3d(imu_out.linear_acceleration.x,
                                          imu_out.linear_acceleration.y,
                                          imu_out.linear_acceleration.z);

    control_.tail<3>() = Eigen::Vector3d(imu_out.angular_velocity.x,
                                          imu_out.angular_velocity.y,
                                          imu_out.angular_velocity.z);

    // Predict the state using the ESKF
    eskf_loc_->predict(t_pose_curr_, t_pose_prev_, control_);

    // std::cout << "Predicting state at time: " << t_pose_curr_ << std::endl;
    // std::cout << "Control input: " << control_.transpose() << std::endl;

    Meas z_meas;
    z_meas.head<3>() = Eigen::Vector3d(msg->pose.position.x,
                                      msg->pose.position.y,
                                      msg->pose.position.z);
    
    z_meas.tail<4>() = Eigen::Vector4d(msg->pose.orientation.w,
                                          msg->pose.orientation.x,
                                          msg->pose.orientation.y,
                                          msg->pose.orientation.z);
    // std::cout << "Measurement at time: " << t_pose_curr_ << std::endl;
    // std::cout << "Measurement: " << z_meas.transpose() << std::endl;
    eskf_loc_->correct(z_meas);

    // Publish the state
    nav_msgs::Odometry state_msg;
    state_msg.header.stamp = ros::Time(t_pose_curr_);
    state_msg.header.frame_id = "eskf_frame";
    
    State state = eskf_loc_->getState();

    state_msg.pose.pose.position.x = state(0);
    state_msg.pose.pose.position.y = state(1);
    state_msg.pose.pose.position.z = state(2);

    state_msg.twist.twist.linear.x = state(3);
    state_msg.twist.twist.linear.y = state(4);
    state_msg.twist.twist.linear.z = state(5);
    
    state_msg.pose.pose.orientation.w = state(6);
    state_msg.pose.pose.orientation.x = state(7);
    state_msg.pose.pose.orientation.y = state(8);
    state_msg.pose.pose.orientation.z = state(9);

    Vec3 w, gyro_bias;
    gyro_bias << state(13), state(14), state(15); // Extract angular velocity from the state
    w = control_.tail<3>() - gyro_bias; // Corrected angular velocity

    state_msg.twist.twist.angular.x = w(0);
    state_msg.twist.twist.angular.y = w(1);
    state_msg.twist.twist.angular.z = w(2);
    
    state_pub_.publish(state_msg);
    nav_msgs::Odometry mocap_msg;

    mocap_msg.header = state_msg.header;
    mocap_msg.pose.pose.position.x = z_meas(0);
    mocap_msg.pose.pose.position.y = z_meas(1);
    mocap_msg.pose.pose.position.z = z_meas(2);

    mocap_msg.pose.pose.orientation.w = z_meas(3);
    mocap_msg.pose.pose.orientation.x = z_meas(4);
    mocap_msg.pose.pose.orientation.y = z_meas(5);
    mocap_msg.pose.pose.orientation.z = z_meas(6);

    double vx_mocap, vy_mocap, vz_mocap;

    Quat q_mocap_curr, q_mocap_prev;
    q_mocap_curr << z_meas(3), z_meas(4), z_meas(5), z_meas(6);
    q_mocap_prev << z_meas_prev_(3), z_meas_prev_(4), z_meas_prev_(5), z_meas_prev_(6);

    // Calculate the angular velocity from the quaternion difference
    vx_mocap = (z_meas(0) - z_meas_prev_(0)) / (t_pose_curr_ - t_pose_prev_);
    vy_mocap = (z_meas(1) - z_meas_prev_(1)) / (t_pose_curr_ - t_pose_prev_);
    vz_mocap = (z_meas(2) - z_meas_prev_(2)) / (t_pose_curr_ - t_pose_prev_);

    mocap_msg.twist.twist.linear.x = vx_mocap;
    mocap_msg.twist.twist.linear.y = vy_mocap;
    mocap_msg.twist.twist.linear.z = vz_mocap;

    mocap_pub_.publish(mocap_msg);

    static tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(state(0), state(1), state(2)));
    double qw, qx, qy, qz;
    qw = state(6);
    qx = state(7);
    qy = state(8);
    qz = state(9);
    tf::Quaternion q(qx, qy, qz, qw);
    transform.setRotation(q);
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "eskf_frame"));

    z_meas_prev_ = z_meas; // Store the current measurement for the next iteration
    

    t_pose_prev_ = t_pose_curr_;
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