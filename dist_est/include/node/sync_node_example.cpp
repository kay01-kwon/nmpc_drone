#include "sync_node_example.h"

SyncNodeExample::SyncNodeExample(ros::NodeHandle &nh)
:nh_(nh)
{
    ros::TransportHints transport_hint;
    transport_hint = ros::TransportHints()
                    .tcpNoDelay(true);
    
    // Subscribers
    state_sub_ = nh_.subscribe<Odometry>("/custom_hexacopter/ground_truth/odometry", 10,
    &SyncNodeExample::stateCallback, this, transport_hint);
    rpm_sub_ = nh_.subscribe<hexa_actual_rpm>("/uav/actual_rpm", 10,
    &SyncNodeExample::rpmCallback, this, transport_hint);
    // wrench_sub_ = nh_.subscribe<Wrench>("/wrench/in", 10,
    // &SyncNodeExample::wrenchCallback, this, transport_hint);
    
    // Publish timer and publisher
    publish_timer_ = nh_.createTimer(ros::Duration(0.01),
    &SyncNodeExample::publishCallback, this);
    time_sync_pub_ = nh_.advertise<std_msgs::Float64>("/time_sync", 10);

    sync_thread_ = thread(&SyncNodeExample::aggregate_thread, this);
    // process_thread_ = thread(&SyncNodeExample::process_thread, this);

    time_latest_[0] = -0.01;
    time_latest_[1] = -0.01;

    last_rx_wall_[0] = 0;
    last_rx_wall_[1] = 0;

    fresh_data_[0] = false;
    fresh_data_[1] = false;

    time_out_[0] = 0.015; // state
    time_out_[1] = 0.015;  // rpm

}

SyncNodeExample::~SyncNodeExample()
{
    ROS_INFO("Shutting down sync node example...");
    if(sync_thread_.joinable())
        sync_thread_.join();
    // if(process_thread_.joinable())
    //     process_thread_.join();
}

void SyncNodeExample::run()
{
    ros::spin();
}

void SyncNodeExample::stateCallback(const Odometry::ConstPtr& msg)
{
    std::lock_guard<mutex> lock(mBuf_);
    StateData state_data;
    state_data.time_stamp = msg->header.stamp.toSec();
    
    state_data.p << msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z;
    
    state_data.v << msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y,
                    msg->twist.twist.linear.z;
    
    state_data.w << msg->twist.twist.angular.x,
                    msg->twist.twist.angular.y,
                    msg->twist.twist.angular.z;

    state_data.q.w() = msg->pose.pose.orientation.w;
    state_data.q.x() = msg->pose.pose.orientation.x;
    state_data.q.y() = msg->pose.pose.orientation.y;
    state_data.q.z() = msg->pose.pose.orientation.z;

    state_buffer_.push_back(state_data);

    time_latest_[0] = std::max(time_latest_[0], state_data.time_stamp);
    last_rx_wall_[0] = ros::WallTime::now().toSec();
    cv_.notify_one();

}

void SyncNodeExample::rpmCallback(const hexa_actual_rpm::ConstPtr &rpm_msg)
{
    std::lock_guard<mutex> lock(mBuf_);
    RpmData rpm_data;
    rpm_data.time_stamp = rpm_msg->stamp.toSec();
    rpm_data.rpm << rpm_msg->rpm[0], rpm_msg->rpm[1], rpm_msg->rpm[2],
                    rpm_msg->rpm[3], rpm_msg->rpm[4], rpm_msg->rpm[5];


    rpm_buffer_.push_back(rpm_data);

    time_latest_[1] = std::max(time_latest_[1], rpm_data.time_stamp);
    last_rx_wall_[1] = ros::WallTime::now().toSec();
    cv_.notify_one();
}

void SyncNodeExample::aggregate_thread()
{
    double t_loop = 0.0;
    ROS_INFO("Start aggregate thread");
    while(ros::ok())
    {
        // ROS_INFO("Waiting for data...");
        std::unique_lock<mutex> lk(mBuf_);
        cv_.wait_for(lk,
        std::chrono::duration<double>(2*period_), 
        [this]
        {
            if(!ros::ok())
                ROS_INFO("Shutting down aggregate thread...");
            return ( (watermark_event_time() - t_prev_ >= period_) || !ros::ok() );
        });

        t_curr_ = watermark_event_time();

        t_loop = t_curr_ - t_prev_;


        // if(t_loop <= 0.0)
        // {
        //     ROS_INFO("t_curr: %.4f, t_prev: %.4f, t_loop: %.4f", t_curr_, t_prev_, t_loop);
        // }

        ROS_INFO("t_curr: %.4f, t_prev: %.4f, t_loop: %.4f", t_curr_, t_prev_, t_loop);        

        t_prev_ = t_curr_;
        // std::string s;
        // ROS_INFO("state: %s", fresh_data_[0] ? "fresh" : "stale",
        //          ", rpm: ", fresh_data_[1] ? "fresh" : "stale",
        //          ", wrench: ", fresh_data_[2] ? "fresh" : "stale");

        lk.unlock();
    }

}

void SyncNodeExample::process_thread()
{
}

bool SyncNodeExample::freshByTTL(int i, double now_wall)
{
    return (now_wall - last_rx_wall_[i] < time_out_[i]);
}

double SyncNodeExample::watermark_event_time()
{
    double watermark;

    watermark = time_latest_[0];

    for(int i = 0; i < 2; ++i)
    {
        if(freshByTTL(i, ros::WallTime::now().toSec()))
            watermark = std::min(watermark, time_latest_[i]);
    }

    return watermark;
}

void SyncNodeExample::publishCallback(const ros::TimerEvent& event)
{

}