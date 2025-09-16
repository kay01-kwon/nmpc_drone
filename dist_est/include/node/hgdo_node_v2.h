#ifndef HGDO_NODE_V2_H
#define HGDO_NODE_V2_H

#include <ros/ros.h>
#include "dist_est/hgdo_dist_est.h"
#include "utils/forward_dynamics.h"
#include "utils/circular_buffer.h"

#include "ros_libcanard/hexa_actual_rpm.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>

#include <thread>
#include <mutex>
#include <condition_variable>

using nav_msgs::Odometry;
using geometry_msgs::Wrench;

using std::thread;
using std::mutex;
using std::condition_variable;

class HgdoNode2{

    public:

    explicit HgdoNode2(ros::NodeHandle &nh);

    ~HgdoNode2();

    void run();

    private:

    ros::NodeHandle nh_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber odom_sub_;

    ros::Timer publish_timer_;
    ros::Publisher wrench_pub_;

    double t_curr_{0.0};
    double t_prev_{-0.01};
    double period_{0.01};

    double time_latest_[2];
    double last_rx_wall_[2];
    double time_out_[2];

    Wrench wrench_msg_;

    HGDO* hgdo_dist_est_;
    FDynamics* converter_;

    Vec6d f_tau_ext_;

    CircularBuffer<StateData> state_buffer_;
    CircularBuffer<RpmData> rpm_buffer_;

    thread hgdo_est_thread_;
    mutex mBuf_;
    condition_variable cv_;

    void rpmCallback(const ros_libcanard::hexa_actual_rpm &rpm_msg);
    void stateCallback(const Odometry &state_msg);
    void publishCallback(const ros::TimerEvent&);

    void processState();

    double watermark_time();

    bool freshByTTL(int i, double now_wall);

    void setParam(const std::string param_name, MavParam &mav_param);
    void setParam(const std::string param_name, HgdoParam &hgdo_param);

};

#endif