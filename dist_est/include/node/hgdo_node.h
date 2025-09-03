#ifndef HGDO_NODE_H
#define HGDO_NODE_H

#include <ros/ros.h>
#include "dist_est/hgdo_dist_est.h"
#include "utils/forward_dynamics.h"
#include "utils/circular_buffer.h"

#include "ros_libcanard/hexa_actual_rpm.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

using nav_msgs::Odometry;
using geometry_msgs::Wrench;

using std::thread;
using std::mutex;
using std::condition_variable;

class HgdoNode{

    public:

    HgdoNode();

    HgdoNode(ros::NodeHandle &nh);

    ~HgdoNode();

    void run();

    private:

    ros::NodeHandle nh_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber odom_sub_;

    ros::Timer publish_timer_;
    ros::Publisher wrench_pub_;

    Odometry state_msg_;

    Wrench wrench_msg_;

    double t_input_{0.0};




};

#endif