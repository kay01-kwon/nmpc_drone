#ifndef ROS_LPF_NODE_HPP_
#define ROS_LPF_NODE_HPP_
#include <ros/ros.h>
#include "l1_estimator/low_pass_filter.hpp"
#include "l1_estimator/Lpf_test.h"


using l1_estimator::Lpf_test;
using l1_estimator::Lpf_testConstPtr;
using ros::NodeHandle;

class RosLpf{

    public:

        RosLpf() = delete;

        RosLpf(const NodeHandle& nh,
        const double& tau);

        void ros_setup();

        void callback(const Lpf_testConstPtr& signal_msg);

        void filtered_signal_publish();

    private:

        NodeHandle nh_;
        ros::Publisher lpf_publisher_;
        ros::Subscriber signal_subscriber_;

        mat31_t signal_filtered_;

        Lpf lpf_obj;

        double time_curr, time_offset;

        bool init_time;
};



#endif