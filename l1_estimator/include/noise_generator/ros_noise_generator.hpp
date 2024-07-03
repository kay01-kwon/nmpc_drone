#pragma once
#include <ros/ros.h>
#include "l1_estimator/type_definitions.hpp"
#include "noise_generator.hpp"
#include "l1_estimator/Lpf_test.h"

using l1_estimator::Lpf_test;
using ros::NodeHandle;

class RosNoiseGenerator
{
    public:

        RosNoiseGenerator() = delete;

        RosNoiseGenerator(const NodeHandle& nh,
        const mat31_t& mag, const mat31_t& freq,
        const double& stddev);

        void noise_publish();


    private:

        NodeHandle nh_;
        ros::Publisher noise_publisher;

        mat31_t mag_;
        mat31_t freq_;
        double stddev_;

};
