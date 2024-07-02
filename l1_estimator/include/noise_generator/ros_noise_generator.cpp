#include "ros_noise_generator.hpp"

RosNoiseGenerator::RosNoiseGenerator(const NodeHandle &nh, const mat31_t &mag, const mat31_t &freq)
:nh_(nh), mag_(mag), freq_(freq)
{
    noise_publisher = nh_.advertise<Lpf_test>("/input_lpf",1);
}

void RosNoiseGenerator::noise_publish()
{
    
}
