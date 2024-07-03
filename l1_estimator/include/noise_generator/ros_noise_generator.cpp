#include "ros_noise_generator.hpp"

RosNoiseGenerator::RosNoiseGenerator(const NodeHandle &nh, 
const mat31_t &mag, 
const mat31_t &freq,
const double& stddev)
:nh_(nh), mag_(mag), freq_(freq), stddev_(stddev)
{
    noise_publisher = nh_.advertise<Lpf_test>("/input_signal",1);
}

void RosNoiseGenerator::noise_publish()
{

    Lpf_test lpf_input_msg;
    mat31_t v_noise;
    double t;
    long long seedNum = get_seedNum();

    lpf_input_msg.stamp = ros::Time::now();
    t = ros::Time::now().toSec()
    + ros::Time::now().toNSec()*10e-9;

    for(size_t i = 0; i < 3; i++)
    {
        v_noise(i) = mag_(i)*sin(2*M_PI*freq_(i)*t)
        + noise(stddev_, seedNum + i*2);

        lpf_input_msg.v[i] = v_noise(i);
    }

    noise_publisher.publish(lpf_input_msg);

}
