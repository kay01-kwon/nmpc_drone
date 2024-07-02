#include "noise_generator/ros_noise_generator.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "noise_generator");

    ros::NodeHandle nh;

    mat31_t mag, freq;
    double stddev;

    mag     << 5, 5, 5;
    freq    << 10, 10, 10;
    stddev = 0.01;

    RosNoiseGenerator ros_noise_generator(
        nh, 
        mag, 
        freq, 
        stddev);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros_noise_generator.noise_publish();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}