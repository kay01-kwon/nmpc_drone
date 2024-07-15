#include "estimator_test/estimator_test_tools.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;

    param_setup(nh);

    rpm << 0, 0, 0, 0;

    sigma_ext << 0, 0, 0.01;
    theta_ext << 0.001, 0, 0;

    assert(sigma_ext.size() == 3);
    assert(theta_ext.size() == 3);

    for(size_t i = 0; i < N; i++)
    {
        play_simulation_model(rpm, sigma_ext, theta_ext, simulation_time[i]);
    }

    return EXIT_SUCCESS;
}
