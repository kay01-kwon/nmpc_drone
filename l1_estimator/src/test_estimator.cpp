#include "estimator_test/estimator_test_tools.h"
// #include <ros/ros.h>
// #include <numeric>
// #include "l1_estimator/l1_estimator.hpp"
// #include "yaml_converter/read_config.hpp"
// #include "simulation_model/simulation_model.hpp"
// #include "estimator_test/variable_def.h"

// void param_setup(const ros::NodeHandle& nh);

// void print_parameter_setup(const mat31_t& bound_theta, const double& epsilon_theta,
// const mat31_t& bound_sigma, const double& epsilon_sigma,
// const mat33_t& Gamma_sigma, const mat33_t& Gamma_theta,
// const double& tau_sigma, const double& tau_theta);

// void play_simulation_model(const mat41_t& rpm_, 
// const mat31_t& sigma_ext_, const mat31_t& theta_ext_,
// const double& simulation_time_);

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
