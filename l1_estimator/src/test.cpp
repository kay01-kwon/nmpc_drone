#include <ros/ros.h>
#include "l1_estimator/l1_estimator.hpp"
#include "yaml_converter/read_config.hpp"


int main(int argc, char**argv)
{
    ros::init(argc, argv, "Test node");

    ros::NodeHandle nh;

    string simulation_param_dir,
    nominal_param_dir;

    nh.getParam("simulation_param", simulation_param_dir);
    nh.getParam("nominal_param", nominal_param_dir);

    ReadConfig* sim_param_read_ptr;
    ReadConfig* nominal_param_read_ptr;

    sim_param_read_ptr = 
    new ReadConfig(simulation_param_dir);
    
    nominal_param_read_ptr = 
    new ReadConfig(nominal_param_dir);



    return EXIT_SUCCESS;
}