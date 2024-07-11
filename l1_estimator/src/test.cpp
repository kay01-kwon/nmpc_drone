#include <ros/ros.h>
#include "l1_estimator/l1_estimator.hpp"
#include "yaml_converter/read_config.hpp"


int main(int argc, char**argv)
{
    ros::init(argc, argv, "Test node");

    ros::NodeHandle nh;


    // Declare inertial parameters for simulation
    // and nominal model.
    inertial_param_t simulation_inertial_param;
    inertial_param_t nominal_inertial_param;

    // Declare lift and moment coefficients
    aero_coeff_t aero_coeff;

    // Declare arm length
    double l;

    string simulation_param_dir,
    nominal_param_dir;


    // Get parameter configuration directory
    // for simulation and nominal model, respectively.
    nh.getParam("simulation_param", simulation_param_dir);
    nh.getParam("nominal_param", nominal_param_dir);

    // Declare ReadConfig pointer for 
    // simulation and nominal model, respectively.
    ReadConfig* read_simulation_param_ptr;
    ReadConfig* read_nominal_param_ptr;

    // Allocate the pointers to the heap
    // to delete the memory afterwards.

    read_simulation_param_ptr = 
    new ReadConfig(simulation_param_dir);

    read_nominal_param_ptr = 
    new ReadConfig(nominal_param_dir);

    // Write the parameter by reference.
    read_simulation_param_ptr->get_param(simulation_inertial_param, 
    aero_coeff, l);

    read_nominal_param_ptr->get_param(nominal_inertial_param);

    // By deleting the pointers
    // memory is saved.
    delete read_simulation_param_ptr;
    delete read_nominal_param_ptr;








    return EXIT_SUCCESS;
}