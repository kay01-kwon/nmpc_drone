#include <ros/ros.h>
#include "l1_estimator/l1_estimator.hpp"
#include "yaml_converter/read_config.hpp"
#include "simulation_model/simulation_model.hpp"
#include "estimator_test/variable_def.h"


void ros_get_param(const ros::NodeHandle& nh);

int main(int argc, char**argv)
{
    
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;

    ros_get_param(nh);

    for(size_t i = 0; i < 3; i++)
    {
        bound_sigma(i) = bound_sigma(0);
        bound_theta(i) = bound_theta(0);

        Gamma_sigma(i*4) = Gamma_sigma(0);
        Gamma_theta(i*4) = Gamma_theta(0);

    }

    cout<< "Bound (trans): " << bound_sigma <<endl;
    cout<< "Epsil (trans): " << epsilon_sigma <<endl;

    cout<< "Bound (orien): " << bound_theta << endl;
    cout<< "Epsil (orien): " << epsilon_theta << endl;

    cout << "Gamma Proj (trans): " << endl;
    cout << Gamma_sigma << endl;

    cout << "Gamma Proj (orien): " << endl;
    cout << Gamma_theta << endl;


    ReadConfig read_simulation_param_obj = 
    ReadConfig(simulation_param_dir);

    ReadConfig read_nominal_param_obj = 
    ReadConfig(nominal_param_dir);

    // Write the parameter by reference.
    read_simulation_param_obj.get_param(simulation_inertial_param, 
    aero_coeff, l);

    read_nominal_param_obj.get_param(nominal_inertial_param);

    dt = 0.01;
    N = Tf/dt;

    cout<<"Final Time: "<< Tf <<endl;

    cout<<"Discrete time: "<< dt <<endl;

    cout<<"Simulation step: "<< N <<endl;

    // Capacity allocation
    simulation_time.reserve(N);


    cout << simulation_time.capacity() << endl;

    assert(simulation_time.capacity() == N);

    // Simulation model object to test estimation performance
    SimulationModel sim_model_obj 
    = SimulationModel(quad_model, 
    aero_coeff, 
    simulation_inertial_param, 
    l);

    RefModel ref_model_obj= 
    RefModel(nominal_inertial_param,
    kp, kv, kq, kw);

    DisturbanceEstimator disturbance_estimator_obj
    = DisturbanceEstimator(nominal_inertial_param,
        bound_sigma, epsilon_sigma, 
        bound_theta, epsilon_theta, 
        Gamma_sigma, Gamma_theta,
        tau_sigma, tau_theta);


    return EXIT_SUCCESS;
}

void ros_get_param(const ros::NodeHandle& nh)
{
    // Get parameter configuration directory
    // for simulation and nominal model, respectively.
    nh.getParam("simulation_param_dir", simulation_param_dir);
    nh.getParam("nominal_param_dir", nominal_param_dir);

    int quad_model_;

    nh.getParam("quad_model",quad_model_);
    if (quad_model_ == 1)
        quad_model = QuadModel::model1;
    else
        quad_model = QuadModel::model2;

    nh.getParam("simulation_final_time", Tf);
    nh.getParam("discrete_time", dt);

    nh.getParam("kp", kp);
    nh.getParam("kv", kv);
    nh.getParam("kq", kq);
    nh.getParam("kw", kw);

    nh.getParam("b_sigma", bound_sigma(0));
    nh.getParam("e_sigma", epsilon_sigma);

    nh.getParam("b_theta", bound_theta(0));
    nh.getParam("e_theta", epsilon_theta);

    nh.getParam("Gamma_sigma", Gamma_sigma(0));
    nh.getParam("Gamma_theta", Gamma_theta(0));

    nh.getParam("tau_sigma", tau_sigma);
    nh.getParam("tau_theta", tau_theta);
}