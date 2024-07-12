#include "estimator_test/load_param.h"

int main(int argc, char**argv)
{
    
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;
    
    load_param(nh);

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