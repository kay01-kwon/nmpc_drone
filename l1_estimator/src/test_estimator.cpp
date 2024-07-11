#include "test_estimator.hpp"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "Test node");

    int quad_model_;

    // Get parameter configuration directory
    // for simulation and nominal model, respectively.
    nh.getParam("simulation_param", simulation_param_dir);
    nh.getParam("nominal_param", nominal_param_dir);

    nh.getParam("quad_model",quad_model_);
    if (quad_model_ == 1)
        quad_model = QuadModel::model1;
    else
        quad_model = QuadModel::model2;

    nh.getParam("kp", kp);
    nh.getParam("kv", kv);
    nh.getParam("kq", kq);
    nh.getParam("kw", kw);

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

    // Simulation model object to test estimation performance
    SimulationModel sim_model_obj(quad_model, 
    aero_coeff, simulation_inertial_param, l);

    // Reference model object
    RefModel ref_model_obj(nominal_inertial_param,
    kp, kv, kq, kw);


    Tf = 10;
    dt = 0.01;
    N = Tf/dt;

    cout<<"Simulation step: "<<endl;
    cout<<N<<endl;

    return EXIT_SUCCESS;
}