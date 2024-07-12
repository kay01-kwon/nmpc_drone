#include <ros/ros.h>
#include <numeric>
#include "l1_estimator/l1_estimator.hpp"
#include "yaml_converter/read_config.hpp"
#include "simulation_model/simulation_model.hpp"
#include "estimator_test/variable_def.h"

void param_estup(const ros::NodeHandle& nh);

void print_parameter_setup(const mat31_t& bound_theta, const double& epsilon_theta,
const mat31_t& bound_sigma, const double& epsilon_sigma,
const mat33_t& Gamma_sigma, const mat33_t& Gamma_theta,
const double& tau_sigma, const double& tau_theta);

int main(int argc, char**argv)
{
    
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;

    param_setup(nh);

    rpm << 0, 0, 0, 0;

    theta_ext << 0.001, 0, 0;
    sigma_ext << 0, 0, 0.01;

    for(size_t i = 0; i < N; i++)
    {
        simulation_model_ptr->set_control_input(rpm);
        simulation_model_ptr->set_disturbance(theta_ext, sigma_ext);
        simulation_model_ptr->set_time(simulation_time[i]);

    }

    return EXIT_SUCCESS;
}

void param_setup(const ros::NodeHandle& nh)
{

    // String to store parameter directory
    string simulation_param_dir,
    nominal_param_dir;

    // quad model: '+' or 'x'
    QuadModel quad_model;

    // Reference model control gain
    double kp, kv, kq, kw;

    // Parameter for convex function
    mat31_t bound_sigma, bound_theta;
    double epsilon_sigma, epsilon_theta;

    // parameter for projection operator
    mat33_t Gamma_sigma, Gamma_theta;

    // parameter for low pass filter
    double tau_sigma, tau_theta;

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

    for(size_t i = 0; i < 3; i++)
    {
        bound_sigma(i) = bound_sigma(0);
        bound_theta(i) = bound_theta(0);

        Gamma_sigma(i*4) = Gamma_sigma(0);
        Gamma_theta(i*4) = Gamma_theta(0);
    }

    // Declare inertial parameters for simulation
    // and nominal model.
    inertial_param_t simulation_inertial_param;
    inertial_param_t nominal_inertial_param;

    // Declare lift and moment coefficients
    aero_coeff_t aero_coeff;

    // Declare arm length
    double l;

    ReadConfig read_simulation_param_obj = 
    ReadConfig(simulation_param_dir);

    ReadConfig read_nominal_param_obj = 
    ReadConfig(nominal_param_dir);

    // Write the parameter by reference.
    read_simulation_param_obj.get_param(simulation_inertial_param, 
    aero_coeff, l);

    read_nominal_param_obj.get_param(nominal_inertial_param);

    N = Tf/dt;

    // Capacity allocation
    simulation_time.reserve(N);

    for(size_t i = 0; i < N; i++)
    {
        simulation_time.push_back(i*dt);
    }

    assert(dt > std::numeric_limits<double>::min());
    assert(simulation_time.capacity() == N);

    print_parameter_setup(bound_theta, epsilon_theta,
    bound_theta, epsilon_theta,
    Gamma_sigma, Gamma_theta,
    tau_sigma, tau_theta);

    // Simulation model object to test estimation performance
    SimulationModel simulation_model_obj 
    = SimulationModel(quad_model, 
    aero_coeff, 
    simulation_inertial_param, 
    l);

    RefModel reference_model_obj= 
    RefModel(nominal_inertial_param,
    kp, kv, kq, kw);

    DisturbanceEstimator disturbance_est_obj
    = DisturbanceEstimator(nominal_inertial_param,
        bound_sigma, epsilon_sigma, 
        bound_theta, epsilon_theta, 
        Gamma_sigma, Gamma_theta,
        tau_sigma, tau_theta);

    simulation_model_ptr = & simulation_model_obj;
    reference_model_ptr = &reference_model_obj;
    disturbance_est_ptr = &disturbance_est_obj;

}

void print_parameter_setup(const mat31_t &bound_theta, const double &epsilon_theta, 
const mat31_t &bound_sigma, const double &epsilon_sigma, 
const mat33_t &Gamma_sigma, const mat33_t &Gamma_theta, 
const double &tau_sigma, const double &tau_theta)
{
    cout << "***************************************" << endl;
    cout << "Convex function setup" << endl;
    cout << "Bound (trans): " << bound_sigma << endl;
    cout << "Epsil (trans): " << epsilon_sigma << endl;
    cout << endl;
    cout << "Bound (orien): " << bound_theta << endl;
    cout << "Epsil (orien): " << epsilon_theta << endl;

    cout << "***************************************" << endl;

    cout << "Gamma projection setup" << endl;
    cout << "Gamma Proj (trans): " << endl;
    cout << Gamma_sigma << endl;
    cout << endl;
    cout << "Gamma Proj (orien): " << endl;
    cout << Gamma_theta << endl;

    cout << "***************************************" << endl;

    cout << "Low pass filter setup" << endl;
    cout << "tau (trans): "<< tau_sigma << endl;
    cout << "tau (orien): "<< tau_theta << endl;

    cout << "***************************************" << endl;

    cout << "Simulation time setup" << endl;
    cout<<"Final Time: "<< Tf <<endl;
    cout<<"Discrete time: "<< dt <<endl;
    cout<<"Simulation step: "<< N <<endl;
}
