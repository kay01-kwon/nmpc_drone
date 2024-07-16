#ifndef ESTIMATOR_TEST_TOOLS_H_
#define ESTIMATOR_TEST_TOOLS_H_

#include <ros/ros.h>
#include <numeric>
#include "l1_estimator/l1_estimator.hpp"
#include "yaml_converter/read_config.hpp"
#include "simulation_model/simulation_model.hpp"
#include "variable_def.h"

#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;

void param_setup(const ros::NodeHandle& nh);

void print_parameter_setup(const inertial_param_t& inertial_param, 
    const aero_coeff_t& aero_param,
    const mat31_t& bound_theta, const double& epsilon_theta,
    const mat31_t& bound_sigma, const double& epsilon_sigma,
    const mat33_t& Gamma_sigma, const mat33_t& Gamma_theta,
    const double& tau_sigma, const double& tau_theta);

void play_simulation_model(const mat41_t& rpm_, 
const mat31_t& sigma_ext_, const mat31_t& theta_ext_,
const double& simulation_time_);

void variable_capacity_reserve(const int& N);

void demux_simulation_state(const mat31_t& position,
const mat31_t& linear_velocity,
const quat_t& quaternion,
const mat31_t& angular_velocity);

void demux_reference_state(const mat31_t& position,
const mat31_t& linear_velocity,
const quat_t& quaternion,
const mat31_t& angular_velocity);

void demux_vec3(const mat31_t& v, vector<double>&x, vector<double>&y, vector<double>&z);

void demux_quat(const quat_t& q, vector<double>&qw, vector<double>&qx, vector<double>&qy, vector<double>&qz);



/**
 * @brief From ros node handle get parameters
 * such as inertial param, reference model param, 
 * disturbance param and so on
 * 
 * @param nh Node handle
 */
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

    bound_sigma.setZero();
    bound_theta.setZero();
    Gamma_sigma.setZero();
    Gamma_theta.setZero();

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

    // Compute simulation time step
    N = Tf/dt + 1;

    // Allocate vector by the simulation type step
    variable_capacity_reserve(N);

    // Put simulation time step
    for(size_t i = 0; i < N; i++)
    {
        simulation_time.push_back(i*dt);
    }

    assert(dt > std::numeric_limits<double>::min());
    assert(simulation_time.capacity() == N);

    print_parameter_setup(simulation_inertial_param,
        aero_coeff,
        bound_theta, epsilon_theta,
        bound_theta, epsilon_theta,
        Gamma_sigma, Gamma_theta,
        tau_sigma, tau_theta);

    // Simulation model object to test estimation performance
    simulation_model_ptr 
    = new SimulationModel(quad_model, 
    aero_coeff, 
    simulation_inertial_param, 
    l);

    reference_model_ptr = 
    new RefModel(nominal_inertial_param,
    kp, kv, kq, kw);

    disturbance_est_ptr
    = new DisturbanceEstimator(nominal_inertial_param,
        bound_sigma, epsilon_sigma, 
        bound_theta, epsilon_theta, 
        Gamma_sigma, Gamma_theta,
        tau_sigma, tau_theta);

}

/**
 * @brief print parameter setup
 * 
 * @param bound_theta 
 * @param epsilon_theta 
 * @param bound_sigma 
 * @param epsilon_sigma 
 * @param Gamma_sigma 
 * @param Gamma_theta 
 * @param tau_sigma 
 * @param tau_theta 
 */
void print_parameter_setup(const inertial_param_t& inertial_param, 
    const aero_coeff_t& aero_param,
    const mat31_t &bound_theta, const double &epsilon_theta, 
    const mat31_t &bound_sigma, const double &epsilon_sigma, 
    const mat33_t &Gamma_sigma, const mat33_t &Gamma_theta, 
    const double &tau_sigma, const double &tau_theta)
{
    cout << "***************************************" << endl;
    cout << "Inertial parameter setup" << endl;
    cout << "mass: " << inertial_param.m << endl;
    cout << "MOI: " << endl;
    cout << inertial_param.J <<endl;
    cout << endl;
    cout << "COM offset: ";
    for(size_t i = 0; i < inertial_param.r_offset.size(); i++)
        cout << inertial_param.r_offset(i) << " ";
    cout<<endl;


    cout << "***************************************" << endl;
    cout << "Aero parameter setup" << endl;
    cout << "lift coeff: " << aero_param.lift_coeff << endl;
    cout << "moment coeff: " << aero_param.moment_coeff << endl;

    cout << "***************************************" << endl;
    cout << "Convex function setup" << endl;
    cout << "Bound (trans): " << endl;
    for(size_t i = 0; i < bound_sigma.size(); i++)
        cout << bound_sigma(i) << " ";
    cout<<endl;
    cout << "Epsil (trans): " << epsilon_sigma << endl;
    cout << endl;
    cout << "Bound (orien): " << endl;
    for(size_t i = 0; i < bound_theta.size(); i++)
        cout << bound_theta(i) << " ";
    cout << endl;
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

    cout << "***************************************" << endl;
}


/**
 * @brief play simulation model
 * 
 * @param rpm_ rotor rpm
 * @param sigma_ext_ disturbance (trans)
 * @param theta_ext_ disturbance (orien)
 * @param simulation_time_ simulation time
 */
void play_simulation_model(const mat41_t &rpm_, 
const mat31_t &sigma_ext_, const mat31_t &theta_ext_, 
const double &simulation_time_)
{

    assert(rpm_.size() == 4);
    assert(sigma_ext_.size() == 3);
    assert(theta_ext_.size() == 3);
    assert(typeid(simulation_time_) == typeid(double));

    simulation_model_ptr->set_control_input(rpm_);
    simulation_model_ptr->set_disturbance(sigma_ext_, theta_ext_);
    simulation_model_ptr->set_time(simulation_time_);
    simulation_model_ptr->integrate();
    
}


/**
 * @brief Allocate variables by the size of N
 * 
 * @param N_ 
 */
inline void variable_capacity_reserve(const int &N_)
{
    // simulation time
    simulation_time.reserve(N_);

    // Simulation state
    // ************************************************
    // State position variables
    x_state.reserve(N_);
    y_state.reserve(N_);
    z_state.reserve(N_);

    // State velocity variables
    vx_state.reserve(N_);
    vy_state.reserve(N_);
    vz_state.reserve(N_);

    // State quaternion variables
    qw_state.reserve(N_);
    qx_state.reserve(N_);
    qy_state.reserve(N_);
    qz_state.reserve(N_);

    // State angular velocity variables
    wx_state.reserve(N_);
    wy_state.reserve(N_);
    wz_state.reserve(N_);

    // ************************************************
    // Reference model state
    // Reference position variables
    x_ref.reserve(N_);
    y_ref.reserve(N_);
    z_ref.reserve(N_);

    // Reference velocity variables
    vx_ref.reserve(N_);
    vy_ref.reserve(N_);
    vz_ref.reserve(N_);

    // REference quaternion variables
    qw_ref.reserve(N_);
    qx_ref.reserve(N_);
    qy_ref.reserve(N_);
    qz_ref.reserve(N_);

    // Reference angular velocity variables
    wx_ref.reserve(N_);
    wy_ref.reserve(N_);
    wz_ref.reserve(N_);    

    // ************************************************
    // Disturbance terms

    sigma_ext_x.reserve(N_);
    sigma_ext_y.reserve(N_);
    sigma_ext_z.reserve(N_);

    sigma_est_x.reserve(N_);
    sigma_est_y.reserve(N_);
    sigma_est_z.reserve(N_);

    theta_est_x.reserve(N_);
    theta_est_y.reserve(N_);
    theta_est_z.reserve(N_);

    sigma_est_lpf_x.reserve(N_);
    sigma_est_lpf_y.reserve(N_);
    sigma_est_lpf_z.reserve(N_);

    theta_est_lpf_x.reserve(N_);
    theta_est_lpf_y.reserve(N_);
    theta_est_lpf_z.reserve(N_);


}

/**
 * @brief Put simulation state in the order of position, 
 * linear velocity, quaternion, and angular velocity
 * 
 * @param position 
 * @param linear_velocity 
 * @param quaternion 
 * @param angular_velocity 
 */
inline void demux_simulation_state(const mat31_t &position, 
const mat31_t &linear_velocity, 
const quat_t &quaternion, 
const mat31_t &angular_velocity)
{
    demux_vec3(position, x_state, y_state, z_state);
    demux_vec3(linear_velocity, vx_state, vy_state, vz_state);
    demux_quat(quaternion, qw_state, qx_state, qy_state, qz_state);
    demux_vec3(angular_velocity, wx_state, wy_state, wz_state);
}

/**
 * @brief Put reference state in the order of position, 
 * linear velocity, quaternion, and angular velocity
 * 
 * @param position 
 * @param linear_velocity 
 * @param quaternion 
 * @param angular_velocity 
 */
inline void demux_reference_state(const mat31_t &position, 
const mat31_t &linear_velocity, 
const quat_t &quaternion, 
const mat31_t &angular_velocity)
{
    demux_vec3(position, x_ref, y_ref, z_ref);
    demux_vec3(linear_velocity, vx_ref, vy_ref, vz_ref);
    demux_quat(quaternion, qw_ref, qx_ref, qy_ref, qz_ref);
    demux_vec3(angular_velocity, wx_ref, wy_ref, wz_ref);
}

/**
 * @brief Demux 3 dimensional vector into comp_x, comp_y, comp_z
 * 
 * @param v 3 dim vector to demux
 * @param comp_x 
 * @param comp_y 
 * @param comp_z 
 */
inline void demux_vec3(const mat31_t &v, 
vector<double> &comp_x, vector<double> &comp_y, vector<double> &comp_z)
{
    comp_x.push_back(v(0));
    comp_y.push_back(v(1));
    comp_z.push_back(v(2));
}

/**
 * @brief Demux quaternion into w, x, y, and z
 * 
 * @param q quaternion to demux
 * @param qw 
 * @param qx 
 * @param qy 
 * @param qz 
 */
inline void demux_quat(const quat_t & q, 
vector<double>& qw, vector<double>& qx, vector<double>& qy, vector<double>& qz)
{
    qw.push_back(q.w());
    qx.push_back(q.x());
    qy.push_back(q.y());
    qz.push_back(q.z());
}

#endif