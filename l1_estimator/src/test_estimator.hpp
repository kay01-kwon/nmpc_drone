#ifndef TEST_HPP_
#define TEST_HPP_
#include <ros/ros.h>
#include "l1_estimator/l1_estimator.hpp"
#include "yaml_converter/read_config.hpp"
#include "simulation_model/simulation_model.hpp"

using std::vector;

// Declare inertial parameters for simulation
// and nominal model.
inertial_param_t simulation_inertial_param;
inertial_param_t nominal_inertial_param;

// Declare lift and moment coefficients
aero_coeff_t aero_coeff;

// Declare arm length
double l;

// Declare ReadConfig pointer for 
// simulation and nominal model, respectively.
ReadConfig* read_simulation_param_ptr;
ReadConfig* read_nominal_param_ptr;

// String to store parameter directory
string simulation_param_dir,
nominal_param_dir;

// quad model: '+' or 'x'
QuadModel quad_model;

double kp, kv, kq, kw;

// External force and moment
mat31_t theta_ext, sigma_ext;

// Noisy estimated external force and moment
mat31_t theta_est_noisy, sigma_est_noisy;

// Filtered estimated external force and moment
mat31_t theta_est_lpf, sigma_est_lpf;

// Discrete time
double dt;

// Final time for simulation
double Tf;

// Step
int N;

vector<double> simulation_time;

#endif