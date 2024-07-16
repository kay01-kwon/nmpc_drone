#pragma once
#ifndef VARIABLE_H
#define VARIABLE_H

#include <ros/ros.h>
#include "l1_estimator/l1_estimator.hpp"
#include "yaml_converter/read_config.hpp"
#include "simulation_model/simulation_model.hpp"

using std::vector;

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

mat31_t p_state;
mat31_t v_state;
quat_t q_state;
mat31_t w_state;

mat31_t p_ref;
mat31_t v_ref;
quat_t q_ref;
mat31_t w_ref;

vector<double> x_state, y_state, z_state; 
vector<double> vx_state, vy_state, vz_state;
vector<double> qw_state, qx_state, qy_state, qz_state;
vector<double> wx_state, wy_state, wz_state;

vector<double> x_ref, y_ref, z_ref;
vector<double> vx_ref, vy_ref, vz_ref;
vector<double> qw_ref, qx_ref, qy_ref, qz_ref;
vector<double> wx_ref, wy_ref, wz_ref;

vector<double> sigma_est_x, sigma_est_y, sigma_est_z;
vector<double> sigma_est_lpf_x, sigma_est_lpf_y, sigma_est_lpf_z;
vector<double> sigma_ext_x, sigma_ext_y, sigma_ext_z;

vector<double> theta_est_x, theta_est_y, theta_est_z;
vector<double> theta_est_lpf_x, theta_est_lpf_y, theta_est_lpf_z;
vector<double> theta_ext_x, theta_ext_y, theta_ext_z;


SimulationModel* simulation_model_ptr;
RefModel* reference_model_ptr;
DisturbanceEstimator* disturbance_est_ptr;

mat41_t rpm;

#endif