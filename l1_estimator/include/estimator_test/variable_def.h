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

vector<mat31_t> p_state;
vector<mat31_t> v_state;
vector<quat_t> q_state;
vector<mat31_t> w_state;

vector<mat31_t> p_ref;
vector<mat31_t> v_ref;
vector<quat_t> q_ref;
vector<mat31_t> w_ref;

SimulationModel* simulation_model_ptr;
RefModel* reference_model_ptr;
DisturbanceEstimator* disturbance_est_ptr;

mat41_t rpm;

#endif