#ifndef TEST_DIST_EST_H
#define TEST_DIST_EST_H
#include <ros/ros.h>
#include "opt_based_estimator/rotational_disturbance_estimator.hpp"
#include "simulation_model/rotational_simulation_class.hpp"
// #include "utils/plot_tools.hpp"
#include <vector>

using std::vector;

mat33_t J;

// pointer for rotational distrubance estimator
RotDistEst* rot_dist_est_ptr;

// pointer for rotational simulator
RotationalSimulation* rot_sim_ptr;

// Control input and disturbance for rotational simulator
vector_t M, theta_exg;


quaternion_t q_obs;
vector_t w_obs;
vector_t theta_est;

vector<double> time_vec;
vector< vector<double> > q_obs_vec;
vector< vector<double> > w_obs_vec;

vector< vector<double> > theta_est_vec;

#endif