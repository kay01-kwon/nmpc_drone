#ifndef TEST_DIST_EST_H
#define TEST_DIST_EST_H
#include <ros/ros.h>
#include "opt_based_estimator/rotational_disturbance_estimator.hpp"
#include "simulation_model/rotational_simulation_class.hpp"
// #include "utils/plot_tools.hpp"
#include <vector>

using ros::NodeHandle;
using std::vector;

void set_parameter(const NodeHandle &nh);

mat33_t J;

// pointer for rotational simulator 
// and the corresponding distrubance estimator
RotationalSimulation* rot_sim_ptr;
RotDistEst* rot_dist_est_ptr;

// Control input and disturbance for rotational simulator
vector_t M, theta_exg;

// State variables to store observation temporaliry from simulator
quaternion_t q_obs;
vector_t w_obs;

// Variables to store estimated disturbance result temporaliry
vector_t theta_est;

// Simulation final time, time difference, and the total step
double Tf;
double rate;
size_t N;

// State variables data to plot
vector<double> time_vec;
vector< vector<double> > q_obs_vec;
vector< vector<double> > w_obs_vec;

// True rotational disturbance data to plot
vector< vector<double> > theta_true_vec;

// Estimated rotational disturbance data to plot
vector< vector<double> > theta_est_vec;



void set_parameter(const NodeHandle &nh)
{
    // Initialize moment of inertia
    J.setZero();

    // Get MOI parameter from launch file
    nh.getParam("J_xx", J(0,0));
    nh.getParam("J_yy", J(1,1));
    nh.getParam("J_zz", J(2,2));

    // Get simulation time information from launch file
    nh.getParam("rate", rate);
    nh.getParam("Tf", Tf);

    assert(rate >= 100);
    assert(Tf > 1/rate);

    N = static_cast<size_t>(Tf*rate);
}


#endif