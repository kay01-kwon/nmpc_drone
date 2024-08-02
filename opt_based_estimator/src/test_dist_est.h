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

void reserve_vec_data(const int &N_);

mat33_t J;

// pointer for rotational simulator 
// and the corresponding distrubance estimator
RotationalSimulation* rot_sim_ptr;
RotDistEst* rot_dist_est_ptr;

// Weight for the disturbance estimator 
// (qw, qx, qy, qz, wx, wy, wz)
mat77_t Q;

double term_error;
int iter_max;

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
    
    // Get simulation step
    N = static_cast<size_t>(Tf*rate);

    // Initialize weight matrix for optimization based disturnbace estimator
    Q.setZero();
    nh.getParam("Q_qw", Q(0,0));
    nh.getParam("Q_qx", Q(1,1));
    nh.getParam("Q_qy", Q(2,2));
    nh.getParam("Q_qz", Q(3,3));
    nh.getParam("Q_wx", Q(4,4));
    nh.getParam("Q_wy", Q(5,5));
    nh.getParam("Q_wz", Q(6,6));

    nh.getParam("term_error", term_error);
    nh.getParam("iter_max", iter_max);

    assert(iter_max > 0);

    rot_sim_ptr = new RotationalSimulation(J, 1/rate);
    
}

void reserve_vec_data(const int &N)
{

}

#endif