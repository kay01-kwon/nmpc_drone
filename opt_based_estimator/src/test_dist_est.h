#ifndef TEST_DIST_EST_H
#define TEST_DIST_EST_H
#include <ros/ros.h>
#include "opt_based_estimator/rotational_disturbance_estimator.hpp"
#include "simulation_model/rotational_simulation_class.hpp"
#include "utils/plot_tools.hpp"
#include <numeric>
#include <vector>

#define QUATERNION_T_DIM    4
#define VECTOR_T_DIM        3

using ros::NodeHandle;
using std::vector;

void set_parameter(const NodeHandle &nh_);

void reserve_vec_data(const size_t &N_, const size_t &dim_,
vector< vector<double> > &data_);

void print_parameter(const mat33_t &J_, const double &Tf_,
const double &rate_, const mat77_t &Q_,
const double &term_error_, const int &iter_max_);

void push_back_vector(const vector_t &vec_, vector< vector<double> > &plot_vec_);

void push_back_vector(const quaternion_t &q_vec_, vector< vector<double> > &plot_vec_);

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

vector<double> time_vec;
vector<vector<double> > q_obs_vec;
vector<vector<double> > w_obs_vec;
vector<vector<double> > theta_true_vec;
vector<vector<double> > theta_est_vec;

// Simulation final time, time difference, and the total step
double Tf;
double rate;
size_t N;
double curr_time, prev_time;

size_t x_tick_size, y_tick_size;

double line_width, label_font_size, tick_font_size;

string title_name, y_label_name, data1_name, data2_name;

size_t idx[3] = {0, 1, 2};

void set_parameter(const NodeHandle &nh_)
{

    // Initialize moment of inertia
    J.setZero();

    // Get MOI parameter from launch file
    nh_.getParam("J_xx", J(0,0));
    nh_.getParam("J_yy", J(1,1));
    nh_.getParam("J_zz", J(2,2));

    // Get simulation time information from launch file
    nh_.getParam("rate", rate);
    nh_.getParam("Tf", Tf);

    assert(rate >= 100);
    assert(Tf > 1/rate);

    // Initialize weight matrix for optimization based disturnbace estimator
    Q.setZero();
    nh_.getParam("Q_qw", Q(0,0));
    nh_.getParam("Q_qx", Q(1,1));
    nh_.getParam("Q_qy", Q(2,2));
    nh_.getParam("Q_qz", Q(3,3));
    nh_.getParam("Q_wx", Q(4,4));
    nh_.getParam("Q_wy", Q(5,5));
    nh_.getParam("Q_wz", Q(6,6));

    nh_.getParam("term_error", term_error);
    nh_.getParam("iter_max", iter_max);

    assert(iter_max > 0);

    N = Tf*rate;
    double dt = 1/rate;

    rot_sim_ptr = new RotationalSimulation(J, dt);
    rot_dist_est_ptr = new RotDistEst(J, Q, term_error, iter_max);

    print_parameter(J, Tf, rate, Q, term_error, iter_max);

    // Reserve time vector, obs state variable, and 
    // true and estimated disturbacne data
    time_vec.reserve(N);
    size_t size_time_vec;
    size_time_vec = time_vec.capacity();
    reserve_vec_data(N, QUATERNION_T_DIM, q_obs_vec);
    reserve_vec_data(N, VECTOR_T_DIM, w_obs_vec);
    reserve_vec_data(N, VECTOR_T_DIM, theta_true_vec);
    reserve_vec_data(N, VECTOR_T_DIM, theta_est_vec);

    curr_time = 0;
    prev_time = curr_time;

    theta_est.setZero();

    x_tick_size = 5;
    y_tick_size = 5;

    line_width = 2;
    label_font_size = 30;
    tick_font_size = 25;

}

void reserve_vec_data(const size_t &N_, const size_t &dim_, vector< vector<double> > &data_)
{
    data_.reserve(dim_);

    cout << "Data length: " << N_ << endl;

    size_t n;
    n = data_.capacity();
    // cout << n << endl;

    size_t m;

    for(size_t i = 0; i < dim_; i++)
    {
        data_.push_back(vector<double>());
        data_[i].reserve(N_);
        m = data_[i].capacity();
        assert(m == N_);
        // cout << m << endl;
    }
}

void print_parameter(const mat33_t &J_, const double &Tf_, const double &rate_,
const mat77_t &Q_, const double &term_error_, const int &iter_max_)
{
    cout << "*************************************" << endl;
    cout << "MOI (kg m^3) setup" << endl;
    cout << J_ <<endl;

    cout << "*************************************" << endl;
    cout << "Final time (s): " << Tf_ << endl;

    cout << "*************************************" << endl;
    cout << "Rate (Hz): " << rate_ << endl;

    cout << "*************************************" << endl;
    cout << "Setup for rotataional disturbance estimator" << endl;
    
    cout << "Wieght info" << endl;
    cout << Q_ << endl;

    cout << "Termination of state error: " << term_error_ << endl;

    cout << "Allowable maximum iteration: " << iter_max_ << endl;

}

void push_back_vector(const vector_t &vec_, vector<vector<double>> &plot_vec_)
{
    for(size_t i = 0; i < VECTOR_T_DIM; i++)
        plot_vec_[i].push_back(vec_(i));
}

inline void push_back_vector(const quaternion_t &q_vec_, vector<vector<double>> &plot_vec_)
{
    plot_vec_[0].push_back(q_vec_.w());
    plot_vec_[1].push_back(q_vec_.x());
    plot_vec_[2].push_back(q_vec_.y());
    plot_vec_[3].push_back(q_vec_.z());
}

#endif