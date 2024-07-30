#ifndef ROTATIONAL_DISTURBANCE_ESTIMATOR_HPP
#define ROTATIONAL_DISTURBANCE_ESTIMATOR_HPP
#include "rotational_rk4_gradient.hpp"
#include "ode_solver/custom_rk4_ode.h"

class RotDistEst{

    public:

    RotDistEst() = delete;

    RotDistEst(const mat33_t& J, const mat77_t &Q,
    const double &term_error = 1E-3, const uint8_t &iter_max = 30);

    void set_time(const double &curr_time);
    void set_meas_state(const quaternion_t &q_meas, const vector_t &w_meas);

    void solve();

    private:

    mat33_t J_nom_;
    mat77_t Q_;
    double term_error_;
    uint8_t iter_max_;

    // Object to solve runge kutta 4th order 
    // and its gradient w.r.t disturbance
    OdeRK4Custom<rotational_state_t> rk4_solver_obj_;
    RotRK4Grad rot_rk4_grad_obj_;

    rotational_state_t s_rk4_;
    rotational_state_t s_meas_;

    void nominal_dynamics(const rotational_state_t &s,
    rotational_state_t &dsdt, const double &time,
    const vector_t &M, const vector_t &theta);

    vector_t theta_k_;

    double curr_time_, prev_time_, dt_;

};

#endif