#ifndef DISTURBANCE_ESTIMATOR_DEFAULT_HPP_
#define DISTURBANCE_ESTIMATOR_DEFAULT_HPP_
#include "convex_fn.hpp"
#include "gamma_proj.hpp"
#include "low_pass_filter.hpp"

class DisturbanceEstimator{

    public:

    DisturbanceEstimator() = delete;

    DisturbanceEstimator(const Inertial_param& Inertial_param);

    void set_state_time(const mat31_t& p_state, const mat31_t& p_ref,
    const mat31_t& v_state, const mat31_t& v_ref,
    const quat_t& q_state, const quat_t& q_ref,
    const mat31_t& w_state, const quat_t& w_ref,
    const double& time);

    void get_est_raw(mat31_t& sigma_est, mat31_t& theta_est) const;

    void get_est_filtered(mat31_t& sigma_est, mat31_t& theta_est) const;

    void solve();

    private:

    Inertial_param nominal_param_;

    quat_t q_tilde_;
    mat31_t p_tilde_, v_tilde_, w_tilde_;
    mat31_t sigma_hat_, theta_hat_;
    mat31_t dsigma_hat_, dtheta_hat_;
    mat31_t sigma_hat_lpf_, theta_hat_lpf_;
    state6_t s_;

    double curr_time_, prev_time_, dt_;
    
    Convex_fn conv_fn_obj[2];
    GammaPrj gamma_prj_obj[2];
    runge_kutta4<state6_t> rk4;
    
    Lpf lpf_obj[2];

    void system_dynamics(const state6_t& s, state6_t& dsdt, const double t);

};




#endif