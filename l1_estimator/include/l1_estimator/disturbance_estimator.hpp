#ifndef DISTURBANCE_ESTIMATOR_HPP_
#define DISTURBANCE_ESTIMATOR_HPP_
#include <numeric>
#include "convex_fn.hpp"
#include "gamma_proj.hpp"
#include "low_pass_filter.hpp"

class DisturbanceEstimator{

    public:

    DisturbanceEstimator() = delete;

    DisturbanceEstimator(const inertial_param_t& Inertial_param,
    const mat31_t& bound_sigma, const double& epsilon_sigma,
    const mat31_t& bound_theta, const double& epsilon_theta,
    const mat33_t& gamma_sigma, 
    const mat33_t& gamma_theta, 
    const double& tau_sigma,
    const double& tau_theta);

    void set_state_time(const mat31_t& p_state, const mat31_t& p_ref,
    const mat31_t& v_state, const mat31_t& v_ref,
    const quat_t& q_state, const quat_t& q_ref,
    const mat31_t& w_state, const mat31_t& w_ref,
    const double& time);

    void solve();

    void get_est_raw(mat31_t& sigma_est, 
    mat31_t& theta_est) const;

    void get_est_filtered(mat31_t& sigma_est_filtered, 
    mat31_t& theta_est_filtered) const;

    ~DisturbanceEstimator() = default;
    
    private:

    double m_;
    mat33_t J_;

    mat31_t sigma_hat_, theta_hat_;
    mat31_t dsigma_hat_, dtheta_hat_;
    mat31_t sigma_hat_lpf_, theta_hat_lpf_;
    state6_t D_;

    double curr_time_, prev_time_, dt_;
    
    Convex_fn convex_fn_obj_[2];
    GammaPrj gamma_prj_obj_[2];
    Lpf lpf_obj_[2];

    runge_kutta4<state6_t> rk4_;

    runge_kutta_dopri5<state6_t> rk45;

    void system_dynamics(const state6_t& D, state6_t& dDdt, const double t);

};

#endif