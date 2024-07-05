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


    void get_est_raw(mat31_t& sigma_est, mat31_t& theta_est);

    void get_est_filtered(mat31_t& sigma_est, mat31_t& theta_est);

    private:

};




#endif