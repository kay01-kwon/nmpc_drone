#include "disturbance_estimator_default.hpp"

DisturbanceEstimator::DisturbanceEstimator(const Inertial_param &inertial_param, 
const mat31_t &bound_sigma, const double &epsilon_sigma, 
const mat31_t &bound_theta, const double &epsilon_theta, 
const mat31_t &gamma_sigma, const mat33_t &gamma_theta, 
const double &tau_sigma, const double &tau_theta)
:inertial_param_(inertial_param),
convex_fn_obj_{Convex_fn(bound_sigma, epsilon_sigma),
Convex_fn(bound_theta, epsilon_theta)},
gamma_prj_obj_{GammaPrj(gamma_sigma), 
GammaPrj(gamma_theta)},
lpf_obj_{Lpf(tau_sigma), 
Lpf(tau_theta)}
{
}

void DisturbanceEstimator::set_state_time(const mat31_t &p_state, const mat31_t &p_ref, 
const mat31_t &v_state, const mat31_t &v_ref, 
const quat_t &q_state, const quat_t &q_ref, 
const mat31_t &w_state, const mat31_t &w_ref, 
const double &time)
{
    mat31_t p_tilde, v_tilde;

    p_tilde = p_ref - p_state;

    v_tilde = v_ref - v_state;

    curr_time_ = time;

    quat_t q_conj, q_tilde;
    mat33_t R;
    mat31_t w_tilde;

    conjugate(q_state, q_conj);
    otimes(q_conj, q_ref, q_tilde);
    get_rotm_from_quat(q_tilde,R);
    w_tilde = w_ref - R.transpose()*w_state;
    
}
