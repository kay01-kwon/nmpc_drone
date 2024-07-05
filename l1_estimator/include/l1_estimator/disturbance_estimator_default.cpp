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