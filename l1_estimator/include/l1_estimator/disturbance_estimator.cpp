#include "disturbance_estimator.hpp"

/**
 * @brief Construct a new Disturbance Estimator:: Disturbance Estimator object
 * 
 * @param inertial_param 
 * @param bound_sigma translational disturbance bound for convex function
 * @param epsilon_sigma epsilon for convex function (translational dynamics)
 * @param bound_theta orientational disturbance bound for convex function
 * @param epsilon_theta epsilon for convex function (orientational dynamics)
 * @param gamma_sigma Gain for Gamma projection for translational dynamics
 * @param gamma_theta Gain for Gamma projection for orientational dynamics
 * @param tau_sigma Low pass filter parameter to filter noisy translational disturbance
 * @param tau_theta Low pass filter parameter to filter noise orientational disturbance
 */
DisturbanceEstimator::DisturbanceEstimator(const inertial_param_t &inertial_param, 
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

/**
 * @brief set estimated state and current time stamp
 * 
 * @param p_state position from estimator
 * @param p_ref position from reference model
 * @param v_state velocity from estimator
 * @param v_ref velocity from reference model
 * @param q_state quaternion from estimator
 * @param q_ref quaterion from reference model
 * @param w_state angular velocity from estimator
 * @param w_ref angular velocity from reference model
 * @param time current time
 */
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

    mat31_t y_sigma, y_theta;
    mat33_t R, P, P_transpose;
    mat33_t J_inv, J_inv_transpose;
    mat31_t q_vec;
    double c = 1.0;

    J_inv = inertial_param_.J.inverse();
    J_inv_transpose = J_inv.transpose();

    quat_t q_tilde_unit;

    convert_quat_to_unit_quat(q_tilde, q_tilde_unit);

    get_rotm_from_quat(q_tilde_unit,R);
    convert_quat_to_quat_vec(q_tilde, q_vec);
    q_vec = signum(q_tilde.w()) * q_vec;

    P = inertial_param_.J 
    * R.transpose() 
    * inertial_param_.J.inverse()
    * R;

    P_transpose = P.transpose();

    y_sigma = - v_tilde;
    y_theta = - P_transpose * w_tilde
    -c*P_transpose*J_inv_transpose*q_vec;

    double f1, f2;
    mat31_t Df1, Df2;
    
    convex_fn_obj_[0].get_fn_value(sigma_hat_, f1, Df1);
    convex_fn_obj_[1].get_fn_value(theta_hat_, f2, Df2);


    // Get the time derivative of translational and orientational disturbance, respectively.
    gamma_prj_obj_[0].getProjGamma(y_sigma
    , f1, Df1, dsigma_hat_);
    gamma_prj_obj_[1].getProjGamma(y_theta
    , f2, Df2, dtheta_hat_);


    lpf_obj_[0].set_input_and_time(sigma_hat_, curr_time_);
    lpf_obj_[0].solve();
    lpf_obj_[0].get_filtered_vector(sigma_hat_lpf_);


    lpf_obj_[1].set_input_and_time(theta_hat_, curr_time_);
    lpf_obj_[1].solve();
    lpf_obj_[1].get_filtered_vector(theta_hat_lpf_);

}

/**
 * @brief Integrate the rate of disturbance, so you can get the noisy disturbance.
 * 
 */
void DisturbanceEstimator::solve()
{
    dt_ = curr_time_ - prev_time_;

    rk4.do_step([this] 
    (const state6_t& D, state6_t& dDdt, const double& t)
    {
        this->DisturbanceEstimator::system_dynamics(D, dDdt, t);
    },
    D_, prev_time_, dt_);
    prev_time_ = curr_time_;

    for(size_t i =  0; i < 3; i++)
    {
        sigma_hat_(i) = D_(i);
        theta_hat_(i) = D_(i+3);
    }
}

/**
 * @brief noisy disturbance since it is not filted by low pass filter
 * 
 * @param sigma_est noisy translational disturbance
 * @param theta_est noise orientational disturbance
 */
void DisturbanceEstimator::get_est_raw(mat31_t &sigma_est, 
mat31_t &theta_est) const
{
    sigma_est = sigma_hat_;
    theta_est = theta_hat_;
}

/**
 * @brief Disturbance filtered by low pass filter
 * 
 * @param sigma_est_filtered estimated translational disturbance (filtered by low pass filter)
 * @param theta_est_filtered estimated orientational disturbance (filtered by low pass filter)
 */
void DisturbanceEstimator::get_est_filtered(mat31_t &sigma_est_filtered, 
mat31_t &theta_est_filtered) const
{
    sigma_est_filtered = sigma_hat_lpf_;
    theta_est_filtered = theta_hat_lpf_;
}

/**
 * @brief the time derivative of disturbance should be matched to the vector obtained from gamma projection.
 * When you set the disturbance, gamma projection automatically computes the rate of it.
 * 
 * @param D resultant disturbance (noisy one)
 * @param dDdt (the rate of disturbance)
 * @param t (time)
 */
void DisturbanceEstimator::system_dynamics(const state6_t &D, state6_t &dDdt, const double t)
{    
    for(size_t i = 0; i < 3; i++)
    {
        dDdt(i) = dsigma_hat_(i);
        dDdt(i + 3) = dtheta_hat_(i);
    }
}
