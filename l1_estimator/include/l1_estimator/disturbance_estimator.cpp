#include "disturbance_estimator.hpp"

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

    gamma_prj_obj_[0].getProjGamma(sigma_hat_, y_sigma
    , f1, Df1, dsigma_hat_);
    
    gamma_prj_obj_[1].getProjGamma(theta_hat_, y_theta
    , f2, Df2, dtheta_hat_);


    lpf_obj_[0].set_input_and_time(sigma_hat_, curr_time_);
    lpf_obj_[0].solve();
    lpf_obj_[0].get_filtered_vector(sigma_hat_lpf_);


    lpf_obj_[1].set_input_and_time(theta_hat_, curr_time_);
    lpf_obj_[1].solve();
    lpf_obj_[1].get_filtered_vector(theta_hat_lpf_);

}

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
}

void DisturbanceEstimator::get_est_raw(mat31_t &sigma_est, 
mat31_t &theta_est) const
{
    sigma_est = sigma_hat_;
    theta_est = theta_hat_;
}

void DisturbanceEstimator::get_est_filtered(mat31_t &sigma_est_filtered, 
mat31_t &theta_est_filtered) const
{
    sigma_est_filtered = sigma_hat_lpf_;
    theta_est_filtered = theta_hat_lpf_;
}

void DisturbanceEstimator::system_dynamics(const state6_t &D, state6_t &dDdt, const double t)
{
    state6_t v_in;
    
    for(size_t i = 0; i < 3; i++)
    {
        v_in(i) = dsigma_hat_(i);
        v_in(i + 3) = dtheta_hat_(i);
    }

    dDdt = v_in;
}
