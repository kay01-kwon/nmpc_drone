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
const mat33_t &gamma_sigma, const mat33_t &gamma_theta, 
const double &tau_sigma, const double &tau_theta)
:m_(inertial_param.m),
J_(inertial_param.J),
convex_fn_obj_{Convex_fn(bound_sigma, epsilon_sigma),Convex_fn(bound_theta, epsilon_theta)},
gamma_prj_obj_{GammaPrj(gamma_sigma), GammaPrj(gamma_theta)},
lpf_obj_{Lpf(tau_sigma), Lpf(tau_theta)},
sigma_hat_(sigma_hat_.setZero()), theta_hat_(theta_hat_.setZero()),
dsigma_hat_(dsigma_hat_.setZero()), dtheta_hat_(dtheta_hat_.setZero()),
sigma_hat_lpf_(sigma_hat_lpf_.setZero()), theta_hat_lpf_(theta_hat_lpf_.setZero()),
D_(D_.setZero()),
prev_time_(0)
{
    
    assert(tau_sigma > std::numeric_limits<double>::min());
    assert(tau_theta > std::numeric_limits<double>::min());
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
    curr_time_ = time;
    mat31_t v_tilde, w_tilde;
    mat31_t y_sigma, y_theta;

    v_tilde = v_ref - v_state;

    w_tilde = w_ref - w_state;


    y_sigma = - v_tilde;
    y_theta = - w_tilde;

    for(size_t i = 0; i < w_tilde.size(); i++)
        assert(isnan(w_tilde(i)) == false);

    double f1, f2;
    mat31_t Df1, Df2;
    
    convex_fn_obj_[0].get_fn_value(sigma_hat_, f1, Df1);
    convex_fn_obj_[1].get_fn_value(theta_hat_, f2, Df2);

    assert(isnan(f1) == false);
    assert(isnan(f2) == false);
    assert(isnan(Df2(0)) == false);
    assert(isnan(Df2(1)) == false);
    assert(isnan(Df2(2)) == false);


    // Get the time derivative of translational and orientational disturbance, respectively.
    gamma_prj_obj_[0].getProjGamma(y_sigma
    , f1, Df1, dsigma_hat_);
    gamma_prj_obj_[1].getProjGamma(y_theta
    , f2, Df2, dtheta_hat_);

    for(size_t i = 0; i < dtheta_hat_.size(); i++)
        assert(isnan(dtheta_hat_(i)) == false);

}

/**
 * @brief Integrate the rate of disturbance, so you can get the noisy disturbance.
 * 
 */
void DisturbanceEstimator::solve()
{
    dt_ = curr_time_ - prev_time_;

    rk4_.do_step([this] 
    (const state6_t& D, state6_t& dDdt, const double& t)
    {
        this->DisturbanceEstimator::system_dynamics(D, dDdt, t);
    },
    D_, prev_time_, dt_);

    // integrate_const(make_dense_output<runge_kutta_dopri5<state6_t>>(1E-6, 1E-3),
    // [this] 
    // (const state6_t& s, state6_t& dsdt, const double& t)
    // {
    //     this->DisturbanceEstimator::system_dynamics(s, dsdt, t);
    // }, D_, prev_time_, curr_time_, dt_*0.1);

    // integrate_adaptive(make_controlled<runge_kutta_cash_karp54<state6_t>>(1E-6, 1E-3),
    // [this] 
    // (const state6_t& s, state6_t& dsdt, const double& t)
    // {
    //     this->DisturbanceEstimator::system_dynamics(s, dsdt, t);
    // }, D_, prev_time_, curr_time_, dt_*0.01);

    // integrate_adaptive(make_controlled<runge_kutta_fehlberg78<state6_t>>(1E-6, 1E-3),
    // [this] 
    // (const state6_t& s, state6_t& dsdt, const double& t)
    // {
    //     this->DisturbanceEstimator::system_dynamics(s, dsdt, t);
    // }, D_, prev_time_, curr_time_, dt_*0.01);

    for(size_t i =  0; i < 3; i++)
    {
        sigma_hat_(i) = D_(i);
        theta_hat_(i) = D_(i+3);
    }

    lpf_obj_[0].set_input_and_time(sigma_hat_, curr_time_);
    lpf_obj_[0].solve();
    lpf_obj_[0].get_filtered_vector(sigma_hat_lpf_);

    lpf_obj_[1].set_input_and_time(theta_hat_, curr_time_);
    lpf_obj_[1].solve();
    lpf_obj_[1].get_filtered_vector(theta_hat_lpf_);

    prev_time_ = curr_time_;

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