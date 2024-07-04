#include "ref_model.hpp"

RefModel::RefModel(const Inertial_param_t &inertial_param,
const double& k_p, const double& k_v,
const double& k_q, const double& k_w):
inertial_param_(inertial_param), 
k_p_(k_p), k_v_(k_v),
k_q_(k_q), k_w_(k_w),
curr_time_(0), prev_time_(0), dt_(0)
{
    u_hat_.setZero();
    mu_hat_.setZero();

    s_hat_.setZero();
    s_hat_(6) = 1.0;

    p_hat_.setZero();
    v_hat_.setZero();

    q_hat_.w() = 1;
    q_hat_.x() = 0;
    q_hat_.y() = 0;
    q_hat_.z() = 0;

    w_hat_.setZero();

    grav.setZero();
    grav(2) = -9.81;
}

void RefModel::set_input_state_disturbance_time(const mat31_t &u_comp, 
const mat31_t &mu_comp, const state13_t &s, 
const mat31_t &sigma_est, const mat31_t &theta_est,
const double &time)
{
    // Set time
    curr_time_ = time;
    // Temporarily store state values.
    mat31_t p_state, v_state;

    quat_t q_state;
    mat31_t w_state;

    for(size_t i = 0; i < 3; i++)
    {
        p_state(i) = s(i);
        v_state(i) = s(i+3);
        w_state(i) = s(i+10);
    }

    q_state.w() = s(6);
    q_state.x() = s(7);
    q_state.y() = s(8);
    q_state.z() = s(9);
    
    // Get the position and velocity error, respectively.
    mat31_t p_tilde, v_tilde;

    p_tilde = p_hat_ - p_state;
    v_tilde = v_hat_ - v_state;

    // Control translational reference model.
    u_hat_ = u_comp - (k_p_*p_tilde + k_v_*v_tilde);


    mat33_t C, R, skiew_sym;
    mat31_t q_vec;

    // Get the current state of quaternion
    // and then compute the quaternion error.
    quat_t q_state_conj, q_tilde;

    conjugate(q_state, q_state_conj);
    otimes(q_state_conj, q_hat_, q_tilde);

    // Get rotation matrix from q_tilde
    get_rotm_from_quat(q_tilde, R);
    convert_quat_to_quat_vec(q_tilde, q_vec);

    q_vec = signum(q_tilde.w())*q_vec;

    mat31_t w_tilde;

    // Get the error of angular velocity
    w_tilde = w_hat_ - R.transpose()*w_state;

    C = inertial_param_.J 
    * R.transpose() 
    * inertial_param_.J.inverse();

    convert_vec_to_skew(w_tilde, skiew_sym);

    mu_hat_ = C
    *(mu_comp + R*theta_est)
    - inertial_param_.J
    *skiew_sym
    *R.transpose()*w_state
    -(k_q_*q_vec + k_w_*w_tilde);

}

void RefModel::get_state_from_ref_model(mat31_t &p_ref, mat31_t &v_ref, 
quat_t &q_ref, mat31_t &w_ref) const
{
    p_ref = p_hat_;
    v_ref = v_hat_;
    q_ref = q_hat_;
    w_ref = w_hat_;
}

void RefModel::solve()
{
    dt_ = curr_time_ - prev_time_;

    rk4.do_step([this] 
    (const mat31_t& s, mat31_t& dsdt, const double& t)
    {
        this->RefModel::ref_dynamics(s, dsdt, t);
    },
    s_hat_, prev_time_, dt_);

    // Copy the state
    for(size_t i = 0; i < 3; i++)
    {
        p_hat_(i) = s_hat_(i);
        v_hat_(i) = s_hat_(i+3);
        w_hat_(i) = s_hat_(i+10);
    }

    q_hat_.w() = s_hat_(6);
    q_hat_.x() = s_hat_(7);
    q_hat_.y() = s_hat_(8);
    q_hat_.z() = s_hat_(9);

    prev_time_ = curr_time_;
}

void RefModel::ref_dynamics(const mat31_t &s, mat31_t &dsdt, const double &t)
{
}
