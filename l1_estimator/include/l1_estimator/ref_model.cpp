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

void RefModel::set_input(const mat31_t &u_comp, 
const mat31_t& mu_comp)
{
    u_hat_ = u_comp;

}

void RefModel::set_state(const mat31_t &p_state, const mat31_t &v_state, 
const quat_t &q_state, const mat31_t& w_state)
{
    mat31_t p_tilde, v_tilde;

    p_tilde = p_hat_ - p_state;
    v_tilde = v_hat_ - v_state;

    u_hat_ -= (k_p_*p_tilde + k_v_*v_tilde);
}

void RefModel::set_est_disturbance(const mat31_t &sigma_est, const mat31_t theta_est)
{
    u_hat_ += sigma_est;
}

void RefModel::set_time(const double &t)
{
    curr_time_ = t;
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
