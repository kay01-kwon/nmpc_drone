#include "ref_model.hpp"

RefModel::RefModel(const Inertial_param_t &inertial_param,
const double& k_p, const double& k_v,
const double& k_q, const double& k_w):
inertial_param_(inertial_param), 
k_p_(k_p), k_v_(k_v)
k_q_(k_q), k_w_(k_w),
curr_time_(0), prev_time_(0), dt_(0)
{
    u_hat_.setZero();
    mu_hat_.setZero();

    s_hat_.setZero();
    s_hat_(6) = 1.0;

    grav.setZero();
    grav(2) = -9.81;
}

void RefModel::set_input(const mat31_t &u_comp, const mat31_t mu_comp)
{
}

void RefModel::set_state(const mat31_t &p_state, const mat31_t &v_state, const quat_t &q_state, const mat31_t w_state)
{
}

void RefModel::set_est_disturbance(const mat31_t &sigma_est, const mat31_t theta_est)
{
}

void RefModel::set_time(const double &t)
{
}

void RefModel::get_state_from_ref_model(const mat31_t &p_ref, const mat31_t &v_ref, const quat_t &q_ref, const mat31_t &w_ref)
{
}

void RefModel::solve()
{
}

void RefModel::ref_dynamics(const mat31_t &s, mat31_t &dsdt, const double &t)
{
}
