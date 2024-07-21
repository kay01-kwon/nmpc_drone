#include "ref_model.hpp"
/**
 * @brief Construct a new Ref Model:: Ref Model object
 * 
 * @param inertial_param : mass, moment of inertia, and B_p_CG_COM
 * @param k_p  P gain of translational reference model
 * @param k_v  D gain of translational reference model
 * @param k_q  P gain of orientational reference model
 * @param k_w  D gain of orientational reference model
 */
RefModel::RefModel(const inertial_param_t &inertial_param,
const double& k_p, const double& k_v,
const double& k_q, const double& k_w)
:m_(inertial_param.m),
J_(inertial_param.J),
k_p_(k_p), k_v_(k_v),
k_q_(k_q), k_w_(k_w),
curr_time_(0), prev_time_(0), dt_(0),
u_hat_(u_hat_.setZero()),
mu_hat_(mu_hat_.setZero())
{
    initialize_state_variables();
    assert(J_.size() == 9);
}
/**
 * @brief Construct a new Ref Model:: Ref Model object
 * When using kalman filter instead of controlling reference model,
 * use this one.
 * 
 * @param inertial_param 
 */
RefModel::RefModel(const inertial_param_t &inertial_param)
:m_(inertial_param.m),
J_(inertial_param.J)
{
    initialize_state_variables();
    assert(J_.size() == 9);
}

void RefModel::initialize_state_variables()
{
    s_hat_.setZero();
    s_hat_(6) = 1.0;

    p_hat_.setZero();

    v_hat_.setZero();

    q_hat_.w() = 1;
    q_hat_.x() = 0;
    q_hat_.y() = 0;
    q_hat_.z() = 0;

    w_hat_.setZero();

    grav_.setZero();
    grav_(2) = -9.81;
}

/**
 * @brief set input, state and disturbance in order.
 * 
 * @param u_comp force rejected translational disturbance from rotor thrust
 * @param mu_comp moment compensated orientational disturbance from rotor thrust
 * @param p_state position from meas
 * @param v_state linear velocity from meas
 * @param q_state quaternion from meas
 * @param w_state angular velocity from meas
 * @param sigma_hat noisy translational disturbance
 * @param theta_hat noisy orientational disturbance
 * @param time Current time
 */
void RefModel::set_input_state_disturbance_time(const mat31_t &u_comp, 
const mat31_t &mu_comp, 
const mat31_t &p_state, const mat31_t &v_state,
const quat_t &q_state, const mat31_t &w_state,
const mat31_t &sigma_hat, const mat31_t &theta_hat,
const double &time)
{

    // Get the position and velocity error, respectively.
    mat31_t v_tilde, w_tilde;
    mat33_t C, R, skiew_sym;
    mat31_t q_vec;
    quat_t q_state_conj, q_tilde, unit_q_tilde;

    // Set time
    curr_time_ = time;

    for(size_t i = 0; i < 3; i++)
    {
        s_hat_(i) = p_state(i);
        s_hat_(i+3) = v_state(i);
        s_hat_(i+10) = w_state(i);
    }

    s_hat_(6) = q_state.w();
    s_hat_(7) = q_state.x();
    s_hat_(8) = q_state.y();
    s_hat_(9) = q_state.z();

    u_hat_ = u_comp + sigma_hat;

    // Get rotation matrix from q_tilde
    conjugate(q_state, q_state_conj);
    otimes(q_state_conj, q_hat_, q_tilde);
    convert_quat_to_unit_quat(q_tilde, unit_q_tilde);
    get_rotm_from_quat(unit_q_tilde, R);
    convert_quat_to_quat_vec(unit_q_tilde, q_vec);

    assert(isnan(q_tilde.w()) == false);
    assert(isnan(q_tilde.x()) == false);
    assert(isnan(q_tilde.y()) == false);
    assert(isnan(q_tilde.z()) == false);

    q_vec = signum(unit_q_tilde.w())*q_vec;

    // Get the error of angular velocity
    w_tilde = w_hat_ - R.transpose()*w_state;

    C = R.transpose() * J_.inverse();

    convert_vec_to_skew(w_tilde, skiew_sym);

    for(size_t i = 0; i < theta_hat.size();i++)
        assert(isnan(theta_hat(i)) == false);

    mu_hat_ = C*(mu_comp + R*theta_hat + w_state.cross(J_*w_state))
    - w_tilde.cross(R.transpose()*w_state);

    for(size_t i = 0; i < mu_hat_.size(); i++)
        assert(isnan(mu_hat_(i)) == false);
}

/**
 * @brief User can get estimated state from the reference model.
 * 
 * @param p_ref position of reference model
 * @param v_ref velocity of reference model
 * @param q_ref quaternion of reference model
 * @param w_ref angular velocity of reference model
 */
void RefModel::get_state_from_ref_model(mat31_t &p_ref, mat31_t &v_ref, 
quat_t &q_ref, mat31_t &w_ref) const
{
    p_ref = p_hat_;
    v_ref = v_hat_;
    q_ref = q_hat_;
    w_ref = w_hat_;
}

/**
 * @brief Integrate reference model 
 * after setting input, state, disturbance, and time
 * 
 */
void RefModel::prediction()
{
    dt_ = curr_time_ - prev_time_;

    rk45.do_step([this] 
    (const state13_t& s, state13_t& dsdt, const double& t)
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

    quat_t unit_q_hat;

    q_hat_.w() = s_hat_(6);
    q_hat_.x() = s_hat_(7);
    q_hat_.y() = s_hat_(8);
    q_hat_.z() = s_hat_(9);

    // Convert quaternion of reference into unit quaternion
    // to guarantee stability
    assert(isnan(q_hat_.w()) == false);
    assert(isnan(q_hat_.x()) == false);
    assert(isnan(q_hat_.y()) == false);
    assert(isnan(q_hat_.z()) == false);

    convert_quat_to_unit_quat(q_hat_, unit_q_hat);
    q_hat_ = unit_q_hat;

    for(size_t i = 0; i < w_hat_.size(); i++)
        assert(isnan(w_hat_(i)) == false);

    prev_time_ = curr_time_;
}

/**
 * @brief This implements reference dynamics model.
 * 
 * @param s p_ref, v_ref, q_ref, w_ref
 * @param dsdt dp_ref_dt, dv_ref_dt, dq_ref_dt, dw_ref_dt
 * @param t time
 */
void RefModel::ref_dynamics(const state13_t &s, state13_t &dsdt, const double &t)
{
    mat31_t p,v,dpdt,dvdt;
    quat_t q, q_unit, dqdt;
    mat31_t w, dwdt;

    for(size_t i = 0; i < 3; i++)
    {
        p(i) = s(i);
        v(i) = s(i+3);
    }

    q.w() = s(6);
    q.x() = s(7);
    q.y() = s(8);
    q.z() = s(9);

    for(size_t i = 0; i < 3; i++)
    {
        w(i) = s(i+10);
    }

    dpdt = v;
    dvdt = (1/m_)*u_hat_ + grav_;


    convert_quat_to_unit_quat(q, q_unit);
    get_dqdt(q_unit, w, dqdt);
    dwdt = mu_hat_;

    for(size_t i = 0; i < 3; i++)
    {
        dsdt(i) = dpdt(i);
        dsdt(i+3) = dvdt(i);
        dsdt(i+10) = dwdt(i);
    }

    dsdt(6) = dqdt.w();
    dsdt(7) = dqdt.x();
    dsdt(8) = dqdt.y();
    dsdt(9) = dqdt.z();

}