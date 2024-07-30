#include "rotational_disturbance_estimator.hpp"

RotDistEst::RotDistEst(const mat33_t &J, const mat77_t &Q, 
const double &term_error, const uint8_t &iter_max)
:J_nom_(J), Q_(Q), term_error_(term_error), iter_max_(iter_max),
rot_rk4_grad_obj_(J_nom_), curr_time_(0), prev_time_(0), dt_(0)
{
    s_init_.setZero();
    s_meas_.setZero();
    s_rk4_.setZero();

    s_init_(0) = 1.0;
    s_meas_(0) = 1.0;
    s_rk4_(0) = 1.0;
}

void RotDistEst::set_time(const double &curr_time)
{
    curr_time_ = curr_time;
}

void RotDistEst::set_meas_state(const quaternion_t &q_meas, const vector_t &w_meas)
{
    s_meas_(0) = q_meas.w();
    s_meas_(1) = q_meas.x();
    s_meas_(2) = q_meas.y();
    s_meas_(3) = q_meas.z();

    s_meas_(4) = w_meas(0);
    s_meas_(5) = w_meas(1);
    s_meas_(6) = w_meas(2);
    
}

void RotDistEst::solve()
{
}

void RotDistEst::nominal_dynamics(const rotational_state_t &s, rotational_state_t &dsdt, 
const double &time, const vector_t &M, const vector_t &theta)
{
    quaternion_t q, w_quaternion_form, q_temp, dqdt;
    vector_t w, dwdt;

    q.w() = s(0);
    q.x() = s(1);
    q.y() = s(2);
    q.z() = s(3);

    // Convert the quaternion into unit quaternion
    q.normalize();
    
    w(0) = s(4);
    w(1) = s(5);
    w(2) = s(6);
    
    q_temp = otimes(q, w);

    // dqdt = 0.5 * q otimes [0;w]
    dqdt.w() = 0.5*q_temp.w();
    dqdt.x() = 0.5*q_temp.x();
    dqdt.y() = 0.5*q_temp.y();
    dqdt.z() = 0.5*q_temp.z();

    dwdt = J_nom_.inverse()  * (M - w.cross(J_nom_*w) + theta);
}
