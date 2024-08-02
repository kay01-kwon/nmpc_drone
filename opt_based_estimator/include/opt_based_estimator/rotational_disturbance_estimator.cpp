#include "rotational_disturbance_estimator.hpp"

RotDistEst::RotDistEst(const mat33_t &J, const mat77_t &Q, 
const double &term_error, const int &iter_max)
:J_nom_(J), Q_(Q), term_error_(term_error), iter_max_(iter_max),
rot_rk4_grad_obj_(J_nom_), curr_time_(0), prev_time_(0), dt_(0),
theta_k_(theta_k_.setZero()), M_(M_.setZero())
{
    s_init_.setZero();
    s_meas_.setZero();
    s_rk4_.setZero();

    s_init_(0) = 1.0;
    s_meas_(0) = 1.0;
    s_rk4_(0) = 1.0;

    assert(term_error > 0);
    assert(iter_max > 0);
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

void RotDistEst::set_input(const vector_t &M, const vector_t &theta)
{
    M_ = M;
    theta_k_ = theta;
}

vector_t RotDistEst::get_est_dist() const
{
    return theta_k_;
}

void RotDistEst::solve()
{
    dt_ = curr_time_ - prev_time_;
    double error = 1;
    uint8_t iter = 0;

    quaternion_t q_rk4_temp_conj, q_meas, q_tilde;

    rotational_state_t s_tilde;

    double sign;

    mat73_t RK_grad;
    vector_t gradient_f;
    mat33_t Hessian_f;

    q_meas.w() = s_meas_(0);
    q_meas.x() = s_meas_(1);
    q_meas.y() = s_meas_(2);
    q_meas.z() = s_meas_(3);

    rot_rk4_grad_obj_.set_time_difference(dt_);
    rot_rk4_grad_obj_.set_initial_state(s_init_);
    rot_rk4_grad_obj_.set_input_disturbance(M_, theta_k_);

    while(fabs(error) > term_error_)
    {
        rk4_solver_obj_.do_step(
            [this](const rotational_state_t &s, rotational_state_t &dsdt, const double &t)
            {
                this->RotDistEst::nominal_dynamics(s, dsdt, t, M_, theta_k_);
            }, s_rk4_, prev_time_, dt_
        );

        q_rk4_temp_conj.w() = s_rk4_(0);
        q_rk4_temp_conj.x() = -s_rk4_(1);
        q_rk4_temp_conj.y() = -s_rk4_(2);
        q_rk4_temp_conj.z() = -s_rk4_(3);

        q_tilde = otimes(q_rk4_temp_conj, q_meas);

        s_tilde(0) = q_tilde.w();

        sign = signum(q_tilde.w());

        s_tilde(1) = sign * q_tilde.x();
        s_tilde(2) = sign * q_tilde.y();
        s_tilde(3) = sign * q_tilde.z();
        
        for(size_t i = 0; i < 3; i++)
            s_tilde(i+4) = s_meas_(i+4) - s_rk4_(i+4);
        
        RK_grad = rot_rk4_grad_obj_.getRK4Grad();

        gradient_f = - dt_/6.0 * RK_grad.transpose() * Q_ * s_tilde;

        Hessian_f = dt_*dt_/36.0 * RK_grad.transpose() * Q_ * RK_grad;

        theta_k_ = theta_k_ - Hessian_f.inverse() * gradient_f;

        s_rk4_ = s_init_;

        if(iter >= iter_max_)
        {
            break;
        }
    }

    // Iteration is finished
    // Set the initial state as measured one.
    s_init_ = s_meas_;
    // Set the previous time as current time
    prev_time_ = curr_time_;

}

void RotDistEst::nominal_dynamics(const rotational_state_t &s, rotational_state_t &dsdt, 
const double &time, const vector_t &M, const vector_t &theta)
{
    quaternion_t q, q_mul, dqdt;
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
    
    q_mul = otimes(q, w);

    // dqdt = 0.5 * q otimes [0;w]
    dqdt.w() = 0.5*q_mul.w();
    dqdt.x() = 0.5*q_mul.x();
    dqdt.y() = 0.5*q_mul.y();
    dqdt.z() = 0.5*q_mul.z();

    dwdt = J_nom_.inverse()  * (M - w.cross(J_nom_*w) + theta);
}
