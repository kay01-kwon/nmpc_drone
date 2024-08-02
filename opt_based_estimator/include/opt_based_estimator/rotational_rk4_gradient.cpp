#include "rotational_rk4_gradient.hpp"

RotRK4Grad::RotRK4Grad(const mat33_t &J)
:J_(J), 
a1_(static_cast<double>(1.0)), 
a2_(static_cast<double>(2.0)),
a3_(static_cast<double>(2.0)),
a4_(static_cast<double>(1.0)),
dt_(0.01)
{
    double Jxx, Jyy, Jzz;
    Jxx = J_(0,0);
    Jyy = J_(1,1);
    Jzz = J_(2,2);
    J_x_ << (Jzz-Jyy), (Jxx-Jzz), (Jyy-Jxx);

    Eye_.setIdentity();
}

void RotRK4Grad::set_time_difference(const double &dt)
{
    dt_ = dt;
}

void RotRK4Grad::set_initial_state(const quaternion_t &q_init, const vector_t &w_init)
{
    q_init_(0) = q_init.w();
    q_init_(1) = q_init.x();
    q_init_(2) = q_init.y();
    q_init_(3) = q_init.z();

    w_init_ = w_init;
}

void RotRK4Grad::set_initial_state(const rotational_state_t &s_init)
{
    for(size_t i = 0; i < 4; i++)
        q_init_(i) = s_init(i);
    
    for(size_t i = 0; i < 3; i++)
        w_init_(i) = s_init(i+4);
}

void RotRK4Grad::set_input_disturbance(const vector_t &M_init, const vector_t &theta_init)
{
    M_ = M_init;
    theta_ = theta_init;
}

mat73_t RotRK4Grad::getRK4Grad() const
{
    mat41_t q_temp, q_temp_prev;
    vector_t w_temp, w_temp_prev;

    mat43_t dq_temp;
    mat33_t dw_temp;

    mat33_t diff_inertial_temp;

    mat73_t DK1;

    DK1.setZero();
    DK1.block(4, 0, 3, 3) = J_.inverse();

    // q_temp (q_m1), w_temp (w_m1)
    q_temp = q_init_ + 0.5 * dt_ * 0.5 * otimes(q_init_, w_init_);
    w_temp = w_init_ + 0.5 * dt_ * J_.inverse()*(M_ - w_init_.cross(J_*w_init_) + theta_);

    // dq_temp (dq_m1), dw_temp (dw_m1)
    dq_temp.setZero();
    dw_temp = 0.5 * dt_ * J_.inverse();

    mat73_t DK2;
    DK2.block(0, 0, 4, 3) = 0.5 * otimes(q_temp, dw_temp);
    // diff_inertial_temp D(w_m1 x J*w_m1)
    diff_inertial_temp = DiffInertial(w_temp, dw_temp);
    DK2.block(4, 0, 3, 3) = J_.inverse()*(-diff_inertial_temp + Eye_);

    // q_temp_prev (q_m1), w_temp_prev (w_m1)
    q_temp_prev = q_temp;
    w_temp_prev = w_temp;

    // q_temp (q_m2), w_temp (w_m2)
    q_temp = q_init_ + 0.5 * dt_ * 0.5 * otimes(q_temp_prev, w_temp_prev);
    w_temp = w_init_ + 0.5 * dt_ * J_.inverse() * (M_ - w_temp_prev.cross(J_*w_temp_prev) + theta_);

    // dq_temp (dq_m2), dw_temp (dw_m2)
    dq_temp = 0.5 * dt_ * 0.5 * otimes(q_temp_prev, dw_temp);
    // diff_inertial_temp D(w_m1 x J*w_m1)
    dw_temp = 0.5 * dt_ * J_.inverse() *(-diff_inertial_temp + Eye_);

    mat73_t DK3;
    DK3.block(0, 0, 4, 3) = 0.5 * ( otimes(dq_temp, w_temp) + otimes(q_temp, dw_temp) );
    // diff_inertial_temp D(w_m2 x J*w_m2)
    diff_inertial_temp = DiffInertial(w_temp, dw_temp);
    DK3.block(4, 0, 3, 3)  = J_.inverse()*(-diff_inertial_temp + Eye_);

    // q_temp_prev (q_m2), w_temp_prev (w_m2)
    q_temp_prev = q_temp;
    w_temp_prev = w_temp;

    // q_temp (q_f), w_temp (w_f)
    q_temp = q_init_ + dt_*(0.5 * otimes(q_temp_prev, w_temp_prev));
    w_temp = w_init_ + dt_*J_.inverse()*(M_ - w_temp_prev.cross(J_*w_temp_prev) + theta_);

    // dq_temp (dq_f), dw_temp (dw_f)
    dq_temp = dt_* 0.5 * (otimes(dq_temp, w_temp_prev) + otimes(q_temp_prev, dw_temp));
    // diff_inertial_temp = D(w_m2 x J*w_m2)
    dw_temp = dt_*J_.inverse()*(-diff_inertial_temp + Eye_);

    mat73_t DK4;
    DK4.block(0, 0, 4, 3) = 0.5*( otimes(dq_temp, w_temp) + otimes(q_temp, dw_temp) );
    // diff_inertial_temp = D(w_f x J*w_f)
    diff_inertial_temp = DiffInertial(w_temp, dw_temp);
    DK4.block(4, 0, 3, 3) = J_.inverse()*(-diff_inertial_temp + Eye_);
    
    return a1_*DK1 + a2_*DK2 + a3_*DK3 + a4_*DK4;
}

mat33_t RotRK4Grad::DiffInertial(const vector_t &w, const mat33_t &dw) const
{
    mat33_t temp;

    temp << dw(1,0)*w(2)+w(1)*dw(2,0), dw(1,1)*w(2)+w(1)*dw(2,1), dw(1,2)*w(2)+w(1)*dw(2,2),
    dw(0,0)*w(2)+w(0)*dw(2,0), dw(0,1)*w(2)+w(0)*dw(2,1), dw(0,2)*w(2)+w(0)*dw(2,2),
    dw(0,0)*w(1)+w(0)*dw(1,0), dw(0,1)*w(1)+w(0)*dw(1,1), dw(0,2)*w(1)+w(0)*dw(1,2);

    temp = J_x_*temp;

    return temp;
}