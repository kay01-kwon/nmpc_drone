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
    Jxx = J_(0);
    Jyy = J_(4);
    Jzz = J_(8);
    J_x_ << (Jzz-Jyy), (Jxx-Jzz), (Jyy-Jxx);

    DK1_.setZero();
    DK1_.block(4,0,4,1) = J_.inverse();
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

void RotRK4Grad::set_input_disturbance(const vector_t &M_init, const vector_t &theta_init)
{
    M_ = M_init;
    theta_ = theta_init;
}

mat73_t RotRK4Grad::getRK4Grad() const
{

    mat41_t q_m1;
    vector_t w_m1;

    q_m1 = q_init_ + dt_/2.0*1/2.0*otimes(q_init_, w_init_);
    w_m1 = w_init_ + dt_/2.0*J_.inverse()*(M_ - w_init_.cross(J_*w_init_) + theta_);



    return mat73_t();
}
