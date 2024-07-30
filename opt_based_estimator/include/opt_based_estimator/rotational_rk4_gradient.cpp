#include "rotational_rk4_gradient.hpp"

RotRK4Grad::RotRK4Grad(const mat33_t &J)
:J_(J), 
a1_(static_cast<double>(1.0)), 
a2_(static_cast<double>(2.0)),
a3_(static_cast<double>(2.0)),
a4_(static_cast<double>(1.0)),
dt_(0.01),
Eye_(Eye_.setIdentity())
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
    mat41_t q_temp;
    vector_t w_temp;

    // Compute q_m1 and w_m1
    q_temp = q_init_ + dt_/2.0*1/2.0*otimes(q_init_, w_init_);
    w_temp = w_init_ + dt_/2.0*J_.inverse()*(M_ - w_init_.cross(J_*w_init_) + theta_);

    mat43_t dq_temp;
    mat33_t dw_temp;

    // Compute dq_m1 and dw_m1
    dq_temp.setZero();
    dw_temp = dt_/2.0*J_.inverse();

    mat73_t DK2;
    DK2.block(0, 0, 3, 2) = 1/2.0 * otimes(q_temp, dw_temp);
    DK2.block(4, 0, 6, 2) = J_.inverse()*(-J_x_*dw_temp + Eye_);

    // Compute q_m2 and w_m2
    q_temp = q_init_ + dt_/2.0*1/2.0*otimes(q_temp, w_temp);
    w_temp = w_init_ + dt_/2.0*J_.inverse()*(M_ - w_temp.cross(J_*w_temp) + theta_);

    // Compute dq_m2 and dw_m2
    




    return mat73_t();
}
