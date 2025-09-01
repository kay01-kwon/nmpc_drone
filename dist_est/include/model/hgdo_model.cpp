#include "hgdo_model.h"
#include "utils/quaternion_utils.h"

HgdoModel::HgdoModel()
{
    J_ << 0.052, 0.0, 0.0,
          0.0, 0.052, 0.0,
          0.0, 0.0, 0.080;
    
    J_inv_.setZero();
    for(size_t i = 0; i < 3; i++)
        J_inv_(i,i) = 1.0 / J_(i,i);

    gamma_.setZero();

    w_.setZero();
    v_.setZero();
    u_.setZero();
    q_ = QuatType(1.0, 0.0, 0.0, 0.0);

    rk4_solver_ = new Rk4OdeSolver<Vec6>();
}

HgdoModel::HgdoModel(const MavParam& mav_param, 
                     const HgdoParams &hgdo_param)
{
    set_params(mav_param, hgdo_param);
    gamma_.setZero();
    w_.setZero();
    v_.setZero();
    u_.setZero();
}

void HgdoModel::set_params(const MavParam& mav_param, 
                          const HgdoParams &hgdo_param)
{
    m_ = mav_param.m;
    J_ = mav_param.J;

    J_inv_.setZero();

    for(size_t i = 0; i < 3; i++)
        J_inv_(i,i) = 1.0 / J_(i,i);

    eps_tau_ = hgdo_param.eps_tau;
    eps_f_ = hgdo_param.eps_f;
}

void HgdoModel::set_time(const double &t_prev,
                        const double &t_curr)
{
    t_prev_ = t_prev;
    t_curr_ = t_curr;
    dt_ = t_curr_ - t_prev_;
}

void HgdoModel::set_state_control(const Vec3& w,
                                 const Vec3& v,
                                 const controlInputVector4& u,
                                 const QuatType& q)
{
    w_ = w;
    v_ = v;
    u_ = u;
    q_ = q;
}

void HgdoModel::do_simulate_one_step()
{
    rk4_solver_->do_step(
        [this](const Vec6 &gamma, Vec6 &gamma_dot, const double &t_prev)
            {
                this->compute_dynamics(gamma, gamma_dot, t_prev);
            },
        gamma_, t_prev_, dt_);
}

void HgdoModel::get_disturbance(Vec6 &d) const
{
    d.head<3>() = J_ * (gamma_.head<3>() + 1.0/eps_tau_*w_);
    d.tail<3>() = m_ * (gamma_.tail<3>() + 1.0/eps_f_*v_);

}

HgdoModel::~HgdoModel()
{
    if(rk4_solver_ != nullptr)
        delete rk4_solver_;
}

void HgdoModel::compute_dynamics(const Vec6& gamma,
                                Vec6& gamma_dot,
                                const double &t_prev)
{
    gamma_dot.head<3>() = -1.0/eps_tau_*
    (gamma_.head<3>() + 1.0/eps_tau_*w_)
    +1.0/eps_tau_*(-u_.tail<3>() + J_inv_*w_.cross(J_*w_));

    Vec3 u_T(0.0, 0.0, u_(0)/m_);

    Mat3x3 R_wb = quaternion_to_rotm(q_);

    u_T = R_wb * u_T;
    

    gamma_dot.tail<3>() = -1.0/eps_f_*
    (gamma_.tail<3>() + 1.0/eps_f_*v_)
    + 1.0/eps_f_*(-u_T -g_);

}