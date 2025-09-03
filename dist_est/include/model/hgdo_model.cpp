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

    v_.setZero();
    w_.setZero();
    u_.setZero();
    q_ = Quatd(1.0, 0.0, 0.0, 0.0);

    rk4_solver_ = new Rk4OdeSolver<Vec6d>();
}

HgdoModel::HgdoModel(const MavParam& mav_param, 
                     const HgdoParam &hgdo_param)
{
    setParams(mav_param, hgdo_param);
    gamma_.setZero();
    v_.setZero();
    w_.setZero();
    u_.setZero();
}

void HgdoModel::setParams(const MavParam &mav_param, 
                          const HgdoParam &hgdo_param)
{
    m_ = mav_param.m;
    J_ = mav_param.J;

    J_inv_.setZero();

    for(size_t i = 0; i < 3; i++)
        J_inv_(i,i) = 1.0 / J_(i,i);

    eps_tau_ = hgdo_param.eps_tau;
    eps_f_ = hgdo_param.eps_f;
}

void HgdoModel::updateStateControl(const Vec3d& v,
                                 const Vec3d& w,
                                 const Vec4d& u,
                                 const Quatd& q)
{
    v_ = v;
    w_ = w;
    u_ = u;
    q_ = q;
}

void HgdoModel::do_simulate_one_step(const double &t_prev,
                                    const double &t_curr)
{
    double dt = t_curr - t_prev;
    rk4_solver_->do_step(
        [this](const Vec6d &gamma, Vec6d &gamma_dot, const double &t_prev)
            {
                this->compute_dynamics(gamma, gamma_dot, t_prev);
            },
        gamma_, t_prev, dt);
}

void HgdoModel::getDisturbance(Vec6d &d) const
{
    d.head<3>() = m_ * (gamma_.tail<3>() + 1.0/eps_f_*v_);
    d.tail<3>() = J_ * (gamma_.head<3>() + 1.0/eps_tau_*w_);
}

HgdoModel::~HgdoModel()
{
    if(rk4_solver_ != nullptr)
        delete rk4_solver_;
}

void HgdoModel::compute_dynamics(const Vec6d& gamma,
                                Vec6d& gamma_dot,
                                const double &t_prev)
{

    Vec3d u_T(0.0, 0.0, u_(0)/m_);

    Mat3x3 R_wb = quaternion_to_rotm(q_);

    u_T = R_wb * u_T;
    

    gamma_dot.head<3>() = -1.0/eps_f_*
    (gamma_.head<3>() + 1.0/eps_f_*v_)
    + 1.0/eps_f_*(-u_T -g);

    gamma_dot.tail<3>() = -1.0/eps_tau_*
    (gamma_.tail<3>() + 1.0/eps_tau_*w_)
    +1.0/eps_tau_*(-u_.tail<3>() + J_inv_*w_.cross(J_*w_));

}