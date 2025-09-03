#include "dist_est/hgdo_dist_est.h"
#include "utils/state_demuxer.h"

HGDO::HGDO()
:hgdo_model_(nullptr)
{
    force_tau_disturbance_.setZero();
    u_.setZero();

}

HGDO::HGDO(const MavParam &mav_param,
             const HgdoParam &hgdo_param)
{

    force_tau_disturbance_.setZero();
    u_.setZero();
    setParam(mav_param, hgdo_param);

    hgdo_model_ = new HgdoModel(mav_param, hgdo_param);
}

void HGDO::setParam(const MavParam &mav_param,
                     const HgdoParam &hgdo_params)
{

    if(hgdo_model_ != nullptr)
        delete hgdo_model_;
    hgdo_model_ = new HgdoModel(mav_param, hgdo_params);

    J_ = mav_param.J;
    J_inv_.setZero();

    for(size_t i = 0; i < 3; i++)
            J_inv_(i,i) = J_(i,i);
}

void HGDO::updateStateControlTime(const State &s,
                                  const Vec4d &u,
                                  const double &t_prev,
                                  const double &t_curr)
{

    assert(hgdo_model_ != nullptr);

    Vec3d p, v;
    Quatd q;
    Vec3d w;

    demux_state(s, p, v, q, w);

    hgdo_model_->updateStateControl(v, w, u, q);
    hgdo_model_->do_simulate_one_step(t_prev, t_curr);
    hgdo_model_->getDisturbance(force_tau_disturbance_);
}

void HGDO::getDisturbance(Vec6d &f_tau) const
{
    f_tau = force_tau_disturbance_;
}