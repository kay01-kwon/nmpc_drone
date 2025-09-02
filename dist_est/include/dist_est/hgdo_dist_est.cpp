#include "dist_est/hgdo_dist_est.h"

HGDO::HGDO()
: converter_(nullptr),
hgdo_model_(nullptr)
{
    disturbance_.setZero();
    u_.setZero();
}

HGDO::HGDO(const MavParam &mav_param,
             const HgdoParams &hgdo_params)
{
    disturbance_.setZero();
    u_.setZero();
    setParam(mav_param, hgdo_params);
}

void HGDO::setParam(const MavParam &mav_param,
                     const HgdoParams &hgdo_params)
{
    if(converter_ != nullptr)
        delete converter_;
    converter_ = new FDynamics(mav_param);

    if(hgdo_model_ != nullptr)
        delete hgdo_model_;
    hgdo_model_ = new HgdoModel(mav_param, hgdo_params);

    J_ = mav_param.J;
    J_inv_.setZero();

    for(size_t i = 0; i < 3; i++)
            J_inv_(i,i) = J_(i,i);
}

void HGDO::set_control_input(const Vec6i16 &rpm)
{
    converter_->convert_rpm_to_control_input(rpm, u_);
}

void HGDO::set_state(const State &state)
{
    
}