#include "rotational_disturbance_estimator.hpp"

RotDistEst::RotDistEst(const mat33_t &J, const mat77_t &Q, 
const double &term_error, const uint8_t &iter_max)
:J_nom_(J), Q_(Q), term_error_(term_error), iter_max_(iter_max),
rot_rk4_grad_obj_(J_nom_), curr_time_(0), prev_time_(0), dt_(0)
{
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
