#ifndef ROTATIONAL_DISTURBANCE_ESTIMATOR_HPP
#define ROTATIONAL_DISTURBANCE_ESTIMATOR_HPP
#include "rotational_rk4_gradient.hpp"

class RotDistEst{

    public:

    RotDistEst() = delete;

    RotDistEst(const mat33_t& J, const mat77_t &Q,
    const double &term_error = 1E-3, const uint8_t &iter_max = 30);

    private:

    mat33_t J_nom_;
    mat77_t Q_;
    double term_error_;
    uint8_t iter_max_;

    RotRK4Grad rot_rk4_grad;

    vector_t theta_k_;

};

#endif