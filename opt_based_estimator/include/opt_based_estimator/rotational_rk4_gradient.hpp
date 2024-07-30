#ifndef ROTATIONAL_RK4_GRADIENT_HPP
#define ROTATIONAL_RK4_GRADIENT_HPP
#include "utils/quaternion_math_tools.h"

class RotRK4Grad{

    public:

    RotRK4Grad() = delete;

    RotRK4Grad(const mat33_t& J);

    void set_time(const double& curr_time);

    void set_meas_state(const quaternion_t &q_init, const vector_t& w_init);

    void set_input_disturbance(const vector_t &M_init, const vector_t &theta_init);

    mat73_t getRK4Grad() const;

    private:

    double a1_, a2_, a3_, a4_;

    mat33_t J_, J_x_;
    mat41_t q_m1_, q_m2_, q_f_;
    vector_t w_m1_, w_m2_, w_f_;
    mat73_t DK1, DK2, DK3, DK4;
    

};


#endif