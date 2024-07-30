#ifndef ROTATIONAL_RK4_GRADIENT_HPP
#define ROTATIONAL_RK4_GRADIENT_HPP
#include "utils/quaternion_math_tools.h"

class RotRK4Grad{

    public:

    RotRK4Grad() = delete;

    RotRK4Grad(const mat33_t& J);

    void set_time_difference(const double& dt);

    void set_initial_state(const quaternion_t &q_init, const vector_t& w_init);

    void set_initial_state(const rotational_state_t &s_init);

    void set_input_disturbance(const vector_t &M_init, const vector_t &theta_init);

    mat73_t getRK4Grad() const;

    private:

    double a1_, a2_, a3_, a4_;

    mat33_t J_, J_x_, Eye_;
    
    vector_t M_, theta_;

    mat41_t q_init_;
    vector_t w_init_;

    mat73_t DK1_;
    
    double dt_;

};


#endif