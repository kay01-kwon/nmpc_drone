#ifndef ROTATIONAL_DYNAMICS_HPP
#define ROTATIONAL_DYNAMICS_HPP
#include "tools/quaternion_math_tools.h"

class RotationalDynamics{

    public:

    RotationalDynamics() = delete;

    RotationalDynamics(const mat33_t &J);

    void setInput(const vector_t &M, const vector_t &theta);

    void do_simulation();

    quaternion_t get_quaternion() const;

    vector_t get_angular_velocity() const;

    private:

    rotational_state_t s_;

    double curr_time_, prev_time_, dt_;

};


#endif