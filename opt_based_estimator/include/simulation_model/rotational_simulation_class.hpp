#ifndef ROTATIONAL_SIMULATION_CLASS_HPP
#define ROTATIONAL_SIMULAITON_CLASS_HPP
#include "tools/quaternion_math_tools.h"

class RotationalSimulation{

    public:

    RotationalSimulation() = delete;

    RotationalSimulation(const mat33_t &J, double dt = 0.01);

    void setInput(const vector_t &M, const vector_t &theta);

    void do_simulation();

    quaternion_t get_quaternion() const;

    vector_t get_angular_velocity() const;

    double get_time() const;

    private:

    rotational_state_t s_;

    mat33_t J_;

    vector_t M_, theta_;

    double curr_time_, prev_time_, dt_;

    void rotational_dynamics(rotational_state_t& dsdt,
    const rotational_state_t& s, const double time, 
    const vector_t& M, const vector_t theta);

};


#endif