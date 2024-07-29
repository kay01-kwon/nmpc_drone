#include "rotational_simulation_class.hpp"

RotationalSimulation::RotationalSimulation(const mat33_t &J, double dt = 0.01)
:J_(J), M_(M_.setZero()), theta_(theta_.setZero()),
s_(s_.setZero()), curr_time_(0), prev_time_(0), dt_(dt)
{
    s_(0) = 1.0;
}

void RotationalSimulation::setInput(const vector_t &M, const vector_t &theta)
{
    M_ = M;
    theta_ = theta;
}

void RotationalSimulation::do_simulation()
{

}

quaternion_t *RotationalSimulation::get_quaternion() const
{
    return nullptr;
}

vector_t *RotationalSimulation::get_angular_velocity() const
{
    return nullptr;
}

double *RotationalSimulation::get_time() const
{
    return nullptr;
}

void RotationalSimulation::rotational_dynamics(rotational_state_t &dsdt, 
const rotational_state_t &s, 
const double time, 
const vector_t &M, const vector_t theta)
{

}
