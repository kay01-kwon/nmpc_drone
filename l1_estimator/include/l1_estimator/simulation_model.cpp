#include "simulation_model.hpp"

SimulationModel::SimulationModel(const QuadModel &quad_model, 
const aero_coeff_t &aero_coeff, 
const inertial_param_t &inertial_param,
const double &arm_length)
:quad_model_(quad_model),
aero_coeff_(aero_coeff),
inertial_param_(inertial_param),
l_(arm_length),
dt_(0)
{
    assert(dt_ <= 0);
}

void SimulationModel::set_control_input(const mat41_t &rpm)
{
}

void SimulationModel::set_disturbance(const mat31_t &sigma_ext, 
const mat31_t &theta_ext)
{
}

void SimulationModel::quadrotor_dynamics(const state13_t &dsdt, 
state13_t &s, 
const double &t)
{
}
