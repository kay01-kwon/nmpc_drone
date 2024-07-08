#include "simulation_model.hpp"

SimulationModel::SimulationModel(const QuadModel &quad_model, 
const aero_coeff_t &aero_coeff, 
const inertial_param_t &inertial_param,
const double &arm_length)
:quad_model_(quad_model),
lift_coeff_(aero_coeff.lift_coeff),
moment_coeff_(aero_coeff.moment_coeff),
m_(inertial_param.m),
J_(inertial_param.J),
B_p_CG_COM_(inertial_param.r_offset),
l_(arm_length),
dt_(0)
{
    assert(dt_ <= 0);
}

void SimulationModel::set_control_input(const mat41_t &rpm)
{
    mat41_t thrust;

    thrust << pow(rpm(0), 2.0), 
    pow(rpm(1), 2.0),
    pow(rpm(2), 2.0),
    pow(rpm(3), 2.0);

    thrust *= lift_coeff_*thrust;

    convert_thrust_to_wrench(quad_model_,
    l_,
    B_p_CG_COM_,
    thrust,
    moment_coeff_,
    force_,
    moment_);

}

void SimulationModel::set_disturbance(const mat31_t &sigma_ext, 
const mat31_t &theta_ext)
{
    sigma_ext_ = sigma_ext;
    theta_ext_ = theta_ext;
}

void SimulationModel::set_time(const double &time)
{
    curr_time_ = time;
}

void SimulationModel::get_state(mat31_t &p, 
mat31_t &v, 
quat_t &q, 
mat31_t &w) const
{
    for(size_t i = 0; i < 3; i++)
    {
        p(i) = s_(i);
        v(i) = s_(i+3);
        w(i) = s_(i+10);
    }
    
    q.z() = s_(6);
    q.x() = s_(7);
    q.y() = s_(8);
    q.z() = s_(9);

}

void SimulationModel::get_time(double &time) const
{
    time = curr_time_;
}

void SimulationModel::solve()
{
}

void SimulationModel::quadrotor_dynamics(const state13_t &dsdt, 
state13_t &s, 
const double &t)
{
}