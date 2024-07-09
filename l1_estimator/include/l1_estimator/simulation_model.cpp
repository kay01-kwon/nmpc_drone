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
dt_(0),
force_(force_.setZero()),
moment_(moment_.setZero())
{
    s_.setZero();
    s_(6) = 1;
    gravity_ << 0, 0, -9.81;

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

/**
 * @brief quadrotor dynamics
 * 
 * @param s : p, v, q, w
 * @param dsdt: dpdt, dvdt, dqdt, dwdt
 * @param t : time
 */
void SimulationModel::quadrotor_dynamics(const state13_t &s, 
state13_t &dsdt, 
const double &t)
{
    mat31_t p,v,dpdt,dvdt;
    quat_t q, q_unit, dqdt;
    mat31_t w, dwdt;
    mat33_t R, w_skew;

    // Get current position and velocity
    for(int i = 0; i < 3; i++)
    {
        p(i) = s(i);
        v(i) = s(i+3);
    }

    // Get current quaternion
    q.w() = s(6);
    q.x() = s(7);
    q.y() = s(8);
    q.z() = s(9);

    // Get current angular velocity
    for(int i = 0; i < 3; i++)
    {
        w(i) = s(i+10);
    }

    // Get unit quaternion and then convert it to rotation matrix 
    convert_quat_to_unit_quat(q, q_unit);
    get_rotm_from_quat(q_unit, R);

    // Translational kinematics
    dpdt = v;

    // Translational dynamics
    dvdt = R*force_/m_ + gravity_;

    // Attitude kinematics
    get_dqdt(q_unit, w, dqdt);

    // Attitude dynamics
    convert_vec_to_skew(w, w_skew);
    dwdt = J_.inverse()*(moment_ - w_skew*(J_*w));

    // Put the rate of state
    for(int i = 0; i < 3; i++)
    {
        dsdt(i) = dpdt(i);
        dsdt(i+3) = dvdt(i);
    }

    dsdt(6) = dqdt.w();
    dsdt(7) = dqdt.x();
    dsdt(8) = dqdt.y();
    dsdt(9) = dqdt.z();

    for(int i = 0; i < 3; i++)
    {
        dsdt(i+10) = dwdt(i);
    }



}