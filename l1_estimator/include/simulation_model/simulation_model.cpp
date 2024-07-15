#include "simulation_model.hpp"

/**
 * @brief Construct a new Simulation Model:: Simulation Model object
 * 
 * @param quad_model model1: '+', model2: 'x'
 * @param aero_coeff lift_coeff, moment_coeff
 * @param inertial_param mass, moment of inertia, B_p_CG_COM
 * @param arm_length arm length
 */
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
    // Initialize state variables
    s_.setZero();
    s_(6) = 1.0;

    // Put gravity
    gravity_ << 0, 0, -9.81;

        /**
     * QuadModel::model1
     * 
     *               y
     *               ^
     *               |
     *               
     *               1 (CCW)
     *               |
     *               |
     *               |
     * 2 (CW) --------------- 0 (CW)    ----------> x
     *               |
     *               |
     *               |
     *               3 (CCW)
     * 
     * 
     * QuadModel::model2
     * 
     *            y
     *            ^
     *            |
     *            |
     *    2 (CW)          1 (CCW)
     *      x           x
     *        x       x
     *          x   x
     *            x                    ----------> x
     *          x   x
     *        x       x
     *      x           x
     *    3 (CCW)         0 (CW)
    */

    if(quad_model == QuadModel::model1)
    {
        CG_p_CG_rotors[0] << l_, 0, 0;
        CG_p_CG_rotors[1] << 0, l_, 0;
        CG_p_CG_rotors[2] << -l_, 0, 0;
        CG_p_CG_rotors[3] << 0, -l_, 0;
    }
    else
    {
        CG_p_CG_rotors[0] << -l_*sqrt(2)/2.0, -l_*sqrt(2)/2.0, 0;
        CG_p_CG_rotors[1] << l_*sqrt(2)/2.0, l_*sqrt(2)/2.0, 0;
        CG_p_CG_rotors[2] << -l_*sqrt(2)/2.0, l_*sqrt(2)/2.0, 0;
        CG_p_CG_rotors[3] << -l_*sqrt(2)/2.0, -l_*sqrt(2)/2.0, 0;
    }
    
    assert((quad_model == QuadModel::model1) 
    || (quad_model == QuadModel::model2));
    
    assert(B_p_CG_COM_.size() == 3);
    assert(inertial_param.J.size() == 9);

}

/**
 * @brief Set four rpm to control quadrotor
 * 
 * @param rpm 
 */
void SimulationModel::set_control_input(const mat41_t &rpm)
{
    mat41_t thrust;

    thrust << pow(rpm(0), 2.0), 
    pow(rpm(1), 2.0),
    pow(rpm(2), 2.0),
    pow(rpm(3), 2.0);

    thrust = lift_coeff_*thrust;

    assert( thrust.size() == 4);

    convert_thrust_to_wrench(B_p_CG_COM_,
    CG_p_CG_rotors,
    thrust,
    moment_coeff_,
    force_,
    moment_);

}

/**
 * @brief Set translational and rotational disturbance 
 * to the quadrotor in order.
 * @param sigma_ext translational disturbance
 * @param theta_ext rotational disturbance
 */
void SimulationModel::set_disturbance(const mat31_t &sigma_ext, 
const mat31_t &theta_ext)
{
    sigma_ext_ = sigma_ext;
    theta_ext_ = theta_ext;

    assert(sigma_ext.size() == 3);
    assert(theta_ext.size() == 3);

}

/**
 * @brief Set the simulation time
 * 
 * @param time 
 */
void SimulationModel::set_time(const double &time)
{
    curr_time_ = time;
}

/**
 * @brief Get the state of the simulation
 * 
 * @param p position
 * @param v velocity
 * @param q quaternion
 * @param w angular velocity
 */
void SimulationModel::get_state(mat31_t &p, 
mat31_t &v, 
quat_t &q, 
mat31_t &w) const
{
    assert(p.size() == 3);
    assert(v.size() == 3);
    assert(w.size() == 3);
    
    for(size_t i = 0; i < 3; i++)
    {
        p(i) = s_(i);
        v(i) = s_(i+3);
        w(i) = s_(i+10);
    }
    
    q.w() = s_(6);
    q.x() = s_(7);
    q.y() = s_(8);
    q.z() = s_(9);

}

/**
 * @brief Get the current time of the simulation
 * 
 * @param time 
 */
void SimulationModel::get_time(double &time) const
{
    time = curr_time_;
}

/**
 * @brief solve the simulation model
 * 
 */
void SimulationModel::integrate()
{
    dt_ = curr_time_ - prev_time_;
    rk4_.do_step(
        [this]
        (const state13_t& s, state13_t& dsdt, const double& t)
        {
            this->SimulationModel::quadrotor_dynamics(s, dsdt, t);
        }
        ,s_, prev_time_, dt_
    );

    prev_time_ = curr_time_;

}

/**
 * @brief quadrotor dynamics
 * 
 * @param s p, v, q, w
 * @param dsdt dpdt, dvdt, dqdt, dwdt
 * @param t time
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

    assert(q.w()*q.w() 
    + q.x()*q.x()
    + q.y()*q.y()
    + q.z()*q.z() > 0);

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
        // Put the rate of position
        dsdt(i) = dpdt(i);

        // Put the rate of velocity
        dsdt(i+3) = dvdt(i);
    }

    // Put the rate of quaternion
    dsdt(6) = dqdt.w();
    dsdt(7) = dqdt.x();
    dsdt(8) = dqdt.y();
    dsdt(9) = dqdt.z();

    // Put the rate of angular velocity
    for(int i = 0; i < 3; i++)
    {
        dsdt(i+10) = dwdt(i);
    }

}