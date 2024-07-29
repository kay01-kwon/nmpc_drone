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

quaternion_t RotationalSimulation::get_quaternion() const
{
    quaternion_t q;
    q.w() = s_(0);
    q.x() = s_(1);
    q.y() = s_(2);
    q.z() = s_(3);
    
    return q;
}

vector_t RotationalSimulation::get_angular_velocity() const
{
    vector_t w;
    w(0) = s_(4);
    w(1) = s_(5);
    w(2) = s_(6);
    
    return w;
}

double RotationalSimulation::get_time() const
{
    return curr_time_;
}

void RotationalSimulation::rotational_dynamics(rotational_state_t &dsdt, 
const rotational_state_t &s, 
const double time, 
const vector_t &M, const vector_t theta)
{
    quaternion_t q, w_quaternion_form, q_temp, dqdt;
    vector_t w, dwdt;

    q.w() = s(0);
    q.x() = s(1);
    q.y() = s(2);
    q.z() = s(3);

    q.normalize();
    
    w(0) = s(4);
    w(1) = s(5);
    w(2) = s(6);

    w_quaternion_form.w() = 0;
    w_quaternion_form.x() = w(0);
    w_quaternion_form.y() = w(1);
    w_quaternion_form.z() = w(2);
    
    q_temp = otimes(q, w_quaternion_form);

    dqdt.w() = 0.5*q_temp.w();
    dqdt.x() = 0.5*q_temp.x();
    dqdt.y() = 0.5*q_temp.y();
    dqdt.z() = 0.5*q_temp.z();

    dwdt = J_.inverse()  * (M - w.cross(J_*w) + theta);

}
