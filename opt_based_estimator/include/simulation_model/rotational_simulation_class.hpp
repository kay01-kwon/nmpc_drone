#ifndef ROTATIONAL_SIMULATION_CLASS_HPP
#define ROTATIONAL_SIMULAITON_CLASS_HPP
#include "tools/quaternion_math_tools.h"
#include "ode_solver/ode_solver_include.h"
class RotationalSimulation{

    public:

    RotationalSimulation() = delete;

    /**
     * @brief Construct a new Rotational Simulation object
     * 
     * @param J Moment of inertia
     * @param dt time step to play simulation
     */
    RotationalSimulation(const mat33_t &J, double dt = 0.01);

    /**
     * @brief Set the Input object
     * 
     * @param M moment input for the quadrotor model
     * @param theta rotatiaonal disturbance
     */
    void setInput(const vector_t &M, const vector_t &theta);

    /**
     * @brief Integrate the ode function for the rotataional kinematics
     * and dynamics, respectively.
     * 
     */
    void do_simulation();

    /**
     * @brief Get the quaternion object
     * 
     * @return quaternion_t return actual quaternion value
     */
    quaternion_t get_quaternion() const;

    /**
     * @brief Get the angular velocity object
     * 
     * @return vector_t return actual angular velocity expressed in the body frame
     */
    vector_t get_angular_velocity() const;

    /**
     * @brief Get the time object
     * 
     * @return double return current time
     */
    double get_time() const;

    private:

    runge_kutta4<rotational_state_t> rk4;

    rotational_state_t s_;

    mat33_t J_;

    vector_t M_, theta_;

    double curr_time_, prev_time_, dt_;

    void rotational_dynamics(const rotational_state_t& s,
    rotational_state_t& dsdt, const double &time, 
    const vector_t& M, const vector_t theta);

};


#endif