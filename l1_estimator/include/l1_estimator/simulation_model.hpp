#ifndef SIMULATION_MODEL_HPP_
#define SIMULATION_MODEL_HPP_

#include "tools.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta4;

class SimulationModel{

    public:

        SimulationModel() = delete;

        SimulationModel(const QuadModel_t& quad_model,
        const aero_coeff_t& aero_coeff,
        const inertial_param_t& inertial_param,
        const double& arm_length,
        const double& time_step
        );

        void set_control_input(const mat41_t& rpm);

        void set_disturbance(const mat31_t& sigma_ext,
        const mat31_t& theta_ext);

        void get_state(mat31_t& p, 
        mat31_t& v,
        quat_t& q,
        mat31_t& w) const;
        
    private:

        runge_kutta4<state13_t> rk4_;

        inertial_param_t inertial_param_;
        aero_coeff_t aero_coeff_;

        void quadrotor_dynamics(const state13_t& dsdt,
        state13_t& s,
        const double& t);


};



#endif