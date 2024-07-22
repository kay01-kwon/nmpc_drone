#ifndef SIMULATION_MODEL_HPP_
#define SIMULATION_MODEL_HPP_

#include "l1_estimator/tools.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta_dopri5;
using boost::numeric::odeint::integrate_adaptive;
using boost::numeric::odeint::integrate_const;
using boost::numeric::odeint::make_dense_output;

class SimulationModel{

    public:

        SimulationModel() = delete;

        SimulationModel(const QuadModel& quad_model,
        const aero_coeff_t& aero_coeff,
        const inertial_param_t& inertial_param,
        const double& arm_length
        );

        void set_control_input(const mat41_t& rpm);

        void set_disturbance(const mat31_t& sigma_ext,
        const mat31_t& theta_ext);

        void set_time(const double& time);

        void get_state(mat31_t& p, 
        mat31_t& v,
        quat_t& q,
        mat31_t& w) const;

        void get_time(double& time) const;

        void integrate();

        ~SimulationModel() = default;
        
    private:

        runge_kutta_dopri5<state13_t> rk4_;

        QuadModel quad_model_;
        double lift_coeff_, moment_coeff_;
        double l_;

        double curr_time_, prev_time_;
        double dt_;

        mat31_t force_, sigma_ext_;
        mat31_t moment_, theta_ext_;

        double m_;
        mat33_t J_;
        mat31_t B_p_CG_COM_;
        mat31_t CG_p_CG_rotors[4];

        mat31_t gravity_;

        state13_t s_;

        void quadrotor_dynamics(const state13_t& s,
        state13_t& dsdt,
        const double& t);


};



#endif