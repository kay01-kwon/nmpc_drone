#ifndef REF_MODEL_HPP_
#define REF_MODEL_HPP_
#include "tools.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta_dopri5;
using boost::numeric::odeint::integrate_adaptive;
using boost::numeric::odeint::integrate_const;
using boost::numeric::odeint::make_dense_output;
using boost::numeric::odeint::runge_kutta_cash_karp54;
using boost::numeric::odeint::make_controlled;
using boost::numeric::odeint::runge_kutta_fehlberg78;

class RefModel{

    public:

        RefModel() = delete;

        RefModel(const inertial_param_t& inertial_param,
        const double& k_p, const double& k_v,
        const double& k_q, const double& k_w);

        RefModel(const inertial_param_t& inertial_param);

        void initialize_state_variables();

        /**
         * Set inputs
         * u_comp = u - C(s)*sigma_hat
         * mu_comp = mu - C(s)*theta_hat
         * 
         * Set state
         * s: state from sensor fusion
         * 
         * Set disturbance
         * sigma_hat: translational disturbance (Noisy one)
         * theta_hat: attitude disturbance (Noisy one)
         * 
         * Set time
        */
        void set_input_state_disturbance_time(const mat31_t& u_comp,
        const mat31_t& mu_comp, 
        const mat31_t &p_state, const mat31_t &v_state,
        const quat_t &q_state, const mat31_t &w_state, 
        const mat31_t& sigma_hat, const mat31_t& theta_hat,
        const double& t);

        /**
         * Get position, velocity, quaternion 
         * and angular velocity, respectively.
        */
       void get_state_from_ref_model(mat31_t& p_ref, mat31_t& v_ref,
        quat_t& q_ref, mat31_t& w_ref) const;

        /**
         * Step integration of rungee kutta 4th order.
        */
        void prediction();

        ~RefModel() = default;

    private:

        // Nominal inertial parameter (mass and moment of inertia)
        double m_;
        mat33_t J_;

        // Control gain for translaional dynamics of reference model
        double k_p_, k_v_;

        // Control gain for attitude dynamics of reference model
        double k_q_, k_w_;

        // Control input for reference model
        mat31_t u_hat_, mu_hat_;

        // Gravity
        mat31_t grav_;

        // Estimated state from reference model
        // s(0) ~ s(2): px, py, pz
        // s(3) ~ s(5): vx, vy, vz
        // s(6) ~ s(9): qw, qx, qy ,qz
        // s(10) ~ s(12): 
        state13_t s_hat_;

        // To get the estimated state,
        // store data in the class object.
        mat31_t p_hat_, v_hat_, w_hat_;
        quat_t q_hat_;

        // Time
        double curr_time_, prev_time_, dt_;

        //runge kutta 45 class Declaration
        runge_kutta_dopri5<state13_t> rk45;

        void ref_dynamics(
            const state13_t& s,
            state13_t& dsdt,
            const double& t
        );

};

#endif