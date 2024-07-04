#ifndef REF_MODEL_HPP_
#define REF_MODEL_HPP_
#include "tools.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta4;

class RefModel{

    public:

        RefModel() = delete;

        RefModel(const Inertial_param_t& inertial_param);

        /**
         * Set inputs
         * u_comp = u - C(s)*sigma_hat
         * mu_comp = mu - C(s)*theta_hat  
        */
        void set_input(const mat31_t& u_comp, const mat31_t mu_comp);

        void set_state(const mat31_t& p_state, const mat31_t& v_state,
        const quat_t& q_state, const mat31_t w_state);

        void set_est_disturbance(const mat31_t& sigma_est, 
        const mat31_t theta_est);

        void set_time(const double& t);

        /**
         * Get position, velocity, quaternion 
         * and angular velocity, respectively.
        */

       void get_state_from_ref_model(const mat31_t& p_ref, const mat31_t& v_ref,
       const quat_t& q_ref, const mat31_t& w_ref);

        /**
         * Step integration of rungee kutta 4th order.
        */
        void solve();

    private:

        Inertial_param_t inertial_param_;



};


#endif