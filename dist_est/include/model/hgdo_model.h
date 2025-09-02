#ifndef HGDO_MODEL_H
#define HGDO_MODEL_H

#include "utils/state_def.h"
#include "utils/rk4_ode_solver.h"

class HgdoModel{

    public:

    HgdoModel();

    HgdoModel(const MavParam& mav_param, 
             const HgdoParams &hgdo_param);

    /**
     * @brief Set the Dynamic parameters and high gain observer parameters
     * 
     * @param param MAV dynamic parameters
     * @param hgdo_param High gain observer parameters
     */
    void set_params(const MavParam& mav_param, 
                    const HgdoParams &hgdo_param);

    /**
     * @brief Set the time object
     * 
     * @param t_prev Previous time
     * @param t_curr Current time
     */

    void set_time(const double &t_prev,
                  const double &t_curr);

    /**
     * @brief Set the state and control inputs of the nominal model
     * 
     * @param w angular velocity expressed in body frame
     * @param v linear velocity expressed in world frame
     * @param u force and moments applied to the vehicle
     * @param q quaternion representing the orientation of the vehicle
     */
    void set_state_control(const Vec3& w,
                           const Vec3& v,
                           const controlInputVector4& u,
                           const QuatType& q);

    void do_simulate_one_step();

    void get_disturbance(Vec6 & d) const;

    ~HgdoModel();

    private:

    double m_{2.9};
    Mat3x3 J_, J_inv_;

    Vec6 gamma_;
    Vec3 w_, v_;
    controlInputVector4 u_;
    QuatType q_;
    double eps_tau_, eps_f_;

    Rk4OdeSolver<Vec6> *rk4_solver_;

    double t_prev_{0.0}, t_curr_{0.0}, dt_{0.01};


    Vec3 g_{0.0, 0.0, -9.81};  // gravity vector


    void compute_dynamics(const Vec6& gamma,
                          Vec6 & gamma_dot,
                          const double &t_prev);

};

#endif