#ifndef HGDO_MODEL_H
#define HGDO_MODEL_H

#include "utils/state_def.h"
#include "utils/rk4_ode_solver.h"
struct HgdoParam
{   
    // Gains for hgdo
    double eps_tau{0.01};
    double eps_f{0.01};
};

class HgdoModel{

    public:

    HgdoModel();

    HgdoModel(const MavParam& mav_param, 
             const HgdoParam &hgdo_param);

    void setParams(const MavParam& mav_param, 
                    const HgdoParam &hgdo_param);

    void updateStateControl(const Vec3d& v,
                           const Vec3d& w,
                           const Vec4d& u,
                           const Quatd& q);

    void do_simulate_one_step(const double &t_prev,
                              const double &t_curr);

    void getDisturbance(Vec6d & d) const;

    ~HgdoModel();

    private:

    double m_{2.9};
    Mat3x3 J_, J_inv_;

    Vec6d gamma_;
    Vec3d w_, v_;
    Vec4d u_;
    Quatd q_;
    double eps_tau_, eps_f_;

    Rk4OdeSolver<Vec6d> *rk4_solver_;

    double t_prev_{0.0}, t_curr_{0.0}, dt_{0.01};


    void compute_dynamics(const Vec6d& gamma,
                          Vec6d & gamma_dot,
                          const double &t_prev);

};

#endif