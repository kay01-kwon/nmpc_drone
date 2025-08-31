#ifndef HGDO_H
#define HGDO_H

#include "utils/forward_dynamics.h"
#include "utils/rk4_ode_solver.h"

class HGDO
{
    public:

    HGDO();

    HGDO(const MavParam &param,
         const HgdoParams &hgdo_params);

    void setParam(const MavParam &param,
                  const HgdoParams &hgdo_params);

    void set_control_input(const rpmVector6 &rpm);

    void set_state(const Vec6 &state);

    void get_disturbance(Vec6 &disturbance) const;

    
    private:


    Vec3 gamma_tau_, gamma_f_;
    Vec6 disturbance_;

    Rk4OdeSolver<Vec3> *gamma_tau_ode_;

    Rk4OdeSolver<Vec3> *gamma_f_ode_;

    FDynamics *converter_;

    HgdoParams hgdo_params_;

    Mat3x3 J_, J_inv_;

};

#endif