#ifndef HGDO_H
#define HGDO_H

#include "utils/forward_dynamics.h"
#include "utils/rk4_ode_solver.h"
#include "model/hgdo_model.h"

class HGDO
{
    public:

    HGDO();

    HGDO(const MavParam &param,
         const HgdoParams &hgdo_params);

    void setParam(const MavParam &param,
                  const HgdoParams &hgdo_params);

    void set_control_input(const rpmVector6 &rpm);

    void set_state(const State &state);

    void get_disturbance(Vec6 &disturbance) const;

    
    private:

    FDynamics *converter_;

    Vec4 u_;

    HgdoModel* hgdo_model_;

    Vec6 disturbance_;

    Mat3x3 J_, J_inv_;

};

#endif