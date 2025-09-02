#ifndef HGDO_H
#define HGDO_H

#include "utils/forward_dynamics.h"
#include "utils/rk4_ode_solver.h"
#include "model/hgdo_model.h"

struct HgdoParams
{   
    // Gains for hgdo
    double eps_tau{0.01};
    double eps_f{0.01};
};

class HGDO
{
    public:

    HGDO();

    HGDO(const MavParam &param,
         const HgdoParams &hgdo_params);

    void setParam(const MavParam &param,
                  const HgdoParams &hgdo_params);

    void set_control_input(const Vec6i16 &rpm);

    void set_state(const State &state);

    void get_disturbance(Vec6d &disturbance) const;

    
    private:

    FDynamics *converter_;

    Vec4d u_;

    HgdoModel* hgdo_model_;

    Vec6d disturbance_;

    Mat3x3 J_, J_inv_;

};

#endif