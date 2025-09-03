#ifndef HGDO_H
#define HGDO_H

#include "model/hgdo_model.h"

class HGDO
{
    public:

    HGDO();

    HGDO(const MavParam &param,
         const HgdoParam &hgdo_param);

    void setParam(const MavParam &param,
                  const HgdoParam &hgdo_param);

    void updateStateControlTime(const State &s,
                               const Vec4d &u,
                               const double &t_prev,
                               const double &t_curr);

    void getDisturbance(Vec6d &f_tau) const;

    
    private:

    Vec4d u_;

    HgdoModel *hgdo_model_;

    Vec6d force_tau_disturbance_;

    Mat3x3 J_, J_inv_;

};

#endif