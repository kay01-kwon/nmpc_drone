#ifndef EKF_DIST_EST_H
#define EKF_DIST_EST_H
#include "utils/forward_dynamics.h"

class EkfDistEst
{
    public:

    EkfDistEst();

    EkfDistEst(const MavParam& param, const EKFParams &ekf_params);

    void setParam(const MavParam& param, const EKFParams &ekf_params);

    AugStateVector10 predict(const rpmVector6 &rpm, const double &time);

    AugStateVector10 meas_update(const StateVector7 &s_meas);

    controlInputVector4 getControlInput(const rpmVector6 &rpm) const
    {
        controlInputVector4 u;
        converter_->convert_rpm_to_control_input(rpm, u);
        return u;
    }

    ~EkfDistEst();

    private:

    // Prior and posterior :quaternion, angular velocity, disturbance
    AugStateVector10 s_pred_, s_est_;

    Mat10x10 P_pred_, P_est_;

    FDynamics *converter_;

    EKFParams ekf_params_;

    Mat3x3 J_, J_inv_;

};


#endif