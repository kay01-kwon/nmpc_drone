#ifndef FORWARD_DYNAMICS_H
#define FORWARD_DYNAMICS_H
#include "state_def.h"

typedef Eigen::Matrix<double, 4, 6> Mat4x6;

class FDynamics{
    
    public:

    FDynamics();

    FDynamics(const MavParam& param);

    void setParam(const MavParam& param);

    void convert_rpm_to_control_input(const Vec6i16 &rpm, 
    Vec4d &control_input);

    private:

    double C_T_{1.465e-07};  // thrust coefficient
    Mat4x6 K_f_;    // rotor thrust to force and moment mapping matrix

    void set_forward_mapping(double l, double k_m = 0.01569);
};


#endif