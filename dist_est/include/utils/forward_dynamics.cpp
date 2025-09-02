#include "forward_dynamics.h"

FDynamics::FDynamics()
{
    setParam(MavParam());
}

FDynamics::FDynamics(const MavParam& param)
{
    setParam(param);
}

void FDynamics::setParam(const MavParam& param)
{
    double l_ = param.l;  // arm length in meters
    C_T_ = param.C_T;
    double k_m_ = param.k_m;
    set_forward_mapping(l_, k_m_);
}

void FDynamics::convert_rpm_to_control_input(const Vec6i16& rpm, 
                                        Vec4d &control_input)
{
    Vec6d T;
    double rpm2 = 0;
    for (size_t i = 0; i < 6; i++)
    {
        rpm2 = double(rpm[i]*rpm[i]);
        T[i] = C_T_ * rpm2;  // thrust from each rotor
    }

    control_input = K_f_ * T;  // convert rotors' thrust to control input

}

void FDynamics::set_forward_mapping(double l, double k_m)
{
    double cos_pi_3 = cos(M_PI / 3.0);
    double sin_pi_3 = sin(M_PI / 3.0);

    double ly1, ly2, ly3, ly4, ly5, ly6;
    double lx1, lx2, lx3, lx4, lx5, lx6;

    ly1 = l * cos_pi_3;
    ly2 = l;
    ly3 = l * cos_pi_3;

    ly4 = -l * cos_pi_3;
    ly5 = -l;
    ly6 = -l * cos_pi_3;

    lx1 = l * sin_pi_3;
    lx2 = 0.0;
    lx3 = -l * sin_pi_3;
    
    lx4 = -l * sin_pi_3;
    lx5 = 0.0;
    lx6 = l * sin_pi_3;
    

    K_f_ << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            ly1, ly2, ly3, ly4, ly5, ly6,
            -lx1, -lx2, -lx3, -lx4, -lx5, -lx6,
            -k_m, k_m, -k_m, k_m, -k_m, k_m;


}