#include "rotational_rk4_gradient.hpp"

RotRK4Grad::RotRK4Grad(const mat33_t &J)
:J_(J), 
a1_(static_cast<double>(1.0)), 
a2_(static_cast<double>(2.0)),
a3_(static_cast<double>(2.0)),
a4_(static_cast<double>(1.0))
{
    double Jxx, Jyy, Jzz;
    Jxx = J_(0);
    Jyy = J_(4);
    Jzz = J_(8);
    J_x_ << (Jzz-Jyy), (Jxx-Jzz), (Jyy-Jxx);
}
