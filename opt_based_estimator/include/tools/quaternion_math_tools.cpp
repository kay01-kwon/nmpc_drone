#include "quaternion_math_tools.h"

mat41_t otimes(const mat41_t &q_left, const mat41_t &q_right)
{
    mat44_t Lambda;
    mat41_t qr;

    double ql_w, ql_x, ql_y, ql_z;

    ql_w = q_left(0);
    ql_x = q_left(1);
    ql_y = q_left(2);
    ql_z = q_left(3);

    Lambda <<   ql_w, -ql_x, -ql_y, -ql_z,
                ql_x, ql_w, -ql_z, ql_y,
                ql_y, ql_z, ql_w, -ql_x,
                ql_z, -ql_y, ql_x, ql_w;
    

    qr << q_right(0), q_right(1), q_right(2), q_right(3);

    return Lambda*qr;
}

mat34_t otimes(const mat41_t &q_left, const mat34_t &q_right)
{
    return mat34_t();
}

mat34_t otimes(const mat34_t &q_left, const mat41_t &q_right)
{
    return mat34_t();
}
