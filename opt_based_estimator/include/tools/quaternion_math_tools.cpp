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

    return Lambda*q_right;
}

mat43_t otimes(const mat41_t &q_left, const mat43_t &q_right)
{
    mat43_t columns;
    for(size_t i = 0; i < 3; i++)
    {
        mat41_t q_temp = q_right.col(i);

        columns.col(i) = otimes(q_left, q_temp);
    }

    return columns;
}

mat43_t otimes(const mat43_t &q_left, const mat41_t &q_right)
{
    mat43_t columns;
    for(size_t i = 0; i < 3; i++)
    {
        mat41_t q_temp = q_left.col(i);

        columns.col(i) = otimes(q_temp, q_right);
    }

    return columns;
}