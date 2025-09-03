#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H
#include "state_def.h"

typedef Eigen::Matrix<double, 4, 4> Mat4x4;

Quatd otimes(const Quatd &q1, const Quatd &q2)
{
    double qw, qx, qy, qz;

    qw = q1(0);
    qx = q1(1);
    qy = q1(2);
    qz = q1(3);

    Mat4x4 q1_mat;
    
    q1_mat << qw, -qx, -qy, -qz,
               qx,  qw, -qz,  qy,
               qy,  qz,  qw, -qx,
               qz, -qy,  qx,  qw;
    return q1_mat * q2;
}

Mat3x3 q_vec_to_skew(const Vec3d &v)
{
    Mat3x3 skew;
    skew << 0.0, -v(2), v(1),
            v(2), 0.0, -v(0),
            -v(1), v(0), 0.0;
    return skew;
}

Mat3x3 w_cross_J_w_derivative(const Mat3x3 &J, const Vec3d& w)
{
    double wx = w(0);
    double wy = w(1);
    double wz = w(2);

    double Jxx = J(0,0);
    double Jyy = J(1,1);
    double Jzz = J(2,2);

    double J_zy = Jzz - Jyy;
    double J_xz = Jxx - Jzz;
    double J_yx = Jyy - Jxx;

    Mat3x3 deriv;

    deriv << 0.0, J_zy*wz, J_zy*wy,
            J_xz*wz, 0.0, J_xz*wx,
            J_yx*wy, J_yx*wx, 0.0;
    return deriv;
}

Mat3x3 quaternion_to_rotm(const Quatd &q)
{
    double qw = q(0);
    double qx = q(1);
    double qy = q(2);
    double qz = q(3);

    Mat3x3 R;

    R << 1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy),
         2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx),
         2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy);
    return R;
}

#endif