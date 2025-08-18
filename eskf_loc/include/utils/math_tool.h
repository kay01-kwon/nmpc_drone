#ifndef MATH_TOOL_H
#define MATH_TOOL_H
#include "state_def.h"

Eigen::Matrix<double, 3, 3> vec3toSkew(const Vec3 &vec)
{
    Eigen::Matrix<double, 3, 3> skew;
    skew << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return skew;
}

Eigen::Matrix<double, 3, 3> angle_axis_vec_to_rotm(const Vec3 &angle_axis_vec)
{
    double angle = angle_axis_vec.norm();
    
    Vec3 axis;
    if (angle < 1e-30)
    {
        axis = Vec3::Zero();
    }
    else
    {
        axis = angle_axis_vec / angle;
    }

    Eigen::Matrix<double, 3, 3> skew = vec3toSkew(axis);

    Eigen::Matrix<double, 3, 3> rotm;

    rotm = Eigen::Matrix<double, 3, 3>::Identity() + 
           sin(angle) * skew + 
           (1 - cos(angle)) * skew * skew;

    return rotm;
}

Eigen::Matrix<double, 3, 3 > quat_to_rotm(const Quat &q)
{
    double qw = q(0);
    double qx = q(1);
    double qy = q(2);
    double qz = q(3);

    Eigen::Matrix<double, 3, 3> rotm;

    rotm << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
            2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy);

    return rotm;
}

Quat otimes(const Quat &q1, const Quat &q2)
{
    double qw, qx, qy, qz;
    qw = q1(0);
    qx = q1(1);
    qy = q1(2);
    qz = q1(3);

    Eigen::Matrix<double, 4, 4> q1_mat;

    q1_mat << qw, -qx, -qy, -qz,
              qx,  qw, -qz,  qy,
              qy,  qz,  qw, -qx,
              qz, -qy,  qx,  qw;

    return q1_mat * q2;
}


#endif