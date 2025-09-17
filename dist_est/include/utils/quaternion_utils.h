#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H
#include "state_def.h"

typedef Eigen::Matrix<double, 4, 4> Mat4x4;

Quatd otimes(const Quatd &q1, const Quatd &q2);

Mat3x3 q_vec_to_skew(const Vec3d &v);

Mat3x3 w_cross_J_w_derivative(const Mat3x3 &J, const Vec3d& w);

Mat3x3 quaternion_to_rotm(const Quatd &q);

#endif