#ifndef STATE_DEF_H
#define STATE_DEF_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 3, 1> Vec3;

typedef Eigen::Matrix<double, 3, 3> Mat3x3;

typedef Eigen::Matrix<double, 4, 1> Quat;

typedef Eigen::Matrix<double, 4, 3> Mat4x3;

typedef Eigen::Matrix<double, 4, 4> Mat4x4;

typedef Eigen::Matrix<double, 18, 1> ErrorState;

typedef Eigen::Matrix<double, 19, 1> State;

typedef Eigen::Matrix<double, 18, 7> Mat18x7;

typedef Eigen::Matrix<double, 18, 18> Cov;

// Linear acceleration and angular velocity
typedef Eigen::Matrix<double, 6, 1> Control;

typedef Eigen::Matrix<double, 7, 1> Meas;

typedef Eigen::Matrix<double, 7, 7> Mat7x7;

typedef Eigen::Matrix<double, 18, 18> Fs;

typedef Eigen::Matrix<double, 18, 18> Mat18x18;

typedef Eigen::Matrix<double, 18, 12> Fi;

typedef Eigen::Matrix<double, 12, 12> Qi;

typedef Eigen::Matrix<double, 7, 7> R;

typedef Eigen::Matrix<double, 7, 18> H;

typedef Eigen::Matrix<double, 7, 19> Hs;

typedef Eigen::Matrix<double, 19, 18> Sdels;

#endif