#ifndef STATE_DEF_H
#define STATE_DEF_H
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 3, 1> Vec3d;

typedef Eigen::Matrix<double, 6, 1> Vec6d;

typedef Eigen::Matrix<double, 7, 1> Vec7d;

typedef Eigen::Matrix<double, 4, 1> Quatd;

typedef Eigen::Matrix<double, 19, 1> State;

typedef Eigen::Matrix<double, 18, 1> ErrorState;

static Vec3d g_init(0, 0, -9.81); // Gravity vector in the body frame

#endif