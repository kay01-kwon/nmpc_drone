#ifndef TYPES_DEFINITIONS_H_
#define TYPES_DEFINITIONS_H_
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using std::cout;
using std::endl;

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Quaternion;

typedef Matrix<double, 4, 3> mat34_t;
typedef Matrix<double, 3, 3> mat33_t;
typedef Matrix<double, 3, 1> vector_t;

typedef Quaternion<double> quaternion_t;

#endif