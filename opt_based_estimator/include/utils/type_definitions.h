#ifndef TYPE_DEFINITIONS_H_
#define TYPE_DEFINITIONS_H_
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using std::cout;
using std::endl;

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Quaternion;

typedef Matrix<double, 3, 3> mat33_t;
typedef Matrix<double, 4, 1> mat41_t;
typedef Matrix<double, 4, 3> mat43_t;
typedef Matrix<double, 4, 4> mat44_t;
typedef Matrix<double, 7, 3> mat73_t;
typedef Matrix<double, 7, 7> mat77_t;

typedef Matrix<double, 3, 1> vector_t;
typedef Matrix<double, 7, 1> rotational_state_t;

typedef Quaternion<double> quaternion_t;

#endif