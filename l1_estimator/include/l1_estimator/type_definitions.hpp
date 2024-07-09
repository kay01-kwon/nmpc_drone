#ifndef TYPE_DEFINITIONS_HPP_
#define TYPE_DEFINITIONS_HPP_
#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using Eigen::Quaternion;

using std::cout;
using std::endl;


typedef Matrix<double, 3, 1> mat31_t;
typedef Matrix<double, 3, 3> mat33_t;

typedef Matrix<double, 4, 4> mat44_t;
typedef Matrix<double, 4, 1> mat41_t;

typedef Matrix<double,6,4> mat64_t;

typedef Matrix<double,6,1> state6_t;
typedef Matrix<double,7,1> state7_t;
typedef Matrix<double,13,1> state13_t;
typedef Quaternion<double> quat_t;

enum class QuadModel{model1, model2};

typedef struct __Inertial_param{
    double m;
    mat33_t J;
    mat31_t r_offset;
} inertial_param_t;

typedef struct __Aero_coeff{
    double lift_coeff;
    double moment_coeff;
} aero_coeff_t;

#endif