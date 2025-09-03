#ifndef STATE_DEF_H
#define STATE_DEF_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 3, 1> Vec3d;

typedef Eigen::Matrix<double, 4, 1> Vec4d;

typedef Eigen::Matrix<double, 6, 1> Vec6d;

typedef Eigen::Matrix<int16_t, 6, 1> Vec6i16;

typedef Eigen::Matrix<double, 4, 1> Quatd;

typedef Eigen::Matrix<double, 13, 1> State;

typedef Eigen::Matrix<double, 19, 1> AugState;

typedef Eigen::Matrix<double, 3, 3> Mat3x3;


static Mat3x3 I3(Mat3x3::Identity());  // 3x3 identity matrix

static Vec3d g{0, 0, -9.81};  // gravity vector

struct MavParam{

    double l{0.265};  // arm length in meters
    double m{2.9};  // mass in kg 
    Mat3x3 J;  // inertia matrix
    double C_T{1.465e-07};  // thrust coefficient
    double k_m{0.01569};  // moment constant (C_M/C_T)
};

struct StateData{
    double time_stamp;
    Vec3d p;
    Vec3d v;
    Quatd q;
    Vec3d w;
};

struct RpmData{
    double time_stamp;
    Vec6i16 rpm;
};



#endif