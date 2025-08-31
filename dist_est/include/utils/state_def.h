#ifndef STATE_DEF_H
#define STATE_DEF_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 3, 1> Vec3;

typedef Eigen::Matrix<double, 6, 1> Vec6;

typedef Eigen::Matrix<double, 4, 1> QuatType;

typedef Eigen::Matrix<double, 4, 1> controlInputVector4;

typedef Eigen::Matrix<int16_t, 6, 1> rpmVector6;

typedef Eigen::Matrix<double, 6, 1> RotorThrustVector6;

typedef Eigen::Matrix<double, 7, 1> StateVector7;

typedef Eigen::Matrix<double, 7, 1> MeasVector7;

typedef Eigen::Matrix<double, 10, 1> AugStateVector10;

typedef Eigen::Matrix<double, 3, 1> MomentVector;

typedef Eigen::Matrix<double, 3, 1> DistVector;

typedef Eigen::Matrix<double, 3, 3> Mat3x3;

typedef Eigen::Matrix<double, 3, 4> Mat3x4;

typedef Eigen::Matrix<double, 4, 3> Mat4x3;

typedef Eigen::Matrix<double, 4, 4> Mat4x4;

typedef Eigen::Matrix<double, 4, 6> Mat4x6;

typedef Eigen::Matrix<double, 7, 7> Mat7x7;

typedef Eigen::Matrix<double, 7, 10> Mat7x10;

typedef Eigen::Matrix<double, 10, 10> Mat10x10;

typedef Eigen::Matrix<double, 10, 7> Mat10x7;

static Mat3x3 I3(Mat3x3::Identity());  // 3x3 identity matrix

static Mat10x10 I10(Mat10x10::Identity());  // 10x10 identity matrix

struct MavParam{

    double l{0.265};  // arm length in meters
    double m{2.9};  // mass in kg 
    Mat3x3 J;  // inertia matrix
    double C_T{1.465e-07};  // thrust coefficient
    double k_m{0.01569};  // moment constant (C_M/C_T)
};

struct EKFParams
{
    Mat10x10 P{0.01*Mat10x10::Identity()};  // initial covariance
    Mat10x10 Q{0.01*Mat10x10::Identity()};  // process noise covariance
    Mat7x7 R{0.01*Mat7x7::Identity()};  // measurement noise covariance
};

struct HgdoParams
{   
    // Gains for hgdo
    double eps{0.01};
};


#endif