#ifndef EKF_DIST_EST_H
#define EKF_DIST_EST_H

#include "utils/state_def.h"


typedef Eigen::Matrix<double, 3, 4> Mat3x4;
typedef Eigen::Matrix<double, 4, 3> Mat4x3;
typedef Eigen::Matrix<double, 4, 4> Mat4x4;

typedef Eigen::Matrix<double, 19, 1> AugState;

typedef Eigen::Matrix<double, 13, 13> Mat13x13;
typedef Eigen::Matrix<double, 13, 19> Mat13x19;
typedef Eigen::Matrix<double, 19, 13> Mat19x13;
typedef Eigen::Matrix<double, 19, 19> Mat19x19;

static Mat13x13 I13(Mat13x13::Identity());  // 13x13 identity matrix

static Mat19x19 I19(Mat19x19::Identity());  // 13x13 identity matrix

struct EKFParams
{
    Mat19x19 P{0.01*Mat19x19::Identity()};  // initial covariance
    Mat19x19 Q{0.01*Mat19x19::Identity()};  // process noise covariance
    Mat13x13 R{0.01*Mat13x13::Identity()};  // measurement noise covariance
};

struct EkfData{
    AugState s;
    Mat19x19 P;
};

class EkfDistEst
{
    public:

    EkfDistEst();

    EkfDistEst(const MavParam& param, const EKFParams &ekf_params);

    void setParam(const MavParam& param, const EKFParams &ekf_params);

    void propagate(const Vec4d &u,
                   EkfData &ekf_data,
                   const double &dt);

    void correct(const State &s_meas,
                 EkfData &ekf_data);

    ~EkfDistEst();

    private:

    // State transition matrix
    Mat19x19 F_;

    // Jacobian of Sensor model
    Mat13x19 H_;

    // Kalman gain
    Mat19x13 K_gain_;

    EKFParams ekf_params_;

    double m_;

    Mat3x3 J_, J_inv_;

    const size_t p_start{0}, v_start{3};
    
    const size_t q_start{6}, w_start{10};

    const size_t fext_start{13}, tau_start{16};

};


#endif