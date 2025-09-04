#ifndef ESKF_LOC_H
#define ESKF_LOC_H
#include "utils/state_def.h"

typedef Eigen::Matrix<double, 3, 3> Mat3x3;

typedef Eigen::Matrix<double, 4, 3> Mat4x3;
typedef Eigen::Matrix<double, 4, 4> Mat4x4;

typedef Eigen::Matrix<double, 7, 7> Mat7x7;
typedef Eigen::Matrix<double, 7, 18> Mat7x18;
typedef Eigen::Matrix<double, 7, 19> Mat7x19;

typedef Eigen::Matrix<double, 12, 12> Mat12x12;

typedef Eigen::Matrix<double, 18, 7> Mat18x7;
typedef Eigen::Matrix<double, 18, 12> Mat18x12;
typedef Eigen::Matrix<double, 18, 18> Mat18x18;

typedef Eigen::Matrix<double, 19, 18> Mat19x18;
typedef Eigen::Matrix<double, 19, 19> Mat19x19;

struct Meas{
    Vec3d p_meas;
    Quatd q_meas;
};

struct EskfLocParams{
    Mat18x18 P_init{1e-3 * Eigen::Matrix<double, 18, 18>::Identity()}; // Initial covariance
    Mat7x7 measurement_noise_cov{0.001 * 0.001 * Eigen::Matrix<double, 7, 7>::Identity()};
    double sigma_a_n{0.01}; // Standard deviation of accelerometer noise
    double sigma_a_w{0.01}; // Standard deviation of accelerometer bias noise
    double sigma_w_n{0.01}; // Standard deviation of gyroscope noise
    double sigma_w_w{0.01}; // Standard deviation of gyroscope bias noise
};


class EskfLoc {

    public:

    EskfLoc() = default;

    EskfLoc(const EskfLocParams &params);

    ~EskfLoc();

    void propagate(const Vec6d &u,
                   State &state_in,
                   Mat18x18 &P_in,
                   const double &dt);

    void correct(const Meas &meas,
                 State &state_in,
                 Mat18x18 &P_in);

    private:

    // Error state vector
    ErrorState del_state_;


    // State transition matrix w.r.t. the state vector
    Mat18x18 Fs_;

    // State transition matrix w.r.t. the impulse noise
    Mat18x12 Fi_;

    // Process noise covariance matrix in 12 x 12
    Mat12x12 Qi_;

    Mat12x12 Qi_dt_;

    // Measurement matrix

    // Measurement noise 7 x 7
    Mat7x7 R_;

    // Measurement Jacobian 7 x 18
    Mat7x18 H_;

    // Measurement Jacobian w.r.t. the state vector 7 x 19
    Mat7x19 Hs_;

    // State covariance matrix w.r.t. the state vector 19 x 19
    Mat19x18 Sdels_;

    Mat18x18 G_;

    size_t p_start{0};
    size_t v_start{3};
    size_t theta_start{6};
    size_t ab_start{9};
    size_t wb_start{12};
    size_t g_start{15};

};


#endif