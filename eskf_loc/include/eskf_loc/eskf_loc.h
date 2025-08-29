#ifndef ESKF_LOC_H
#define ESKF_LOC_H
#include "utils/state_def.h"

struct EskfLocParams{
    Cov P_init{1e-3 * Eigen::Matrix<double, 18, 18>::Identity()}; // Initial covariance
    R measurement_noise_cov{0.001 * 0.001 * Eigen::Matrix<double, 7, 7>::Identity()};
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

    void propagate(const double &t_prev, 
                 const double &t_curr,
                 const Control &control,
                 const State &state_in,
                 const Cov &P_in);

    void correct(const Meas &z_meas,
                 const State &state_in,
                 const Cov &P_in);

    State getState() const;

    Cov getCovariance() const;

    private:

    // Initialize gravity vector
    Vec3 g_{0, 0, -9.81}; // Gravity vector in the body frame


    // Control input: [linear acceleration, angular velocity]
    Control control_;

    // Error state vector
    ErrorState del_state_;

    // State vector: [p, v, q, ab, wb, g]
    State state_;

    // Covariance matrix
    Cov P_;

    // State transition matrix w.r.t. the state vector
    Fs Fs_;

    // State transition matrix w.r.t. the impulse noise
    Fi Fi_;

    // Process noise covariance matrix in 12 x 12
    Qi Qi_;

    Qi Qi_dt_;

    // Measurement matrix

    // Measurement noise 7 x 7
    R R_;

    // Measurement Jacobian 7 x 18
    H H_;

    // Measurement Jacobian w.r.t. the state vector 7 x 19
    Hs Hs_;

    // State covariance matrix w.r.t. the state vector 19 x 19
    Sdels Sdels_;

    Mat18x18 G_;

};


#endif