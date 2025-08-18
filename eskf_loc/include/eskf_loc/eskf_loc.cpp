#include "eskf_loc.h"
#include "utils/math_tool.h"
EskfLoc::EskfLoc(const EskfLocParams &params)
{
    // Initialize state and covariance
    Vec3 p_init, v_init;
    Quat q_init;
    Vec3 accel_bias_init, gyro_bias_init, g_init;

    p_init.setZero();
    v_init.setZero();
    q_init.setZero();
    q_init(0) = 1.0;
    accel_bias_init.setZero();
    gyro_bias_init.setZero();
    g_init = g_;

    state_<< p_init, 
             v_init, 
             q_init, 
             accel_bias_init,
             gyro_bias_init,  
             g_init;

    // Initialize control input
    control_.setZero();
    control_(2) = 9.81; // Set the z-component of the control input to gravity

    del_state_.setZero();

    Vec3 p_meas_init;
    Quat q_meas_init;
    p_meas_init.setZero();
    q_meas_init.setZero();
    q_meas_init(0) = 1.0;
    
    z_meas_<< p_meas_init, q_meas_init;

    P_ = params.P_init;

    Fs_.setIdentity();

    Fi_.setZero();
    Fi_.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
    Fi_.block(6,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
    Fi_.block(9,6,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
    Fi_.block(12,9,3,3) = Eigen::Matrix<double, 3, 3>::Identity();

    Qi_.setIdentity();
    Qi_.block(0, 0, 3, 3) = params.sigma_a_n 
                            * params.sigma_a_n 
                            * Eigen::Matrix<double, 3, 3>::Identity();
    Qi_.block(3, 3, 3, 3) = params.sigma_w_n 
                            * params.sigma_w_n 
                            * Eigen::Matrix<double, 3, 3>::Identity();
    Qi_.block(6, 6, 3, 3) = params.sigma_a_w
                            * params.sigma_a_w 
                            * Eigen::Matrix<double, 3, 3>::Identity();
    Qi_.block(9, 9, 3, 3) = params.sigma_w_w 
                            * params.sigma_w_w
                            * Eigen::Matrix<double, 3, 3>::Identity();

    Qi_dt_.setIdentity();

    R_ = params.measurement_noise_cov;    

    H_.setZero();
    Hs_.setZero();
    Hs_.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Hs_.block(3, 6, 4, 4) = Eigen::Matrix<double, 4, 4>::Identity();
    Sdels_.setIdentity();

    G_.setIdentity();
}

EskfLoc::~EskfLoc()
{
    // Destructor
    // No dynamic memory allocation, so nothing to clean up
}

void EskfLoc::predict(const double &t_curr,
             const double &t_prev,
             const Control &control)
{
    double dt;
    dt = t_curr - t_prev;

    // Update control input
    control_ = control;

    // Extract the current state
    Vec3 p = state_.head(3); // Position
    Vec3 v = state_.segment<3>(3); // Velocity
    Quat q = state_.segment<4>(6); // Orientation (quaternion)
    Vec3 accel_bias = state_.segment<3>(10); // Accelerometer bias
    Vec3 gyro_bias = state_.segment<3>(13); // Gyro bias
    Vec3 g = state_.segment<3>(16); // Gravity vector

    // Update position, velocity, and orientation
    Eigen::Matrix<double, 3, 3> rotm = quat_to_rotm(q);
    
    Vec3 acc_input = control_.head(3) - accel_bias; // Corrected accelerometer input
    
    p += v * dt + 0.5 * rotm*(acc_input + g) * dt * dt;
    v += (rotm*acc_input + g) * dt;
    
    // Update orientation using quaternion multiplication
    Vec3 omega = control_.tail(3) - gyro_bias;
    Quat dq;
    dq << 1.0,
          0.5 * omega(0) * dt,
          0.5 * omega(1) * dt, 
          0.5 * omega(2) * dt;
    
    q = otimes(q, dq);

    // Normalize quaternion
    double norm_q = q.norm();
    q /= norm_q;

    // Update the state vector
    state_ << p, v, q, accel_bias, gyro_bias, g;

    // Update the state transition matrix
    Fs_.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
    
    Mat3x3 Fs_12 = -rotm*vec3toSkew(acc_input)*dt;
    Mat3x3 Fs_13 = -rotm*dt;
    Mat3x3 Fs_15 = Eigen::Matrix<double, 3, 3>::Identity() * dt;
    
    Vec3 del_theta = omega*dt;
    Mat3x3 R_omega = angle_axis_vec_to_rotm(del_theta);

    Mat3x3 Fs_22;
    Fs_22 = R_omega.transpose();
    Mat3x3 Fs_24 = -Mat3x3::Identity() * dt;

    Fs_.block(3, 6, 3, 3) = Fs_12;
    Fs_.block(3, 9, 3, 3) = Fs_13;
    Fs_.block(3, 15, 3, 3) = Fs_15;

    Fs_.block(6, 6, 3, 3) = Fs_22;
    Fs_.block(6, 12, 3, 3) = Fs_24;

    Qi_dt_.block(0, 0, 6, 6) = dt * dt * Eigen::Matrix<double, 6, 6>::Identity();
    Qi_dt_.block(6, 6, 6, 6) = dt * Eigen::Matrix<double, 6, 6>::Identity();

    P_ = Fs_ * P_ * Fs_.transpose() + Fi_ * Qi_*Qi_dt_ * Fi_.transpose();

}

void EskfLoc::correct(const Meas &z_meas)
{
    // Update measurement variables
    z_meas_ = z_meas;
    Meas delta_s = z_meas_ - Hs_ * state_;

    Quat q;
    q = state_.segment<4>(6); // Extract the quaternion from the state

    double qw, qx, qy, qz;
    qw = q(0);
    qx = q(1);
    qy = q(2);
    qz = q(3);

    Mat4x3 Q_del_theta;

    Q_del_theta << -qx, -qy, -qz,
                   qw, -qz,  qy,
                   qz,  qw, -qx,
                  -qy,  qx,  qw;
    Q_del_theta = 0.5 * Q_del_theta;

    Sdels_.block(6, 6, 4, 3) = Q_del_theta;

    // Compute jacobian of the measurement w.r.t the error state
    H_ = Hs_ * Sdels_;

    // Compute the Kalman gain
    Mat18x7 K;
    Mat7x7 S, S_inv;
    S = H_ * P_ * H_.transpose() + R_;
    S_inv = S.ldlt().solve(Eigen::Matrix<double, 7, 7>::Identity());
    K = P_ * H_.transpose() * S_inv;

    // Update the error state
    del_state_ = K * delta_s;

    P_ = (Cov::Identity() - K*H_) * P_;

    Vec3 p_inj, v_inj;
    Quat q_inj;
    Vec3 accel_inj, gyro_inj, g_inj;

    p_inj = state_.head(3) + del_state_.head(3); // Position
    v_inj = state_.segment<3>(3) + del_state_.segment<3>(3); // Velocity
    
    Quat q_state, del_q;
    Vec3 del_theta;
    del_theta = del_state_.segment<3>(6);
    del_q << 1.0,
              0.5 * del_theta(0),
              0.5 * del_theta(1), 
              0.5 * del_theta(2);
    q_state = state_.segment<4>(6);
    q_inj = otimes(q_state, del_q);

    // Normalize quaternion
    double norm_q = q_inj.norm();
    q_inj /= norm_q;

    accel_inj = state_.segment<3>(10) + del_state_.segment<3>(9);
    gyro_inj = state_.segment<3>(13) + del_state_.segment<3>(12);
    g_inj = state_.segment<3>(16) + del_state_.segment<3>(15); // Gravity vector

    state_ << p_inj, 
              v_inj, 
              q_inj, 
              accel_inj,
              gyro_inj,  
              g_inj;

    Mat3x3 temp;
    temp = Mat3x3::Identity() - 0.5 * vec3toSkew(del_theta);
    G_.block(6, 6, 3, 3) = temp;

    P_ = G_*P_*G_.transpose();

    del_state_.setZero(); // Reset the delta state after correction
}

State EskfLoc::getState() const
{
    return state_;
}

Cov EskfLoc::getCovariance() const
{
    return P_;
}