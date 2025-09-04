#include "eskf_loc.h"
#include "utils/math_tool.h"
#include "utils/muxer.h"
#include "utils/demuxer.h"

EskfLoc::EskfLoc(const EskfLocParams &params)
{
    // Initialize state and covariance
    Vec3d p_init, v_init;
    Quatd q_init;
    Vec3d accel_bias_init, gyro_bias_init, g_init;


    del_state_.setZero();

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

void EskfLoc::propagate(const Vec6d &u,
                        State &state_in,
                        Mat18x18 &P_in,
                        const double &dt)
{

    // Extract the current state
    Vec3d p, v;
    Quatd q;
    Vec3d accel_bias, gyro_bias, g;

    demux_state(state_in, p, v, 
    q, accel_bias, gyro_bias, g);

    // Update position, velocity, and orientation
    Eigen::Matrix<double, 3, 3> rotm = quat_to_rotm(q);
    
    Vec3d acc_input;
    Vec3d w;

    demux_imu(u, acc_input, w);

    // Corrected acceleration and angular velocity
    acc_input = acc_input - accel_bias;
    w = w - gyro_bias;
    
    // Eq (260a - b)
    p += v * dt + 0.5 * rotm*(acc_input + g) * dt * dt;
    v += (rotm*acc_input + g) * dt;
    
    Quatd dq;
    dq << 1.0,
          0.5 * w(0) * dt,
          0.5 * w(1) * dt, 
          0.5 * w(2) * dt;
    
    // Eq (260c)
    q = otimes(q, dq);

    // Normalize quaternion
    double norm_q = q.norm();
    q /= norm_q;

    // Do not update bias and gravity vector in propagation step
    mux_state(p, v, q, 
    accel_bias , gyro_bias, g, 
    state_in);

    // Update the state transition matrix
    // Eq (270)
    Fs_.block<3, 3>(p_start, v_start) = Mat3x3::Identity() * dt;
    
    Fs_.block<3, 3>(v_start, theta_start) = -rotm*vec3toSkew(acc_input)*dt;
    Fs_.block<3, 3>(v_start, ab_start) = -rotm*dt;
    Fs_.block<3, 3>(v_start, g_start) = Mat3x3::Identity()*dt;
    
    Vec3d del_theta = w*dt;
    Mat3x3 R_omega = angle_axis_vec_to_rotm(del_theta);

    Mat3x3 Fs_22;
    Fs_22 = R_omega.transpose();
    Fs_.block<3, 3>(theta_start, theta_start) = Fs_22;
    Fs_.block<3, 3>(theta_start, wb_start) = -Mat3x3::Identity()*dt;


    Qi_dt_.block<6, 6>(0, 0) = dt * dt * Eigen::Matrix<double, 6, 6>::Identity();
    Qi_dt_.block<6, 6>(6, 6) = dt * Eigen::Matrix<double, 6, 6>::Identity();

    P_in = Fs_ * P_in * Fs_.transpose() + Fi_ * Qi_*Qi_dt_ * Fi_.transpose();

}

void EskfLoc::correct(const Meas &meas,
                      State &state_in,
                      Mat18x18 &P_in)
{
    // Update measurement variables
    Vec7d z_meas;
    mux_meas(meas.p_meas, meas.q_meas, 
    z_meas);
    Vec7d delta_s = z_meas - Hs_ * state_in;

    Vec3d p, v;
    Quatd q;
    Vec3d accel_bias, gyro_bias, g;

    demux_state(state_in, p, v,
    q, accel_bias, gyro_bias, g);

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
    S = H_ * P_in * H_.transpose() + R_;
    S_inv = S.ldlt().solve(Mat7x7::Identity());
    K = P_in * H_.transpose() * S_inv;

    // Update the error state
    del_state_ = K * delta_s;

    P_in = (Mat18x18::Identity() - K*H_) * P_in;

    Vec3d p_err, v_err, del_theta, ab_err, wb_err, g_err;
    demux_error_state(del_state_, p_err, v_err,
    del_theta, ab_err, wb_err, g_err);


    Vec3d p_inj, v_inj;
    Quatd q_inj;
    Vec3d accel_inj, gyro_inj, g_inj;


    p_inj = p + p_err; // Position
    v_inj = v + v_err; // Velocity
    
    Quatd del_q;
    del_q << 1.0,
              0.5 * del_theta(0),
              0.5 * del_theta(1), 
              0.5 * del_theta(2);
    q_inj = otimes(q, del_q);

    // Normalize quaternion
    double norm_q = q_inj.norm();
    q_inj /= norm_q;

    accel_inj = accel_bias + ab_err;
    gyro_inj = gyro_bias + wb_err;
    g_inj = g + g_err; // Gravity vector

    mux_state(p_inj, v_inj, q_inj, 
    accel_inj, gyro_inj, g_inj, 
    state_in);

    Mat3x3 temp;
    temp = Mat3x3::Identity() - 0.5 * vec3toSkew(del_theta);
    G_.block(6, 6, 3, 3) = temp;

    P_in = G_*P_in*G_.transpose();

    del_state_.setZero(); // Reset the delta state after correction
}