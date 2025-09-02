#include "ekf_dist_est.h"
#include "utils/quaternion_utils.h"
#include "utils/state_demuxer.h"
#include "utils/state_muxer.h"
#include <cassert>
EkfDistEst::EkfDistEst()
{

}

EkfDistEst::EkfDistEst(const MavParam& param, const EKFParams &ekf_params)
{
    setParam(param, ekf_params);
}

void EkfDistEst::setParam(const MavParam& param, const EKFParams &ekf_params)
{
    
    F_.setIdentity();  // State transition matrix

    H_.setZero();
    H_.block<3, 3>(p_start, p_start) = I3;
    H_.block<3, 3>(v_start, v_start) = I3;
    H_.block<4, 4>(q_start, q_start) = Mat4x4::Identity();
    H_.block<3, 3>(w_start, w_start) = I3;

    K_gain_.setZero();

    ekf_params_ = ekf_params;

    m_ = param.m;  // mass

    J_ = param.J;  // inertia matrix

    J_inv_.setZero();

    for(size_t i = 0; i < 3; i++)
        J_inv_(i, i) = 1.0 / param.J(i, i);  // inverse of inertia matrix

}

void EkfDistEst::propagate(const Vec4d &u,
                           const AugState &s_in,
                           const Mat19x19 &P_in,
                           AugState &s_out,
                           Mat19x19 &P_out,
                           const double &dt)
{

    assert(dt >= 0.0 && "dt must be positive");

    Vec3d p_in, v_in;
    Quatd q_in;
    Vec3d w_in;
    Vec3d f_ext_in, tau_ext_in;

    demux_state(s_in, p_in, v_in, q_in, w_in, f_ext_in, tau_ext_in);

    Vec3d u_T_e3(0.0, 0.0, u(0));  // thrust vector in body frame

    Mat3x3 R_wb = quaternion_to_rotm(q_in);  // rotation from body to world frame

    Vec3d a_in;
    a_in = 1.0/m_* (R_wb * u_T_e3 + f_ext_in) + g_;  // control input in world frame

    Vec3d p_out, v_out;
    Quatd q_out;
    Vec3d w_out;
    Vec3d f_ext_out = f_ext_in;  // assume constant external force
    Vec3d tau_ext_out = tau_ext_in;  // assume constant external torque

    p_out = p_in + v_in * dt + 0.5 * a_in * dt * dt;
    v_out = v_in + a_in * dt;

    Quatd delta_q;
    delta_q << 1.0,
               0.5 * w_in(0) * dt,
               0.5 * w_in(1) * dt,
               0.5 * w_in(2) * dt;
    
    q_out = otimes(q_in, delta_q);

    w_out = w_in 
    + J_inv_ * (u.tail<3>() - w_in.cross(J_ * w_in) + tau_ext_in) * dt;

    mux_state(p_out, v_out, q_out, w_out, f_ext_out, tau_ext_out, s_out);

    Mat3x4 dRu_dq;
    double qw = q_in(0), qx = q_in(1), qy = q_in(2), qz = q_in(3);
    double wx = w_in(0), wy = w_in(1), wz = w_in(2);
    dRu_dq << qy, qz, qw, qx,
              -qx, -qw, qz, qy,
              0, -2*qx, -2*qy, 0;

    dRu_dq = 2*u(0) * dRu_dq;

    Mat3x4 da_dq = 1.0/m_ * dRu_dq;

    Mat4x4 dqw_dq;
    dqw_dq << 0, -wx, -wy, -wz,
              wx, 0, wz, -wy,
              wy, -wz, 0, wx,
              wz, wy, -wx, 0;
    
    Mat4x3 dqw_dw;
    dqw_dw << - qx, -qy, -qz,
               qw, -qz, qy,
               qz, qw, -qx,
               -qy, qx, qw;

    double Jxx = J_(0,0), Jyy = J_(1,1), Jzz = J_(2,2);

    double Jz_x = Jzz - Jxx;
    double Jx_z = Jxx - Jzz;
    double Jy_x = Jyy - Jxx;

    Mat3x3 dwJwdw;
    dwJwdw << 0, Jz_x*wz, Jy_x*wy,
            Jx_z*wz, 0, Jz_x*wx,
            Jy_x*wy, Jy_x*wx, 0;

    Mat3x3 dwdot_dw;
    dwdot_dw = Mat3x3::Identity() - J_inv_*dwJwdw*dt;

    F_.block<3,3>(p_start, v_start) = Mat3x3::Identity() * dt;
    F_.block<3,4>(p_start, q_start) = 0.5 * da_dq * dt * dt;
    F_.block<3,3>(p_start, fext_start) = 1.0/2.0/m_*dt*dt*Mat3x3::Identity();

    F_.block<3,4>(v_start, q_start) = da_dq * dt;
    F_.block<3,3>(v_start, fext_start) = 1.0/m_*dt*Mat3x3::Identity();

    F_.block<4,4>(q_start, q_start) = 0.5*dqw_dq*dt;
    F_.block<4,3>(q_start, w_start) = 0.5*dqw_dw*dt;

    F_.block<3,3>(w_start, w_start) = dwdot_dw;
    F_.block<3,3>(w_start, tau_start) = J_inv_*dt;

    P_out = F_ * P_in * F_.transpose() + ekf_params_.Q;

}

void EkfDistEst::correct(const State &s_meas,
                    const AugState &s_in,
                    const Mat19x19 &P_in,
                    AugState &s_out,
                    Mat19x19 &P_out)
{

    // Sensor model
    State s_sensor_model = H_*s_in;

    Mat13x13 S = H_ * P_in * H_.transpose() + ekf_params_.R;  // innovation covariance
    Mat13x13 S_inv;
    S_inv = S.ldlt().solve(I13);  // S = S^-1
    K_gain_ = P_in * H_.transpose() * S_inv;  // Kalman gain
    s_out = s_in + K_gain_ * (s_meas - s_sensor_model);
    P_out = (I19 - K_gain_*H_)*P_in;

}

EkfDistEst::~EkfDistEst()
{

}
