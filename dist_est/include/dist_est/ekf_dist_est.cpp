#include "ekf_dist_est.h"
#include "utils/quaternion_utils.h"
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
    converter_ = new FDynamics();
    converter_->setParam(param);

    s_pred_.setZero();
    s_est_.setZero();

    s_pred_(0) = 1;
    s_est_(0) = 1;

    P_pred_ = ekf_params.P;  // Initial covariance
    P_est_ = ekf_params.P;  // Initial covariance

    ekf_params_ = ekf_params;

    J_ = param.J;  // inertia matrix

    J_inv_.setZero();

    for(size_t i = 0; i < 3; i++)
        J_inv_(i, i) = 1.0 / param.J(i, i);  // inverse of inertia matrix

}

AugStateVector10 EkfDistEst::predict(const rpmVector6 &rpm, const double &dt)
{

    assert(dt >= 0.0 && "dt must be positive");

    // Convert rpm to control input
    controlInputVector4 u;
    converter_->convert_rpm_to_control_input(rpm, u);

    // QuatType q(s_est_(0), s_est_(1), s_est_(2), s_est_(3));
    QuatType q_prior;
    Vec3 w_prior, tau_prior;
    Mat10x10 P_prior;
    
    q_prior << s_est_(0), s_est_(1), s_est_(2), s_est_(3);
    w_prior << s_est_(4), s_est_(5), s_est_(6);
    tau_prior << s_est_(7), s_est_(8), s_est_(9);

    P_prior = P_est_;

    double qw, qx, qy, qz;
    double wx, wy, wz;

    qw = q_prior(0);
    qx = q_prior(1);
    qy = q_prior(2);
    qz = q_prior(3);

    wx = w_prior(0);
    wy = w_prior(1);
    wz = w_prior(2);

    QuatType del_q;
    del_q << 1.0, wx*dt/2.0, wy*dt/2.0, wz*dt/2.0;

    Vec3 M = u.segment(1, 3);  // control input torque

    QuatType q_pred;
    Vec3 w_pred, tau_pred;

    q_pred = otimes(q_prior, del_q);
    q_pred.normalize();  // Normalize quaternion to avoid drift
    w_pred = w_prior + J_inv_*(M - w_prior.cross(J_ * w_prior) + tau_prior)*dt;
    tau_pred = tau_prior;

    // std::cout << "Moment : " << M.transpose() << std::endl;

    s_pred_ << q_pred, w_pred, tau_pred;

    Mat4x4 dqdq;
    Mat4x3 dqdw, dqdtau;

    Mat3x4 dwdq;
    Mat3x3 dwdw, dwdtau;

    Mat3x4 dtaudq;
    Mat3x3 dtaudw, dtaudtau;

    Mat4x4 w_prior_skew;
    w_prior_skew << 0.0, -wx, -wy, -wz,
                     wx, 0.0, wz, -wy,
                     wy, -wz, 0.0, wx,
                     wz, wy, -wx, 0.0;

    Mat4x3 q_prior_skew; 

    q_prior_skew << -qx, -qy, -qz,
                    qw, -qz, qy,
                    qz, qw, -qx,
                    -qy, qx, qw;

    dqdq = Mat4x4::Identity() + 0.5*dt*w_prior_skew;
    dqdw = 0.5*dt*q_prior_skew;
    // dqdq << 1, -del_q_vec.transpose(),
    //         del_q_vec, I3 - q_vec_to_skew(del_q_vec);
    
    // dqdw << -0.5*q_vec.transpose()*dt,
    //         0.5*q(0)*I3*dt 
    //         + 0.5*dt*q_vec_to_skew(q_vec);

    dqdtau.setZero();

    dwdq.setZero();
    dwdw = I3 - J_inv_*w_cross_J_w_derivative(J_, w_prior)*dt;
    dwdtau = J_inv_*dt;

    dtaudq.setZero();
    dtaudw.setZero();
    dtaudtau.setIdentity();

    Mat10x10 F;
    F << dqdq, dqdw, dqdtau,
         dwdq, dwdw, dwdtau,
         dtaudq, dtaudw, dtaudtau;

    P_pred_ = F * P_prior * F.transpose() + ekf_params_.Q;


    return s_pred_;
}

AugStateVector10 EkfDistEst::meas_update(const StateVector7 &s_meas)
{

    // Measurement model
    QuatType q(s_pred_(0), s_pred_(1), s_pred_(2), s_pred_(3));
    Vec3 w(s_pred_(4), s_pred_(5), s_pred_(6));
    Vec3 tau(s_pred_(7), s_pred_(8), s_pred_(9));

    QuatType q_meas(s_meas(0), s_meas(1), s_meas(2), s_meas(3));
    Vec3 w_meas(s_meas(4), s_meas(5), s_meas(6));

    q_meas.normalize();  // Normalize the measurement quaternion

    Mat7x10 H;
    H.setZero();

    H << Mat4x4::Identity(), Mat4x3::Zero(), Mat4x3::Zero(),
         Mat3x4::Zero(), I3, Mat3x3::Zero();
    

    Mat7x7 S = H * P_pred_ * H.transpose() + ekf_params_.R;
    
    Mat10x7 K = P_pred_ * H.transpose() * S.inverse();

    MeasVector7 y, h;
    StateVector7 s_meas_normalized;
    s_meas_normalized << q_meas, w_meas;

    h << q, w;
    y = s_meas_normalized - h;

    // assert(!std::isnan(y.norm()));

    // Update state
    s_est_ = s_pred_ + K * y;

    // Update covariance
    P_est_ = (I10 - K * H) * P_pred_;


    return s_est_;
}

EkfDistEst::~EkfDistEst()
{
    if (converter_) {
        delete converter_;
        converter_ = nullptr;
    }
}
