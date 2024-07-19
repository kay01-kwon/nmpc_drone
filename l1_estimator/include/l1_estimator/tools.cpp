#include "tools.hpp"

/**
 * @brief Get the rate of quaternion from current 
 * quaternion and angular velocity 
 * 
 * @param q current quaternion
 * @param w angualr velocity
 * @param dqdt the rate of quaternion
 */
void get_dqdt(const quat_t &q, const mat31_t &w, quat_t &dqdt)
{
    quat_t w_quat;
    quat_t q_otimes_w;

    w_quat.w() = 0;
    w_quat.x() = w(0);
    w_quat.y() = w(1);
    w_quat.z() = w(2);

    otimes(q, w_quat, q_otimes_w);
    
    dqdt.w() = 0.5*q_otimes_w.w();
    dqdt.x() = 0.5*q_otimes_w.x();
    dqdt.y() = 0.5*q_otimes_w.y();
    dqdt.z() = 0.5*q_otimes_w.z();

}

/**
 * @brief Multiplication of two different quaternion
 * 
 * @param q1 left quaternion
 * @param q2 right quaternion
 * @param q_res the result of multiplication
 */
void otimes(const quat_t &q1, const quat_t &q2, quat_t &q_res)
{
    double real;
    mat31_t q1_vec, q2_vec;
    mat33_t q1_skew_sym_mat;
    mat31_t q_res_vec;

    convert_quat_to_quat_vec(q1, q1_vec);
    convert_quat_to_quat_vec(q2, q2_vec);
    
    real = q1.w()*q2.w()
    -q1_vec.transpose()*q2_vec;

    convert_vec_to_skew(q1_vec, q1_skew_sym_mat);

    q_res_vec = q1.w()*q2_vec
    + q2.w()*q1_vec
    + q1_skew_sym_mat*q2_vec;

    q_res.w() = real;
    q_res.x() = q_res_vec(0);
    q_res.y() = q_res_vec(1);
    q_res.z() = q_res_vec(2);
}

/**
 * @brief Get the rotm from quaternion
 * 
 * @param q quaternion to convert into rotation matrix
 * @param rotm The resultant rotation matrix
 */
void get_rotm_from_quat(const quat_t &q, mat33_t &rotm)
{
    double qw, qx, qy, qz;
    
    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();

    rotm << 1-2*(qy*qy + qz*qz), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy),
    2*(qy*qx+qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx),
    2*(qz*qx-qw*qy), 2*(qz*qy+qw*qx), 1-2*(qx*qx+qy*qy);

}

/**
 * @brief Conjegatue the given quaternion
 * 
 * @param q the quaternion to conjugate
 * @param q_conj conjucated quaternion
 */
void conjugate(const quat_t &q, quat_t &q_conj)
{
    q_conj.w() = q.w();
    q_conj.x() = -q.x();
    q_conj.y() = -q.y();
    q_conj.z() = -q.z();
}

/**
 * @brief Convert 3 dimensional vector into skew symmetric matrix
 * 
 * @param vec 3 dimensional vector
 * @param skew_sym_mat 3 by 3 skew symmetric matrix from vec
 */
void convert_vec_to_skew(const mat31_t& vec, mat33_t &skew_sym_mat)
{    
    skew_sym_mat << 0, -vec(2), vec(1),
                    vec(2), 0, -vec(0),
                    -vec(1), vec(0), 0;
}

/**
 * @brief Extract the only imaginary part of the given quaternion.
 * Remove real part of it.
 * 
 * @param q quaternion to extract pure quaternion
 * @param q_vec pure quaternion
 */
void convert_quat_to_quat_vec(const quat_t& q, 
mat31_t &q_vec)
{
    q_vec << q.x(), q.y(), q.z();
}

/**
 * @brief Normalize the given quaternion
 * 
 * @param q quaternion to normalize
 * @param unit_q normalized quaternion
 */
void convert_quat_to_unit_quat(const quat_t& q, quat_t &unit_q)
{
    double den, qw, qx, qy ,qz;

    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();

    den = sqrt(qw * qw
    + qx * qx
    + qy * qy
    + qz * qz);

    assert(den > 0);

    unit_q.w() = q.w()/den;
    unit_q.x() = q.x()/den;
    unit_q.y() = q.y()/den;
    unit_q.z() = q.z()/den;

}

/**
 * @brief In the reference model, check the real part of the quaternion
 * and then stabilize the error of the attitude.
 * Its purpose is to avoid the unwinding pheonomenon.
 * 
 * @param num the real number to check its sign
 * @return signum
 */
double signum(double num)
{
    assert(typeid(num)==typeid(double));
    return num > 0 ? 1.0:-1.0;
}

/**
 * @brief Convert from four rotor thrusts to force and moment.
 * It considers the change of Center of mass.
 * 
 * @param quad_model QuadModel::model1 -'+', QuadMode::model2 - 'x'
 * @param arm_length the length from the center of geometry to that of rotor
 * @param B_p_CG_COM the location of center of mass from center of geometry expressed in body frame
 * @param thrust four rotor thrust obtained from the formula lift coeff*rpm^2
 * @param moment_coeff moment_coeff*thrust generates moment along z axis represented by body
 * @param force output force from the above parameters
 * @param moment output moment from the above parameters
 */
void convert_thrust_to_wrench(const mat31_t &B_p_CG_COM,
const mat31_t CG_p_CG_rotors[],
const mat41_t &thrust,
const double &moment_coeff,
mat31_t &force, 
mat31_t &moment)
{
    assert(fabs(B_p_CG_COM(2)) < 1e-6);
    assert(B_p_CG_COM.size() == 3);
    moment.setZero();

    double collective_thrust;

    collective_thrust = thrust(0) + thrust(1) + thrust(2) + thrust(3);
    
    force <<    0, 
                0,
                collective_thrust;

    mat31_t CG_p_COM_rotors[4];
    mat31_t thrust_xyz[4];
    mat33_t skew_symm;

    for(size_t i = 0; i < 4; i++)
    {
        thrust_xyz[i] << 0, 0, thrust(i);
        CG_p_COM_rotors[i] = CG_p_CG_rotors[i] - B_p_CG_COM;
        convert_vec_to_skew(CG_p_CG_rotors[i], skew_symm);
        moment = moment + skew_symm*thrust_xyz[i];
    }

    moment(2) = moment_coeff*(thrust(0) - thrust(1) + thrust(2) - thrust(3));
    

}