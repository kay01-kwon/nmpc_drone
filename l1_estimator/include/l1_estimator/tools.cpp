#include "tools.h"
#include "tools.hpp"

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

void get_rotm_from_quat(const quat_t &q, mat33_t &rotm)
{

}

void conjugate(const quat_t &q, quat_t &q_conj)
{
    q_conj.w() = q.w();
    q_conj.x() = -q.x();
    q_conj.y() = -q.y();
    q_conj.z() = -q.z();
}

void convert_vec_to_skew(const mat31_t& vec, mat33_t &skew_sym_mat)
{
    skew_sym_mat << 0, -vec(2), vec(1),
                    vec(2), 0, -vec(3),
                    -vec(1), vec(0), 0;
}

void convert_quat_to_quat_vec(const quat_t& q, 
mat31_t &q_vec)
{
    q_vec << q.x(), q.y(), q.z();
}

void convert_quat_to_unit_quat(const quat_t& q, quat_t &unit_q)
{
    double den;

    den = sqrt(
        q.x()*q.x()
        + q.y()*q.y()
        + q.z()*q.z()
        + q.w()*q.w()
    );

    unit_q.x() = q.x()/den;
    unit_q.y() = q.y()/den;
    unit_q.z() = q.z()/den;
    unit_q.w() = q.w()/den;
    
}

double signum(double num)
{
    return num > 0 ? 1.0:-1.0;
}