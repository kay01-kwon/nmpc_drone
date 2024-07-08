#ifndef TOOLS_HPP_
#define TOOLS_HPP_
#include "type_definitions.hpp"

void get_dqdt(const quat_t& q, 
const mat31_t& w, quat_t& dqdt);

void otimes(const quat_t& q1, 
const quat_t& q2, quat_t& q_res);

void get_rotm_from_quat(const quat_t& q,
mat33_t& rotm);

void conjugate(const quat_t& q,
quat_t& q_conj);

void convert_vec_to_skew(const mat31_t& vec, 
mat33_t& skew_sym_mat);

void convert_quat_to_quat_vec(const quat_t& q, 
mat31_t& q_vec);

void convert_quat_to_unit_quat(const quat_t& q, 
quat_t& unit_q);

double signum(double num);

void convert_thrust_to_wrench(const QuadModel& quad_model,
const double &arm_length, 
const mat31_t &COM,
const mat41_t &thrust,
const double &moment_coeff,
mat31_t &force,
mat31_t &moment);


#endif