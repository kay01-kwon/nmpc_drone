#ifndef TOOLS_HPP_
#define TOOLS_HPP_
#include "type_definitions.hpp"


// 1. dqdt from quaternion and angular velocity
void get_dqdt(const quat_t& q, 
const mat31_t& w, quat_t& dqdt);

quat_t& return_dqdt(const quat_t &q, 
const mat31_t& w);

// 2. The multiplication of two quaternions
void otimes(const quat_t& q1, 
const quat_t& q2, quat_t& q_res);

quat_t& return_otimes(const quat_t& q1, 
const quat_t &q2);

// 3. Get rotation matrix from quaternion
void get_rotm_from_quat(const quat_t& q,
mat33_t& rotm);

mat33_t& return_rotm_from_quaternion(const quat_t &q);

// 4. Conjugate the quaternion
void conjugate(const quat_t& q,
quat_t& q_conj);

mat33_t& return_rotm_from_quaternion(const quat_t &q);

// 5. Convert from vector (3D) to skew symmetric matrix (3x3)
void convert_vec_to_skew(const mat31_t& vec, 
mat33_t& skew_sym_mat);

// 6. quaternion tools
void convert_quat_to_quat_vec(const quat_t& q, 
mat31_t& q_vec);

void convert_quat_to_unit_quat(const quat_t& q, 
quat_t& unit_q);

// Signum to control reference model or test controller
double signum(double num);

void convert_thrust_to_wrench(const QuadModel& quad_model,
const double &arm_length, 
const mat31_t &COM,
const mat41_t &thrust,
const double &moment_coeff,
mat31_t &force,
mat31_t &moment);


#endif