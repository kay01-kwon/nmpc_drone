#ifndef TOOLS_HPP_
#define TOOLS_HPP_
#include "type_definitions.hpp"

enum QuadModel{model1, model2};

void get_dqdt(const quat_t& q, 
const mat31_t& w, quat_t& dqdt);

void otimes(const quat_t& q1, 
const quat_t& q2, quat_t& q_res);

void conjugate(const quat_t& q,
quat_t& q_conj);



void get_rotm_from_quat();


#endif