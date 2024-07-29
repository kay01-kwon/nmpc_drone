#ifndef QUATERNION_MATH_TOOLS_H_
#define QUATERNION_MATH_TOOLS_H_
#include "type_definitions.h"

/**
 * @brief multiplication of two quaternions
 * 
 * @param q_left (w, x, y, z)
 * @param q_right (w, x, y, z)
 * @return mat41_t 
 */
mat41_t otimes(const mat41_t &q_left, const mat41_t &q_right);

/**
 * @brief q_left: A quaternion, q_right: three quaternions
 * 
 * @param q_left (w, x, y, z)
 * @param q_right three quaternion with four elements (w, x, y, z)
 * @return mat34_t 
 */
mat43_t otimes(const mat41_t &q_left, const mat43_t &q_right);

/**
 * @brief q_left: Three quaternions, q_right: A quaternion
 * 
 * @param q_left Three quaternions with four elements (w, x, y, z) 
 * @param q_right (w, x, y, z)
 * @return mat43_t 
 */
mat43_t otimes(const mat43_t &q_left, const mat41_t &q_right);


#endif