#ifndef QUATERNION_MATH_TOOLS_H_
#define QUATERNION_MATH_TOOLS_H_
#include "type_definitions.h"

/**
 * @brief Arg - quaternion form, output - quaternion form
 * 
 * @param q_left q.w(), q.y() ,q.x() ,q.z()
 * @param q_right q.w(), q.y() ,q.x() ,q.z()
 * @return quaternion_t 
 */
quaternion_t otimes(const quaternion_t &q_left, const quaternion_t &q_right);

/**
 * @brief Arg - matrix form, Output - matrix form
 * 
 * @param q_left 4 by 1 (w, x, y, z)
 * @param q_right 4 by 1 (w, x, y, z)
 * @return mat41_t 
 */
mat41_t otimes(const mat41_t &q_left, const mat41_t &q_right);

/**
 * @brief Arg - Matrix form, Output - Matrix form
 * 
 * @param q_left 4 by 1 (w, x, y, z)
 * @param q_right 4 by 3
 * @return mat34_t 
 */
mat43_t otimes(const mat41_t &q_left, const mat43_t &q_right);

/**
 * @brief Arg - matrix form, Output - matrix form
 * 
 * @param q_left Three quaternions with four elements (w, x, y, z) 
 * @param q_right (w, x, y, z)
 * @return mat43_t 
 */
mat43_t otimes(const mat43_t &q_left, const mat41_t &q_right);


#endif