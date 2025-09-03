#ifndef STATE_MUXER_H
#define STATE_MUXER_H
#include "state_def.h"

/**
 * @brief Mux the state vector from position, velocity, orientation (quaternion), and angular velocity.
 * 
 * @param p position
 * @param v linear velocity (World frame)
 * @param q quaternion (Body to world)
 * @param w angular velocity (Body frame)
 * @param s_out 
 */
void mux_state(const Vec3d &p,
               const Vec3d &v,
               const Quatd &q,
               const Vec3d &w,
               State &s_out)
{
    s_out(0) = p(0);
    s_out(1) = p(1);
    s_out(2) = p(2);

    s_out(3) = v(0);
    s_out(4) = v(1);
    s_out(5) = v(2);

    s_out(6) = q(0);
    s_out(7) = q(1);
    s_out(8) = q(2);
    s_out(9) = q(3);

    s_out(10) = w(0);
    s_out(11) = w(1);
    s_out(12) = w(2);
}

/**
 * @brief Mux the state vector from position, velocity, orientation (quaternion), angular velocity, external force and torque.
 * 
 * @param p position
 * @param v linear velocity (World frame)
 * @param q quaternion (Body to world)
 * @param w angular velocity (Body frame)
 * @param f_ext External force estimate (World frame)
 * @param tau_ext External moment estimate (Body frame)
 * @param s_out 
 */

void mux_state(const Vec3d &p,
               const Vec3d &v,
               const Quatd &q,
               const Vec3d &w,
               const Vec3d &f_ext,
               const Vec3d &tau_ext,
               AugState &s_out)
{
    s_out(0) = p(0);
    s_out(1) = p(1);
    s_out(2) = p(2);

    s_out(3) = v(0);
    s_out(4) = v(1);
    s_out(5) = v(2);

    s_out(6) = q(0);
    s_out(7) = q(1);
    s_out(8) = q(2);
    s_out(9) = q(3);

    s_out(10) = w(0);
    s_out(11) = w(1);
    s_out(12) = w(2);

    s_out(13) = f_ext(0);
    s_out(14) = f_ext(1);
    s_out(15) = f_ext(2);

    s_out(16) = tau_ext(0);
    s_out(17) = tau_ext(1);
    s_out(18) = tau_ext(2);
    
}


#endif