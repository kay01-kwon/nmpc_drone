#ifndef STATE_DEMUXER_H
#define STATE_DEMUXER_H

#include "state_def.h"

/**
 * @brief Demux the state vector into position, linear velocity, orientation (quaternion), and angular velocity.
 * 
 * @param s_in State vector
 * @param p position
 * @param v linear velocity (World frame)
 * @param q quaternion (Body to world)
 * @param w angular velocity (Body frame)
 */
void demux_state(const State &s_in,
                 Vec3d &p,
                 Vec3d &v,
                 Quatd &q,
                 Vec3d &w)
{
    p << s_in(0), s_in(1), s_in(2);
    v << s_in(3), s_in(4), s_in(5);
    q << s_in(6), s_in(7), s_in(8), s_in(9);
    w << s_in(10), s_in(11), s_in(12);
}

/**
 * @brief Demux augemented state into position, linear velocity, orientation (quaternion), angular velocity, external force, and external torque.
 * 
 * @param s_in Augmented state vector
 * @param p position 
 * @param v linear velocity (World frame)
 * @param q quaternion (Body to world)
 * @param w angular velocity (Body frame)
 * @param f_ext External force estimate (World frame)
 * @param tau_ext External moment estimate (Body frame)
 */

void demux_state(const AugState &s_in,
                 Vec3d &p,
                 Vec3d &v,
                 Quatd &q,
                 Vec3d &w,
                 Vec3d &f_ext,
                 Vec3d &tau_ext)
{
    p << s_in(0), s_in(1), s_in(2);
    v << s_in(3), s_in(4), s_in(5);
    q << s_in(6), s_in(7), s_in(8), s_in(9);
    w << s_in(10), s_in(11), s_in(12);
    f_ext << s_in(13), s_in(14), s_in(15);
    tau_ext << s_in(16), s_in(17), s_in(18);
}

#endif