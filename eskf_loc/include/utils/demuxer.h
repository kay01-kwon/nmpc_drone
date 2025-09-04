#ifndef DEMUXER_H
#define DEMUXER_H
#include "state_def.h"


void demux_state(const State &s_in,
                 Vec3d &p,
                 Vec3d &v,
                 Quatd &q,
                 Vec3d &a_b,
                 Vec3d &w_b,
                 Vec3d &g)
{
    p << s_in(0), s_in(1), s_in(2);
    v << s_in(3), s_in(4), s_in(5);
    q << s_in(6), s_in(7), s_in(8), s_in(9);
    a_b << s_in(10), s_in(11), s_in(12);
    w_b << s_in(13), s_in(14), s_in(15);
    g << s_in(16), s_in(17), s_in(18);
}

void demux_error_state(const ErrorState &dx_in,
                        Vec3d &dp,
                        Vec3d &dv,
                        Vec3d &dtheta,
                        Vec3d &da_b,
                        Vec3d &dw_b,
                        Vec3d &dg)
{
    dp << dx_in(0), dx_in(1), dx_in(2);
    dv << dx_in(3), dx_in(4), dx_in(5);
    dtheta << dx_in(6), dx_in(7), dx_in(8);
    da_b << dx_in(9), dx_in(10), dx_in(11);
    dw_b << dx_in(12), dx_in(13), dx_in(14);
    dg << dx_in(15), dx_in(16), dx_in(17);
}

void demux_imu(const Vec6d &u_in,
              Vec3d &a,
              Vec3d &w)
{
    a << u_in(0), u_in(1), u_in(2);
    w << u_in(3), u_in(4), u_in(5);
}

void demux_meas(const Vec7d &z_in,
               Vec3d &p,
               Quatd &q)
{
    p << z_in(0), z_in(1), z_in(2);
    q << z_in(3), z_in(4), z_in(5), z_in(6);
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






#endif