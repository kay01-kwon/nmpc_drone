#ifndef MUXER_H
#define MUXER_H
#include "state_def.h"


void mux_state(const Vec3d &p,
               const Vec3d &v,
               const Quatd &q,
               const Vec3d &a_b,
               const Vec3d &w_b,
               const Vec3d &g,
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

    s_out(10) = a_b(0);
    s_out(11) = a_b(1);
    s_out(12) = a_b(2);

    s_out(13) = w_b(0);
    s_out(14) = w_b(1);
    s_out(15) = w_b(2);

    s_out(16) = g(0);
    s_out(17) = g(1);
    s_out(18) = g(2);
}

void mux_imu(const Vec3d &a,
             const Vec3d &w,
             Vec6d &u_out)
{
    u_out(0) = a(0);
    u_out(1) = a(1);
    u_out(2) = a(2);

    u_out(3) = w(0);
    u_out(4) = w(1);
    u_out(5) = w(2);
}

void mux_meas(const Vec3d &p,
              const Quatd &q,
              Vec7d &z_out)
{
    z_out(0) = p(0);
    z_out(1) = p(1);
    z_out(2) = p(2);

    z_out(3) = q(0);
    z_out(4) = q(1);
    z_out(5) = q(2);
    z_out(6) = q(3);
}

#endif