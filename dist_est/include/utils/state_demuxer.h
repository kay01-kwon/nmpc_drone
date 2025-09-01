#ifndef STATE_DEMUXER_H
#define STATE_DEMUXER_H

#include "state_def.h"

void demux_state(const State &s_in,
                 Vec3d &pos,
                 Vec3d &linear_vel,
                 Quatd &q,
                 Vec3d &angular_vel)
{
    pos << s_in(0), s_in(1), s_in(2);
    linear_vel << s_in(3), s_in(4), s_in(5);
    q << s_in(6), s_in(7), s_in(8), s_in(9);
    angular_vel << s_in(10), s_in(11), s_in(12);
}

void demux_state(const AugState &s_in,
                 Vec3d &pos,
                 Vec3d &linear_vel,
                 Quatd &q,
                 Vec3d &angular_vel,
                 Vec3d &f_ext,
                 Vec3d &tau_ext)
{
    pos << s_in(0), s_in(1), s_in(2);
    linear_vel << s_in(3), s_in(4), s_in(5);
    q << s_in(6), s_in(7), s_in(8), s_in(9);
    angular_vel << s_in(10), s_in(11), s_in(12);
    f_ext << s_in(13), s_in(14), s_in(15);
    tau_ext << s_in(16), s_in(17), s_in(18);
}

#endif