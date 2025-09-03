#ifndef INTERPOLATE_TOOL_H
#define INTERPOLATE_TOOL_H
#include "state_def.h"

Vec4d interpolate_vec4(const double &t0,
                       const Vec4d &v0,
                       const double &t1,
                       const Vec4d &v1,
                       const double &t)
{

    double alpha;
    alpha = (t - t0) / (t1 - t0);
    return (1 - alpha) * v0 + alpha * v1;
}


#endif