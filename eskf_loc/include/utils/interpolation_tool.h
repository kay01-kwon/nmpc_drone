#ifndef INTERPOLATION_TOOL_H
#define INTERPOLATION_TOOL_H

#include "state_def.h"


Vec6d interpolate(const double &t0, 
                  const Vec6d &data0,
                  const double &t1, 
                  const Vec6d &data1,
                  const double &t)
{
    if (t1 == t0)
        return data0;
    double ratio = (t - t0) / (t1 - t0);
    return data0 + ratio * (data1 - data0);
}


#endif