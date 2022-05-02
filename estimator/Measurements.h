#pragma once

#include <cstddef>

/*****************************************************************************/

struct IMUMeasurement
{
    size_t id;
    double time;
    double Gx, Gy, Gz;
    double Ax, Ay, Az;
};

/*****************************************************************************/

struct StereoMeasurement
{
    double time;
    size_t landmark_id;
    double uL, uR, v;
};

