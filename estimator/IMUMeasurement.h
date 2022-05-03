#pragma once

#include <cstddef>

/*****************************************************************************/

struct IMUMeasurement
{
    size_t id{ 0 };
    double time{ 0 };
    double Ax{ 0 }, Ay{ 0 }, Az{ 0 };
    double Gx{ 0 }, Gy{ 0 }, Gz{ 0 };
};
