#pragma once

#include <cstddef>

/*****************************************************************************/

struct StereoMeasurement
{
    double time{ 0 };
    size_t landmark_id{ 0 };
    double uL{ 0 }, uR{ 0 }, v{ 0 };
};
