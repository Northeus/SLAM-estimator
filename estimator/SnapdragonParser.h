#pragma once

#include "IMUMeasurement.h"
#include "StereoMeasurement.h"

#include <fstream>
#include <string>

/*****************************************************************************/

class SnapdragonParser
{
public:
    SnapdragonParser(
        const std::string &IMU_filename,
        const std::string &stereo_filename );

    bool read_imu( IMUMeasurement &measurement );
    bool read_stereo( StereoMeasurement &measurement );

private:
    std::fstream m_imu_stream, m_stereo_stream;
};
