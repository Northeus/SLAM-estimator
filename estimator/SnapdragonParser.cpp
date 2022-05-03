#include "SnapdragonParser.h"
#include "IMUMeasurement.h"
#include "StereoMeasurement.h"

#include <fstream>
#include <string>

/*****************************************************************************/

using namespace std;

/*****************************************************************************/

SnapdragonParser::SnapdragonParser(
    const string &IMU_filename,
    const string &stereo_filename )
    : m_imu_stream( IMU_filename ),
      m_stereo_stream( stereo_filename )
{
}

/*****************************************************************************/

bool SnapdragonParser::read_imu( IMUMeasurement &measurement )
{
    return !!(
        m_imu_stream >> measurement.id >> measurement.time >> measurement.Gx >>
        measurement.Gy >> measurement.Gz >> measurement.Ax >> measurement.Ay >>
        measurement.Az );
}

/*****************************************************************************/

bool SnapdragonParser::read_stereo( StereoMeasurement &measurement )
{
    return !!(
        m_stereo_stream >> measurement.time >> measurement.landmark_id >>
        measurement.uL >> measurement.uR >> measurement.v );
}
