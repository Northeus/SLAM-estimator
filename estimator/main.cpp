#include "IMUMeasurement.h"
#include "SnapdragonEstimator.h"
#include "SnapdragonParser.h"
#include "StereoMeasurement.h"

#include <string>

/*****************************************************************************/

int main()
{
    IMUMeasurement imu_measurement;
    StereoMeasurement stereo_measurement;

    SnapdragonParser parser( "imu.txt", "projections.csv" );

    parser.read_imu( imu_measurement );
    double initial_time = imu_measurement.time;
    SnapdragonEstimator estimator( initial_time );

    bool has_imu_measurement = parser.read_imu( imu_measurement );
    bool has_stereo_measurement = parser.read_stereo( stereo_measurement );

    while ( true )
    {
        if ( !has_stereo_measurement )
        {
            break;
        }

        if ( has_imu_measurement &&
             imu_measurement.time < stereo_measurement.time )
        {
            estimator.add_measurement( imu_measurement );
            has_imu_measurement = parser.read_imu( imu_measurement );
        }

        if ( has_stereo_measurement &&
             stereo_measurement.time < imu_measurement.time )
        {
            estimator.add_measurement( stereo_measurement );
            has_stereo_measurement = parser.read_stereo( stereo_measurement );
        }
    }

    estimator.export_last_estimate( "positions.csv" );

    return 0;
}
