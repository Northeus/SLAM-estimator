#include "Estimator.h"
#include "Measurements.h"

#include <algorithm>
#include <fstream>
#include <iostream>

/*****************************************************************************/

using namespace std;

/*****************************************************************************/

bool load_imu_measurement( IMUMeasurement &measurement, ifstream &input )
{
    return !!(
        input >> measurement.id >> measurement.time >> measurement.Gx >>
        measurement.Gy >> measurement.Gz >> measurement.Ax >> measurement.Ay >>
        measurement.Az );
}

/*****************************************************************************/

bool load_stereo_measurement( StereoMeasurement &measurement, ifstream &input )
{
    return !!(
        input >> measurement.time >> measurement.landmark_id >>
        measurement.uL >> measurement.uR >> measurement.v );
}

/*****************************************************************************/

int main()
{
    IMUMeasurement imu_measurement;
    StereoMeasurement stereo_measurement;

    ifstream imu_input( "imu.txt" );
    ifstream stereo_input( "projections.csv" );

    if ( !load_imu_measurement( imu_measurement, imu_input ) )
    {
        cout << "Couldn't load IMU file.\n";

        return 1;
    }

    if ( !load_stereo_measurement( stereo_measurement, stereo_input ) )
    {
        cout << "Couldn't load CAM file.\n";

        return 1;
    }

    Estimator estimator( imu_measurement.time );
    load_imu_measurement( imu_measurement, imu_input );

    while ( true )
    {
        if ( imu_measurement.time < stereo_measurement.time )
        {
            estimator.add_measurement( imu_measurement );

            if ( !load_imu_measurement( imu_measurement, imu_input ) )
            {
                break;
            }
        }
        else
        {
            estimator.add_measurement( stereo_measurement );

            if ( !load_stereo_measurement( stereo_measurement, stereo_input ) )
            {
                break;
            }
        }
    }
}
