#include "SnapdragonEstimator.h"

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/navigation/NavState.h>

/*****************************************************************************/

using namespace gtsam;

/*****************************************************************************/

NavState get_initial_position()
{
    Rot3 rotation(
        -0.429613609084829,
        0.00211947431436084,
        -0.00972556493216951,
        0.90295795478222 );
    Point3 position( 0.0, 0.0, 0.0 );
    Vector3 velocity( 0.0, 0.0, 0.0 );
    Pose3 pose( rotation, position );

    return NavState( pose, velocity );
}

/*****************************************************************************/

Cal3_S2Stereo::shared_ptr get_cam_calibration()
{
    double fx = 420;
    double fy = 420;
    double skew = 0.0;
    double cx = 320;
    double cy = 240;
    double baseline = 0.2;

    return Cal3_S2Stereo::shared_ptr(
        new Cal3_S2Stereo( fx, fy, skew, cx, cy, baseline ) );
}

/*****************************************************************************/

SnapdragonEstimator::SnapdragonEstimator( double time )
    : Estimator( time, get_initial_position(), get_cam_calibration() )
{
}
