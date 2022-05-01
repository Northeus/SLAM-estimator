#include <fstream>
#include <iostream> // TODO remove
#include <map>
#include <math.h>
#include <string>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

/*****************************************************************************/

using namespace gtsam;
using namespace std;

using symbol_shorthand::X;

/*****************************************************************************/

Cal3_S2Stereo::shared_ptr get_camera_calibration();

ISAM2 get_isam2();

NonlinearFactorGraph get_graph();

/*****************************************************************************/

int main()
{
    auto camera_calibration = get_camera_calibration();
    auto isam2 = get_isam2();
    auto graph = get_graph();

    Values values;
    map< size_t, SmartStereoProjectionPoseFactor::shared_ptr > smart_factors;

    ifstream input_stereo_camera( "projections.csv" );
    ofstream output_positions( "positions.csv" );
    double current_time = 0.0;
    size_t frame = 0;

    while ( true )
    {
        size_t feature_id;
        double feature_time, feature_uL, feature_uR, feature_v;

        if ( input_stereo_camera >> feature_time >> feature_id >> feature_uL >>
             feature_uR >> feature_v )
        {
            if ( current_time == 0.0 )
            {
                current_time = feature_time;
            }

            if ( feature_time > current_time )
            {
                current_time = feature_time;
                std::cout << "Updating frame: " << feature_time << "s" << '\n';

                if ( frame == 0 )
                {
                    std::cout << "adding pose: " << frame << '\n';
                    values.insert( X( frame ), Pose3() );
                }
                else
                {
                    std::cout << "adding pose: " << frame << '\n';
                    Values best_values = isam2.calculateBestEstimate();
                    Pose3 prev_pose = best_values.at< Pose3 >( X( frame - 1 ) );
                    values.insert( X( frame ), prev_pose );

                    Point3 prev_coords = prev_pose.translation();
                    Rot3 prev_rotation = prev_pose.rotation();
                    output_positions
                        << prev_coords.x() << ' ' << prev_coords.y() << ' '
                        << prev_coords.z() << ' '
                        << prev_rotation.toQuaternion().w() << ' '
                        << prev_rotation.toQuaternion().x() << ' '
                        << prev_rotation.toQuaternion().y() << ' '
                        << prev_rotation.toQuaternion().z() << std::endl;
                    std::cout << "previous coords: " << prev_coords << '\n';
                }

                std::cout << "updating" << '\n';
                isam2.update( graph, values );
                isam2.update();
                values.clear();

                frame++;
            }

            if ( smart_factors.count( feature_id ) == 0 )
            {
                std::cout << "See new landmark: " << feature_id << '\n';

                auto noise = noiseModel::Isotropic::Sigma( 3, 1.0 );
                SmartProjectionParams params( HESSIAN, ZERO_ON_DEGENERACY );

                smart_factors [ feature_id ] =
                    SmartStereoProjectionPoseFactor::shared_ptr(
                        new SmartStereoProjectionPoseFactor( noise, params ) );
                graph.push_back( smart_factors [ feature_id ] );
            }

            std::cout << "Adding measurement for landmark: " << feature_id
                      << ", " << feature_uL << ", " << feature_uR << ", "
                      << feature_v << " [" << frame << "]\n";

            smart_factors [ feature_id ]->add(
                StereoPoint2( feature_uL, feature_uR, feature_v ),
                X( frame ),
                camera_calibration );
        }
        else
        {
            Values current_estimate = isam2.calculateBestEstimate();
            Pose3 prev_pose = values.at< Pose3 >( X( frame - 1 ) );
            values.insert( X( frame ), prev_pose );

            break;
        }
    }

    return 0;
}

/*****************************************************************************/

Cal3_S2Stereo::shared_ptr get_camera_calibration()
{
    // TODO: add config
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

ISAM2 get_isam2()
{
    ISAM2Params params2;
    params2.relinearizeThreshold = 0.05;
    params2.relinearizeSkip = 5;

    return ISAM2( params2 );
}

/*****************************************************************************/

NonlinearFactorGraph get_graph()
{
    NonlinearFactorGraph graph;

    /* Place prior on initial position */
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        ( Vector( 6 ) << Vector3::Constant( 0.001 ),
          Vector3::Constant( 0.001 ) )
            .finished() );

    // TODO: add config
    Rot3 initial_rotation(
        -0.429613609084829,
        0.00211947431436084,
        -0.00972556493216951,
        0.90295795478222 );
    Pose3 initial_pose( initial_rotation, Point3( 0, 0, 0 ) );

    graph.emplace_shared< PriorFactor< Pose3 > >(
        X( 0 ),
        initial_pose,
        priorPoseNoise );

    return graph;
}

