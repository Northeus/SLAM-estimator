#include "Estimator.h"
#include "IMUMeasurement.h"
#include "StereoMeasurement.h"

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

/*****************************************************************************/

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

/*****************************************************************************/

ISAM2 get_isam2()
{
    ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1; // Required for smart factors to work properly
    params.cacheLinearizedFactors = false; // Don't cache smart factors

    return ISAM2( params );
}

/*****************************************************************************/

Values get_values( const NavState &prior_state )
{
    imuBias::ConstantBias prior_imu_bias;

    Values values;
    values.insert( X( 0 ), prior_state.pose() );
    values.insert( V( 0 ), prior_state.v() );
    values.insert( B( 0 ), prior_imu_bias );

    return values;
}

/*****************************************************************************/

NonlinearFactorGraph get_graph( const NavState &prior_state )
{
    imuBias::ConstantBias prior_imu_bias;

    /* Place prior on initial position */
    auto prior_pose_noise = noiseModel::Diagonal::Sigmas(
        ( Vector( 6 ) << 0.02, 0.02, 0.02, 0.02, 0.02, 0.02 ).finished() );
    auto prior_velocity_noise = noiseModel::Isotropic::Sigma( 3, 0.1 );
    auto prior_imu_bias_noise = noiseModel::Isotropic::Sigma( 6, 0.01 );

    NonlinearFactorGraph graph;
    graph.addPrior( X( 0 ), prior_state.pose(), prior_pose_noise );
    graph.addPrior( V( 0 ), prior_state.v(), prior_velocity_noise );
    graph.addPrior( B( 0 ), prior_imu_bias, prior_imu_bias_noise );

    return graph;
}

/*****************************************************************************/

Estimator::Estimator(
    double current_time,
    NavState prior_state,
    Cal3_S2Stereo::shared_ptr cam_calibration,
    shared_ptr< PreintegratedImuMeasurements > preintegration )
    : m_last_cam_time( current_time ),
      m_last_imu_time( current_time ),
      m_isam2( get_isam2() ),
      m_values( get_values( prior_state ) ),
      m_graph( get_graph( prior_state ) ),
      m_preintegration( preintegration ),
      m_previous_state( prior_state ),
      m_estimated_state( prior_state ),
      m_cam_calibration( cam_calibration )
{
}

/*****************************************************************************/

void Estimator::add_measurement( const IMUMeasurement &measurement )
{
    double dt = measurement.time - m_last_imu_time;
    m_last_imu_time = measurement.time;

    Vector3 acceleration( measurement.Ax, measurement.Ay, measurement.Az );
    Vector3 gyroscope( measurement.Gx, measurement.Gy, measurement.Gz );

    m_preintegration->integrateMeasurement( acceleration, gyroscope, dt );
}

/*****************************************************************************/

void Estimator::add_measurement( const StereoMeasurement &measurement )
{
    /* Transform camera frame to world frame */
    constexpr double PI = 3.14159265359;
    Rot3 rotation = Rot3::RzRyRx( -PI / 2, 0, -PI / 2 );
    Pose3 body_P_sensor( rotation, Point3() );

    if ( measurement.time > m_last_cam_time )
    {
        m_last_cam_time = measurement.time;
        optimize();
    }

    if ( m_smart_factors.count( measurement.landmark_id ) == 0 )
    {
        auto noise = noiseModel::Isotropic::Sigma( 3, 1.0 );
        SmartProjectionParams params( HESSIAN, ZERO_ON_DEGENERACY );

        m_smart_factors [ measurement.landmark_id ] =
            SmartStereoProjectionPoseFactor::shared_ptr(
                new SmartStereoProjectionPoseFactor(
                    noise,
                    params,
                    body_P_sensor ) );

        m_new_factor_landmark_map [ m_graph.size() ] = measurement.landmark_id;
        m_graph.push_back( m_smart_factors [ measurement.landmark_id ] );
    }
    else
    {
        FactorIndex index = m_landmark_factor_map [ measurement.landmark_id ];
        m_affected_keys [ index ].insert( X( m_frame ) );
    }

    m_smart_factors [ measurement.landmark_id ]->add(
        StereoPoint2( measurement.uL, measurement.uR, measurement.v ),
        X( m_frame ),
        m_cam_calibration );
}

/*****************************************************************************/

void Estimator::optimize()
{
    auto bias_noise_model = noiseModel::Isotropic::Sigma( 6, 0.001 );
    auto preint_imu = dynamic_cast< const PreintegratedImuMeasurements & >(
        *m_preintegration );

    m_frame++;

    cout << "Optimizing frame: " << m_frame << '\n';

    ImuFactor imu_factor(
        X( m_frame - 1 ),
        V( m_frame - 1 ),
        X( m_frame ),
        V( m_frame ),
        B( m_frame - 1 ),
        preint_imu );
    m_graph.add( imu_factor );

    imuBias::ConstantBias zero_bias(
        Vector3( 0.0, 0.0, 0.0 ),
        Vector3( 0.0, 0.0, 0.0 ) );
    m_graph.add( BetweenFactor< imuBias::ConstantBias >(
        B( m_frame - 1 ),
        B( m_frame ),
        zero_bias,
        bias_noise_model ) );

    /* Estimate new position according to the IMU */
    m_estimated_state =
        m_preintegration->predict( m_previous_state, m_previous_bias );
    m_values.insert( X( m_frame ), m_estimated_state.pose() );
    m_values.insert( V( m_frame ), m_estimated_state.v() );
    m_values.insert( B( m_frame ), m_previous_bias );

    /* Optimize */
    ISAM2UpdateParams params;
    params.newAffectedKeys = std::move( m_affected_keys );
    m_affected_keys = FastMap< FactorIndex, KeySet >();
    auto result = m_isam2.update( m_graph, m_values, params );
    m_isam2.update();

    /* Map newly created factors to the landmarks ids */
    for ( auto const &[ new_factor_index, landmark_id ] :
          m_new_factor_landmark_map )
    {
        m_landmark_factor_map [ landmark_id ] =
            result.newFactorsIndices.at( new_factor_index );
    }

    /* Clear factors and variables */
    m_affected_keys = FastMap< FactorIndex, KeySet >();
    m_new_factor_landmark_map.clear();
    m_graph.resize( 0 );
    m_values.clear();

    /* Clear preintegration */
    Values estimate = m_isam2.calculateEstimate();
    m_previous_state = NavState(
        estimate.at< Pose3 >( X( m_frame ) ),
        estimate.at< Vector3 >( V( m_frame ) ) );
    m_previous_bias = estimate.at< imuBias::ConstantBias >( B( m_frame ) );
    m_preintegration->resetIntegrationAndSetBias( m_previous_bias );
}

/*****************************************************************************/

void Estimator::export_last_estimate( const string &filename )
{
    ofstream output( filename );
    Values last_estimate = m_isam2.calculateEstimate();

    for ( size_t i = 0; i <= m_frame; i++ )
    {
        Pose3 pose = last_estimate.at< Pose3 >( X( i ) );
        auto position = pose.translation();
        auto quat = pose.rotation().toQuaternion();

        output << position.x() << ' ' << position.y() << ' ' << position.z()
               << ' ' << quat.w() << ' ' << quat.x() << ' ' << quat.y() << ' '
               << quat.z() << endl;
    }
}
