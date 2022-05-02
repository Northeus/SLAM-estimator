#include "Estimator.h"
#include "Measurements.h"

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

NavState get_prior_state()
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

ISAM2 get_isam2()
{
    ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 10;

    params.print( "ISAM2 parameters: " );

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

    values.print( "Initial values:" );

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

    graph.print( "Initial graph:" );

    return graph;
}

/*****************************************************************************/

shared_ptr< PreintegratedImuMeasurements > get_preintegration()
{
    imuBias::ConstantBias prior_imu_bias;
    double g = 9.82;

    auto params = PreintegratedCombinedMeasurements::Params::MakeSharedU( g );
    params->accelerometerCovariance = I_3x3 * 0.1;
    params->gyroscopeCovariance = I_3x3 * 0.05;
    params->biasAccCovariance = I_3x3 * 0.02;
    params->biasOmegaCovariance = I_3x3 * 4e-05;
    params->integrationCovariance = I_3x3 * 1e-5;
    params->biasAccOmegaInt = I_6x6 * 1e-3;

    return std::make_shared< PreintegratedImuMeasurements >(
        params,
        prior_imu_bias );
}

/*****************************************************************************/

Cal3_S2Stereo::shared_ptr get_cam_calibration()
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

Estimator::Estimator( double current_time, bool use_cam )
    : m_use_cam( use_cam ),
      m_last_cam_time( 4908.79171 ), // TODO update
      m_last_imu_time( current_time ),
      m_isam2( get_isam2() ),
      m_values( get_values( get_prior_state() ) ),
      m_graph( get_graph( get_prior_state() ) ),
      m_preintegration( get_preintegration() ),
      m_previous_state( get_prior_state() ),
      m_estimated_state( get_prior_state() ),
      m_cam_calibration( get_cam_calibration() )
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
    constexpr double PI = 3.14159265359;
    Rot3 rotation = Rot3::RzRyRx( -PI / 2, 0, -PI / 2 );
    Pose3 body_P_sensor( rotation, Point3() );

    // TODO check if things are in correct order (IMU cam)
    if ( measurement.time > m_last_cam_time )
    {
        m_last_cam_time = measurement.time;
        optimize();
    }

    if ( m_smart_factors.count( measurement.landmark_id ) == 0 )
    {
        auto noise = noiseModel::Isotropic::Sigma( 3, 1.0 );
        /* Required params (we can't have different) */
        SmartProjectionParams params( HESSIAN, ZERO_ON_DEGENERACY );

        m_smart_factors [ measurement.landmark_id ] =
            SmartStereoProjectionPoseFactor::shared_ptr(
                new SmartStereoProjectionPoseFactor(
                    noise,
                    params,
                    body_P_sensor ) );

        m_graph.add( m_smart_factors [ measurement.landmark_id ] );
        m_last_added_landmark.push_back( measurement.landmark_id );
    }
    else
    {
        FactorIndex index = m_landmark_factor_map [ measurement.landmark_id ];
        m_smart_factors_update_map [ index ].insert( X( m_frame ) );
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
    m_frame++;

    cout << "Optimizing frame: " << m_frame << '\n';

    auto preint_imu = dynamic_cast< const PreintegratedImuMeasurements & >(
        *m_preintegration );

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

    m_estimated_state =
        m_preintegration->predict( m_previous_state, m_previous_bias );
    m_values.insert( X( m_frame ), m_estimated_state.pose() );
    m_values.insert( V( m_frame ), m_estimated_state.v() );
    m_values.insert( B( m_frame ), m_previous_bias );

    ISAM2UpdateParams params;
    params.newAffectedKeys = std::move( m_smart_factors_update_map );
    m_smart_factors_update_map = FastMap< FactorIndex, KeySet >();

    auto result = m_isam2.update( m_graph, m_values, params );
    m_isam2.update();

    size_t i = 0;
    for ( const auto &indice : result.newFactorsIndices )
    {
        size_t landmark_id = m_last_added_landmark [ i ];
        m_landmark_factor_map [ landmark_id ] = indice;
        i++;
    }

    m_graph.resize( 0 );
    m_values.clear();
    m_last_added_landmark.clear();

    Values estimate = m_isam2.calculateEstimate();
    m_previous_state = NavState(
        estimate.at< Pose3 >( X( m_frame ) ),
        estimate.at< Vector3 >( V( m_frame ) ) );
    m_previous_bias = estimate.at< imuBias::ConstantBias >( B( m_frame ) );
    m_preintegration->resetIntegrationAndSetBias( m_previous_bias );

    Point3 position = estimate.at< Pose3 >( X( m_frame ) ).translation();
    cout << "Position: (" << position.x() << ", " << position.y() << ", "
         << position.z() << ")\n";
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
