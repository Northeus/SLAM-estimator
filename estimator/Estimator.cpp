#include "Estimator.h"
#include "Measurements.h"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <iostream>
#include <memory>

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
    params.relinearizeThreshold = 0.02;
    params.relinearizeSkip = 1;

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

Estimator::Estimator( double current_time )
    : m_last_cam_time( 4908.79171 ), // TODO update
      m_last_imu_time( current_time ),
      m_isam2( get_isam2() ),
      m_values( get_values( get_prior_state() ) ),
      m_graph( get_graph( get_prior_state() ) ),
      m_preintegration( get_preintegration() ),
      m_previous_state( get_prior_state() ),
      m_estimated_state( get_prior_state() )
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
    // TODO check if things are in correct order (IMU cam)
    // TODO try higher frequency for camera?
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
                new SmartStereoProjectionPoseFactor( noise, params ) );
    }

    m_graph.push_back( m_smart_factors [ measurement.landmark_id ] );
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

    m_isam2.update( m_graph, m_values );
    m_isam2.update();

    m_graph.resize( 0 );
    m_values.clear();
    // TODO all measurements from cam back to graph

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
