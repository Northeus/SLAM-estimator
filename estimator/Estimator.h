#pragma once

#include "Measurements.h"

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <fstream>
#include <map>
#include <memory>

/*****************************************************************************/

class Estimator
{
public:
    Estimator( double current_time, bool use_cam );

    void add_measurement( const IMUMeasurement &measurement );

    void add_measurement( const StereoMeasurement &measurement );

private:
    void optimize();

    bool m_use_cam;
    size_t m_frame = 0;
    double m_last_cam_time;
    double m_last_imu_time;
    ofstream m_output_positions;
    gtsam::ISAM2 m_isam2;
    gtsam::Values m_values;
    gtsam::NonlinearFactorGraph m_graph;
    std::shared_ptr< gtsam::PreintegratedImuMeasurements > m_preintegration;
    gtsam::NavState m_previous_state, m_estimated_state;
    gtsam::imuBias::ConstantBias m_previous_bias;
    std::map< size_t, gtsam::SmartStereoProjectionPoseFactor::shared_ptr >
        m_smart_factors;
    gtsam::Cal3_S2Stereo::shared_ptr m_cam_calibration;
};
