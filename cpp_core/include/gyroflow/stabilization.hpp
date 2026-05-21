#pragma once

#include <vector>

#include "gyroflow/lens_profile.hpp"
#include "gyroflow/types.hpp"

namespace gyroflow {

struct StabilizationSample {
    double timestamp_ms = 0.0;
    Quaternion raw;
    Quaternion smooth;
    Quaternion compensation;
};

struct RollingShutterRowSample {
    int row = 0;
    double timestamp_ms = 0.0;
    Quaternion raw;
    Quaternion smooth;
    Quaternion compensation;
};

std::vector<StabilizationSample> computeStabilizationSamples(
    const std::vector<TimeQuat>& raw_quaternions,
    const std::vector<TimeQuat>& smoothed_quaternions);

std::vector<RollingShutterRowSample> computeRollingShutterRows(
    const std::vector<TimeQuat>& raw_quaternions,
    const std::vector<TimeQuat>& smoothed_quaternions,
    double frame_center_timestamp_ms,
    double frame_readout_time_ms,
    int image_height,
    int row_step = 1);

} // namespace gyroflow
