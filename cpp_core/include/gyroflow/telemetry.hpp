#pragma once

#include <optional>
#include <string>
#include <vector>

#include "gyroflow/lens_profile.hpp"
#include "gyroflow/types.hpp"

namespace gyroflow {

struct TimeIMU {
    double timestamp_ms = 0.0;
    std::optional<Vec3> gyro;
    std::optional<Vec3> accel;
    std::optional<Vec3> magnetometer;
};

struct FileMetadata {
    std::string detected_source;
    std::vector<TimeIMU> raw_imu;
    std::vector<TimeQuat> quaternions;
    std::optional<LensProfile> lens_profile;
    double frame_readout_time_ms = 0.0;
};

} // namespace gyroflow
