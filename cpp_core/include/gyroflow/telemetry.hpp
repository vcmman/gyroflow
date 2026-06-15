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

// Rolling-shutter readout direction. Mirrors gyroflow-core's ReadoutDirection
// (src/core/stabilization_params.rs).
enum class ReadoutDirection {
    TopToBottom,
    BottomToTop,
    LeftToRight,
    RightToLeft,
};

inline bool isHorizontal(ReadoutDirection d) {
    return d == ReadoutDirection::LeftToRight || d == ReadoutDirection::RightToLeft;
}

struct FileMetadata {
    std::string detected_source;
    std::vector<TimeIMU> raw_imu;

    // Fused (org / raw) attitude quaternions. For DJI this comes straight from the
    // device, so no IMU integration is required.
    std::vector<TimeQuat> quaternions;

    std::optional<LensProfile> lens_profile;

    double fps = 0.0;
    int width = 0;
    int height = 0;

    double frame_readout_time_ms = 0.0;
    ReadoutDirection frame_readout_direction = ReadoutDirection::TopToBottom;
};

} // namespace gyroflow
