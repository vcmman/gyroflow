#include "gyroflow/stabilization.hpp"

#include <algorithm>

namespace gyroflow {

std::vector<StabilizationSample> computeStabilizationSamples(
    const std::vector<TimeQuat>& raw_quaternions,
    const std::vector<TimeQuat>& smoothed_quaternions) {
    std::vector<StabilizationSample> out;
    out.reserve(raw_quaternions.size());

    for (const TimeQuat& raw : raw_quaternions) {
        const Quaternion smooth = sampleQuaternion(smoothed_quaternions, raw.timestamp_ms);
        out.push_back({
            raw.timestamp_ms,
            raw.quat,
            smooth,
            smooth.inverse() * raw.quat,
        });
    }

    return out;
}

std::vector<RollingShutterRowSample> computeRollingShutterRows(
    const std::vector<TimeQuat>& raw_quaternions,
    const std::vector<TimeQuat>& smoothed_quaternions,
    double frame_center_timestamp_ms,
    double frame_readout_time_ms,
    int image_height,
    int row_step) {
    std::vector<RollingShutterRowSample> rows;
    if (image_height <= 0) {
        return rows;
    }

    row_step = std::max(1, row_step);
    const double readout = std::max(0.0, frame_readout_time_ms);
    const double row_readout = readout / static_cast<double>(image_height);
    const double start_timestamp = frame_center_timestamp_ms - readout * 0.5;

    rows.reserve(static_cast<std::size_t>((image_height + row_step - 1) / row_step));
    for (int row = 0; row < image_height; row += row_step) {
        const double timestamp = start_timestamp + row_readout * static_cast<double>(row);
        const Quaternion raw = sampleQuaternion(raw_quaternions, timestamp);
        const Quaternion smooth = sampleQuaternion(smoothed_quaternions, timestamp);
        rows.push_back({
            row,
            timestamp,
            raw,
            smooth,
            smooth.inverse() * raw,
        });
    }

    return rows;
}

} // namespace gyroflow
