#pragma once

#include <utility>
#include <vector>

#include "gyroflow/lens_profile.hpp"
#include "gyroflow/stabilization/frame_transform.hpp"
#include "gyroflow/telemetry.hpp"
#include "gyroflow/types.hpp"

namespace gyroflow {

// Adaptive (dynamic) zoom. Ports src/core/zooming/{fov_iterative,zoom_dynamic}.rs.
//
// For each frame it forward-maps a polygon of source-border points into stabilized output
// space, inscribes the largest centered rectangle of the output aspect ratio (the largest
// crop with no black borders), and converts that to a per-frame FOV. The per-frame FOVs
// are then smoothed over time with the EnvelopeFollower (adaptive_zoom_method == 1, the
// DJI default) so the crop changes smoothly, and clamped by max_zoom.
struct AdaptiveZoomParams {
    double window_s = 4.0;             // adaptive_zoom_window
    double max_zoom_percent = 130.0;   // <= 50 disables the clamp
    double fov_algorithm_margin = 2.0; // pixels trimmed from the source border ring
    std::pair<double, double> center_offset = {0.0, 0.0};  // adaptive_zoom_center_offset
};

// Returns one FOV multiplier per entry in frame_timestamps_ms (same order). FOV < 1 zooms
// in. Feed each value into TransformParams::fov for the corresponding frame.
std::vector<double> computeAdaptiveFovs(const std::vector<double>& frame_timestamps_ms,
                                        const std::vector<TimeQuat>& raw,
                                        const std::vector<TimeQuat>& smoothed,
                                        const LensProfile& lens, int width, int height,
                                        double fps, const TransformParams& tp,
                                        const AdaptiveZoomParams& az);

} // namespace gyroflow
