#pragma once

#include <utility>
#include <vector>

#include "gyroflow/lens_profile.hpp"
#include "gyroflow/stabilization/frame_transform.hpp"
#include "gyroflow/telemetry.hpp"
#include "gyroflow/types.hpp"

namespace gyroflow {

// Temporal smoothing method for the per-frame FOVs (Gyroflow `adaptive_zoom_method`).
// Both match zoom_dynamic::compute's non-keyframed (static-window) branch.
enum class ZoomMethod {
    GaussianFilter = 0,    // rolling-min over the window, then Gaussian convolution
    EnvelopeFollower = 1,  // two-pass min-tracking envelope follower (Gyroflow default)
};

// Adaptive (dynamic) zoom. Ports src/core/zooming/{fov_iterative,zoom_dynamic}.rs.
//
// For each frame it forward-maps a polygon of source-border points into stabilized output
// space, inscribes the largest centered rectangle of the output aspect ratio (the largest
// crop with no black borders), and converts that to a per-frame FOV. The per-frame FOVs
// are then smoothed over time (so the crop changes smoothly) by the selected `method` and
// clamped by max_zoom. This is the dynamic-zoom path (Gyroflow `adaptive_zoom_window > 0`);
// the keyframed-window sub-branch (zooming-speed keyframes / video-speed) is not ported as
// the headless bridge carries no keyframes.
struct AdaptiveZoomParams {
    double window_s = 4.0;             // adaptive_zoom_window
    double max_zoom_percent = 130.0;   // <= 50 disables the clamp
    double fov_algorithm_margin = 2.0; // pixels trimmed from the source border ring
    std::pair<double, double> center_offset = {0.0, 0.0};  // adaptive_zoom_center_offset
    ZoomMethod method = ZoomMethod::EnvelopeFollower;       // adaptive_zoom_method (default 1)
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
