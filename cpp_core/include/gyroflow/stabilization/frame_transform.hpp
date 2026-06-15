#pragma once

#include <array>
#include <vector>

#include "gyroflow/lens_profile.hpp"
#include "gyroflow/telemetry.hpp"
#include "gyroflow/types.hpp"

namespace gyroflow {

// The per-frame stabilization transform: one 3x3 matrix per scanline (for rolling-shutter
// correction) plus the source-camera intrinsics and distortion coefficients consumed by
// the undistort kernel.
//
// Each matrix is i_r = inverse(new_K * R), stored row-major (m[row*3 + col]), exactly as
// produced by src/core/stabilization/frame_transform.rs.
struct FrameTransform {
    std::vector<std::array<double, 9>> matrices;  // size == 1 (no RS) or readout-axis length
    bool horizontal_readout = false;              // index matrices by x (true) or y (false)

    std::array<double, 2> f{0, 0};  // source focal length in pixels (fx, fy)
    std::array<double, 2> c{0, 0};  // source principal point in pixels (cx, cy)
    std::array<double, 4> k{0, 0, 0, 0};  // fisheye distortion coefficients

    int width = 0;          // source frame dimensions
    int height = 0;
    int output_width = 0;
    int output_height = 0;
};

struct TransformParams {
    // Output framing. fov == 1.0 is no scaling; fov < 1.0 zooms in (crops the moving
    // black borders, at the cost of resolution); fov > 1.0 zooms out.
    double fov = 1.0;

    double frame_readout_time_ms = 0.0;  // 0 disables rolling-shutter correction
    ReadoutDirection frame_readout_direction = ReadoutDirection::TopToBottom;
};

// Build the transform for the frame whose center timestamp is frame_center_ts_ms.
// rawQuats is the org attitude series; smoothedQuats is the smoothed orientation series
// (both sorted by timestamp). The smoothed orientation is sampled at the frame center; the
// raw orientation is sampled per scanline to model rolling shutter.
FrameTransform computeFrameTransform(double frame_center_ts_ms,
                                     const std::vector<TimeQuat>& rawQuats,
                                     const std::vector<TimeQuat>& smoothedQuats,
                                     const LensProfile& lens, int width, int height,
                                     const TransformParams& params);

} // namespace gyroflow
