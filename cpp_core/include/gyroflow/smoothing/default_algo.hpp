#pragma once

#include <vector>

#include "gyroflow/types.hpp"

namespace gyroflow {

// Parameters for the default smoothing algorithm. Defaults match
// src/core/smoothing/default_algo.rs (DefaultAlgo::default).
struct DefaultAlgoParams {
    double smoothness = 0.5;       // master velocity-scaling factor (slider 0..1)
    double max_smoothness = 1.0;   // time constant at zero velocity, seconds
    double alpha_0_1s = 0.1;       // time constant at max velocity, seconds
    bool second_pass = true;

    // Per-axis smoothing. When per_axis is true the algorithm smooths the three euler
    // components of each relative rotation independently, each with its own velocity scaling
    // (smoothness_pitch/yaw/roll), instead of a single slerp on the full quaternion. Matches
    // src/core/smoothing/default_algo.rs (per_axis branch). per_axis == false (default) keeps
    // the scalar path => existing golden parity.
    bool per_axis = false;
    double smoothness_pitch = 0.5; // scales euler component [0] (nalgebra roll about X)
    double smoothness_yaw = 0.5;   // scales euler component [1] (nalgebra pitch about Y)
    double smoothness_roll = 0.5;  // scales euler component [2] (nalgebra yaw about Z)

    // Camera diagonal FOV in degrees; fov_ratio = camera_diagonal_fov / 120.
    // Compute as 2*atan(diag/(2*fy))*180/PI (see compute_params.rs::calculate_camera_fovs).
    double camera_diagonal_fov = 120.0;
};

// Phase 1 port of Gyroflow's "Default" smoothing algorithm (non-per-axis path, no
// keyframes, video_speed == 1). Input is the raw/org attitude series; output is the
// smoothed *orientation* series (NOT pre-multiplied compensation), sampled at the same
// timestamps. duration_ms is the gyro stream duration (last_ts - first_ts works).
std::vector<TimeQuat> smoothDefault(const std::vector<TimeQuat>& quats, double duration_ms,
                                    const DefaultAlgoParams& params);

} // namespace gyroflow
