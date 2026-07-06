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

    // --- Direction-Consistency-Ratio (DCR) gating -----------------------------------------
    // Not part of upstream Gyroflow. The stock velocity-dampening loosens smoothing whenever
    // the angular *speed* (an abs magnitude) is high, so it cannot tell a sustained pan
    // (consistent direction — should loosen and follow) from a reciprocating shake such as a
    // running "bob" (alternating direction — should stay strongly smoothed). DCR restores that
    // distinction: over a sliding window it measures direction consistency
    //     DCR = |mean(omega)| / mean(|omega|)  in [0,1]
    // (per euler axis in the per_axis path; the 3-D rotation-vector analogue
    //  ||sum(omega_vec)|| / sum(||omega_vec||) in the scalar path) and multiplies the
    // normalized velocity ratio by DCR^dcr_power. So the filter only loosens when the motion
    // is both fast AND directionally consistent; oscillation (DCR->0) keeps full smoothing.
    // dcr == false (default) leaves both paths bit-identical => golden parity preserved.
    bool dcr = false;
    double dcr_window_s = 0.5;  // sliding window length (seconds) for the consistency estimate
    double dcr_power = 1.0;     // gate = DCR^dcr_power; >1 sharpens, <1 softens the gating

    // --- Finite look-ahead (in-camera realizability) ---------------------------------------
    // The forward/backward slerp EMA is zero-phase only with unlimited future. A real-time
    // in-camera implementation buffers just this many seconds of future to run the backward
    // pass. When > 0, the (acausal) backward pass of the main adaptive smoothing is limited to
    // a window of look_ahead_s of future (seeded at the newest buffered frame); the forward
    // pass stays causal over the full past. 0 (default) = offline / unlimited => bit-identical.
    // Only the scalar path is affected; the velocity/distance sub-smoothings (tau=0.1 s) and
    // their normalisation maxes are left global (their finite-look-ahead error is negligible at
    // 1 s >> 0.3 s). See cpp_core/SMOOTHING_RND.md §7.
    double look_ahead_s = 0.0;
};

// Phase 1 port of Gyroflow's "Default" smoothing algorithm (non-per-axis path, no
// keyframes, video_speed == 1). Input is the raw/org attitude series; output is the
// smoothed *orientation* series (NOT pre-multiplied compensation), sampled at the same
// timestamps. duration_ms is the gyro stream duration (last_ts - first_ts works).
std::vector<TimeQuat> smoothDefault(const std::vector<TimeQuat>& quats, double duration_ms,
                                    const DefaultAlgoParams& params);

} // namespace gyroflow
