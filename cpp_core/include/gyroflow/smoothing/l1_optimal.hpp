#pragma once

#include <array>
#include <vector>

#include "gyroflow/types.hpp"

namespace gyroflow {

// L1-optimal camera path (Grundmann et al. 2011), experimental beyond-Gyroflow smoothing.
// See cpp_core/JOLT_RND.md. Minimizes the L1 norm of the 1st/2nd/3rd derivatives of the
// camera path (=> static / linear / parabolic segments) subject to a per-axis crop box around
// the raw path (the crop-aware part). Solved per euler channel by ADMM (dependency-free).
struct L1OptimalParams {
    double w1 = 10.0;   // L1 weight on 1st derivative (velocity)
    double w2 = 1.0;    // ...2nd derivative (acceleration)
    double w3 = 100.0;  // ...3rd derivative (jerk)
    // crop budget: |smoothed - raw| <= this, PER euler channel (X,Y,Z) in degrees. A single
    // value over-constrains the channel that pans most (yaw), so this is per-axis.
    std::array<double, 3> max_deviation_deg = {6.0, 6.0, 6.0};
    int iterations = 2000;           // ADMM iterations
    double rho = 1.0;                // ADMM penalty
    double over_relax = 1.8;         // ADMM over-relaxation (1=off, 1.5-1.8 accelerates)
};

// Per-euler-channel max |a - b| (deg) sampled at frame cadence (unwrapped) — used to set the
// L1 crop box equal to the budget another smoother (e.g. default_algo) actually used, for an
// apples-to-apples same-crop comparison.
std::array<double, 3> frameEulerMaxDeviationDeg(const std::vector<TimeQuat>& a,
                                                const std::vector<TimeQuat>& b, double fps);

// Returns a FRAME-RATE smoothed orientation series (sampled at ts = f*1000/fps): the raw
// attitude is sampled at frame cadence, smoothed per euler channel, and recomposed. Downstream
// (adaptive zoom, render) samples it the same way it samples the default smoother's output.
std::vector<TimeQuat> smoothL1Optimal(const std::vector<TimeQuat>& quats, double fps,
                                      const L1OptimalParams& params);

} // namespace gyroflow
