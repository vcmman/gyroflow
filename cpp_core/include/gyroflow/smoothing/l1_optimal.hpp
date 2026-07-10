#pragma once

#include <array>
#include <functional>
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

    // --- Real-time receding-horizon mode (SMOOTHING_RND §8o) ------------------------------
    // < 0 (default) = offline: one global solve over the whole clip.
    // >= 0 = in-camera: repeatedly solve a small window [past_s | commit block | look_ahead_s]
    // with the already-committed samples pinned (zero-width box) for continuity, commit
    // commit_block frames, slide. Each window solve is small -> rt_iterations converge fast;
    // total cost O(nf * rt_iterations), streamable with a look_ahead_s future buffer.
    double look_ahead_s = -1.0;      // future buffer (e.g. 1.0 for the in-camera 1 s budget)
    double past_s = 2.0;             // past context kept in the window
    int commit_block = 15;           // frames committed per solve (latency granularity)
    int rt_iterations = 4000;        // ADMM iterations per window solve (jerk-clean; ~11x realtime)
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

// --- Crop-constrained L1 (SMOOTHING_RND §8p/§8q, "E4") ------------------------------------
// The static per-axis angle box cannot be tight in the zoom domain (per-axis boxes combine to
// sqrt(3)x the single-axis deviation; the angle->zoom slope varies ~3.5x with direction/lens
// position), so a box that survives the worst frame over-constrains all others (§8p). This
// variant puts the constraint where the black border lives: an outer constraint-generation
// loop around the same per-channel ADMM. Each round solves with the current per-frame
// per-axis boxes, asks `reqzoom_fn` for the per-frame required zoom (1/raw_fov, instantaneous
// inscribed-crop demand BEFORE temporal smoothing/clamp) of the candidate path, and shrinks
// the boxes of violating frames (+-2-frame halo) toward the deviation that would meet
// `max_zoom` (with a small margin), leaving all other frames at the full budget. Zero borders
// by construction at convergence; smoothness is paid only where the budget binds.
// The callback keeps this module lens-free; build it from computeAdaptiveFovs (see
// tools/l1_crop_utils.hpp). Offline only (params.look_ahead_s is ignored).
using L1ReqZoomFn = std::function<std::vector<double>(const std::vector<TimeQuat>&)>;

struct L1CropReport {
    int outer_iters = 0;                       // solve rounds used (1 = never violated)
    int breach_before = 0, breach_after = 0;   // frames with required zoom > max_zoom
    double max_reqz_before = 0.0, max_reqz_after = 0.0;
};

std::vector<TimeQuat> smoothL1CropConstrained(const std::vector<TimeQuat>& quats, double fps,
                                              const L1OptimalParams& params,
                                              double max_zoom,  // absolute, e.g. 1.30
                                              const L1ReqZoomFn& reqzoom_fn,
                                              L1CropReport* report = nullptr);

} // namespace gyroflow
