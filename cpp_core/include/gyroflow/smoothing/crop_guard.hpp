#pragma once

#include <functional>
#include <vector>

#include "gyroflow/types.hpp"

namespace gyroflow {

// --- Crop-budget guard (zero black borders for the EMA family, SMOOTHING_RND §8r) -------
//
// Post-pass over any default_algo output (plain EMA, DCR, clamps). Black borders appear
// wherever the per-frame required zoom (instantaneous inscribed-crop demand) exceeds the
// max_zoom clamp; the bounded-deviation experiments showed a static ANGLE budget can never be
// tight in the ZOOM domain (per-axis combination, direction-dependent angle->zoom slope). This
// guard therefore works directly in the zoom domain:
//
//   1. measure the per-frame demand of the smoothed path (callback, keeps this module
//      lens-free — build it from computeAdaptiveFovs, see tools/crop_fit_utils.hpp);
//   2. convert to a per-frame gain g = (target - d_ref) / (envelope(demand) - d_ref) in
//      [0,1] — the slerp fraction toward a fundamental-only reference (zero-phase EMA of raw)
//      that brings the frame inside a margined max_zoom;
//   3. gain varies at ENVELOPE speed (centered sliding-window max over window_s + zero-phase
//      EMA + per-frame peak-hold), not per sample — the §8l cleanliness law: slow gain =
//      compressor (clean waveform), per-sample gain = clipper (harmonic distortion);
//   4. one verification round re-measures the result and tightens residual frames locally
//      (the demand model is only locally linear).
//
// Mixing toward the fundamental reference — not raw — keeps raw's impact harmonics out of the
// output while saturated (§8n-2 lesson); ref_tau_s is small so the reference itself stays
// within budget during violence (§8j-4 lesson).
//
// Naming note: the envelope->slow-gain->slerp skeleton is the same compressor pattern as the
// deviation-AGC *smoothing mode* experiment (§8n, claude/deviation-agc branch), because §8l
// dictates that shape for any clean-waveform bounding. But this is NOT another AGC smoothing
// mode: the control variable is crop DEMAND (not angle), the budget is a hard geometric
// constraint from max_zoom (not an aesthetic knob), and it is a guard applicable on top of
// any smoother.
struct CropGuardParams {
    double window_s = 0.8;     // centered window for the demand max-envelope (seconds)
    double env_tau_s = 0.10;   // zero-phase EMA tau smoothing the envelope (seconds)
    double ref_tau_s = 0.03;   // fundamental-reference EMA tau (seconds; small => ref ~ raw)
    double margin = 0.97;      // target = 1 + (max_zoom-1)*margin (headroom for model error)
    int outer_iters = 3;       // demand re-measure / local-tighten rounds
};

struct CropGuardReport {
    int outer_iters = 0;
    int breach_before = 0, breach_after = 0;   // frames with demand > max_zoom
    double max_reqz_before = 0.0, max_reqz_after = 0.0;
    double min_gain = 1.0;                     // deepest compression applied
    int gained_frames = 0;                     // frames with gain < 1
};

// Per-frame required zoom (1/raw_fov) of a candidate smoothed series, frame cadence.
using CropDemandFn = std::function<std::vector<double>(const std::vector<TimeQuat>&)>;

// Returns the guarded series (same timestamps as `smoothed`; `raw` must be the same series
// the smoother consumed). No-op (bit-identical copy) wherever the budget never binds.
std::vector<TimeQuat> applyCropBudgetGuard(const std::vector<TimeQuat>& raw,
                                         const std::vector<TimeQuat>& smoothed, double fps,
                                         double max_zoom,  // absolute, e.g. 1.30
                                         const CropDemandFn& demand_fn,
                                         const CropGuardParams& params,
                                         CropGuardReport* report = nullptr);

} // namespace gyroflow
