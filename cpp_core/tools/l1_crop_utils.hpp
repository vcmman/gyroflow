#pragma once

// Shared glue for the crop-constrained L1 mode between the validate and stabilize CLIs
// (--l1-fit-crop / --l1-auto-box, SMOOTHING_RND §8q): the per-frame required-zoom callback
// for smoothL1CropConstrained, and the geometric per-axis budget derivation (E3).
// Header-only; OpenCV-free (uses core's computeAdaptiveFovs).

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

#include "gyroflow/lens_profile.hpp"
#include "gyroflow/smoothing/l1_optimal.hpp"
#include "gyroflow/stabilization/frame_transform.hpp"
#include "gyroflow/types.hpp"
#include "gyroflow/zooming/adaptive_zoom.hpp"

namespace gyroflow_tools {

// Per-frame required zoom (1/raw_fov, instantaneous inscribed-crop demand before temporal
// smoothing / max-zoom clamp) of a candidate smoothed series, under the tool's exact lens /
// framing / RS setup. `raw` and `lens` are captured by pointer and must outlive the callback.
inline gyroflow::L1ReqZoomFn makeReqZoomFn(std::vector<double> ts_all,
                                           const std::vector<gyroflow::TimeQuat>* raw,
                                           const gyroflow::LensProfile* lens, int width,
                                           int height, double fps, gyroflow::TransformParams tp,
                                           gyroflow::AdaptiveZoomParams az) {
    tp.fov = 1.0;  // demand is measured at unit base FOV (validate convention)
    return [=](const std::vector<gyroflow::TimeQuat>& cand) {
        std::vector<double> rf;
        gyroflow::computeAdaptiveFovs(ts_all, *raw, cand, *lens, width, height, fps, tp, az, &rf);
        std::vector<double> rz(rf.size());
        for (std::size_t i = 0; i < rf.size(); ++i)
            rz[i] = rf[i] > 1e-9 ? 1.0 / rf[i] : 1e9;
        return rz;
    };
}

// E3: derive the per-axis deviation budget (degrees) from max_zoom and the lens geometry —
// for each euler axis, bisect the constant pure-axis offset whose instantaneous required
// zoom hits max_zoom (identity base orientation; single-axis euler offsets coincide with
// axis-angle rotations about X/Y/Z). `scale` de-rates the single-axis budgets for combined
// use (per-axis boxes add up to ~sqrt(3)x a single axis; 1/sqrt(3) ~= 0.577 is the safe
// default, the fit-crop loop cleans up whatever slips through).
inline std::array<double, 3> autoBoxFromGeometry(const gyroflow::LensProfile& lens, int width,
                                                 int height, double fps,
                                                 gyroflow::TransformParams tp,
                                                 gyroflow::AdaptiveZoomParams az,
                                                 double max_zoom, double scale) {
    tp.fov = 1.0;
    constexpr double kPi = 3.14159265358979323846;
    const int N = 8;  // tiny constant series; every frame is identical
    std::vector<double> ts(N);
    std::vector<gyroflow::TimeQuat> raw(N);
    for (int f = 0; f < N; ++f) {
        ts[f] = static_cast<double>(f) * 1000.0 / fps;
        raw[f].timestamp_ms = ts[f];
        raw[f].quat = gyroflow::Quaternion{1.0, 0.0, 0.0, 0.0};
    }
    const gyroflow::Vec3 axes[3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    std::array<double, 3> box{};
    for (int k = 0; k < 3; ++k) {
        double lo = 0.0, hi = kPi / 4.0;  // 45 deg upper bracket
        for (int it = 0; it < 24; ++it) {
            const double mid = 0.5 * (lo + hi);
            std::vector<gyroflow::TimeQuat> cand(N);
            for (int f = 0; f < N; ++f) {
                cand[f].timestamp_ms = ts[f];
                cand[f].quat = gyroflow::Quaternion::fromAxisAngle(axes[k], mid);
            }
            std::vector<double> rf;
            gyroflow::computeAdaptiveFovs(ts, raw, cand, lens, width, height, fps, tp, az, &rf);
            double mx = 0.0;
            for (double v : rf) mx = std::max(mx, v > 1e-9 ? 1.0 / v : 1e9);
            (mx > max_zoom ? hi : lo) = mid;
        }
        box[k] = lo * (180.0 / kPi) * scale;
    }
    return box;
}

} // namespace gyroflow_tools
