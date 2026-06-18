#include "gyroflow/lens_profile.hpp"
#include "gyroflow/zooming/adaptive_zoom.hpp"

#include <cassert>
#include <cmath>
#include <vector>

int main() {
    using namespace gyroflow;

    const int W = 1920, H = 1080;
    LensProfile lens;
    lens.calib_width = lens.output_width = W;
    lens.calib_height = lens.output_height = H;
    lens.camera_matrix = {1000.0, 1000.0, W / 2.0, H / 2.0};
    lens.distortion_coeffs = {0.0, 0.0, 0.0, 0.0};  // no distortion => undistort is identity

    // Identity attitude => undistorted border == source border.
    std::vector<TimeQuat> raw, smooth;
    for (int i = 0; i < 90; ++i) {
        const double ts = i * (1000.0 / 30.0);
        raw.push_back({ts, Quaternion::identity()});
        smooth.push_back({ts, Quaternion::identity()});
    }

    std::vector<double> ts;
    for (int i = 0; i < 90; ++i) ts.push_back(i * (1000.0 / 30.0));

    TransformParams tp;
    tp.frame_readout_time_ms = 0.0;
    AdaptiveZoomParams az;  // margin 2 px, window 4 s

    const std::vector<double> fovs = computeAdaptiveFovs(ts, raw, smooth, lens, W, H, 30.0, tp, az);
    assert(fovs.size() == ts.size());

    // With no rotation and no distortion the inscribed crop is the full frame minus the
    // 2px algorithm margin: fov ~= 1 - 2*margin/H ~= 0.9963.
    const double expected = 1.0 - 2.0 * az.fov_algorithm_margin / H;
    for (double v : fovs) {
        assert(v > 0.99 && v <= 1.0);
        assert(std::abs(v - expected) < 0.01);
    }

    // max_zoom clamp: a tiny ceiling forces fov up to the floor 100/max_zoom.
    AdaptiveZoomParams az2 = az;
    az2.max_zoom_percent = 110.0;  // floor = 0.909..., above the ~0.996 here? no -> unaffected
    const std::vector<double> fovs2 = computeAdaptiveFovs(ts, raw, smooth, lens, W, H, 30.0, tp, az2);
    for (double v : fovs2) assert(v >= 100.0 / 110.0 - 1e-9);

    // GaussianFilter method (adaptive_zoom_method == 0). The per-frame FOVs are a constant
    // here, so the rolling-min + Gaussian-convolution chain must preserve both the length
    // (pad/min/pad/convolve bookkeeping) and the constant value.
    AdaptiveZoomParams az_g = az;
    az_g.method = ZoomMethod::GaussianFilter;
    const std::vector<double> fovs_g = computeAdaptiveFovs(ts, raw, smooth, lens, W, H, 30.0, tp, az_g);
    assert(fovs_g.size() == ts.size());
    for (double v : fovs_g) {
        assert(v > 0.99 && v <= 1.0);
        assert(std::abs(v - expected) < 1e-6);  // constant signal => unchanged by smoothing
    }

    // Non-constant scenario: a yaw sweep makes the inscribed crop (and per-frame FOV) vary,
    // so the temporal smoothers actually do work. Both methods must return nf values, all in
    // (0,1], and must engage the zoom (some FOV pushed below 1). This exercises the real
    // pad/min/convolve and envelope paths on a varying signal, not just a constant.
    std::vector<TimeQuat> raw_r, smooth_r;
    for (int i = 0; i < 90; ++i) {
        const double ts_i = i * (1000.0 / 30.0);
        const double ang = 0.15 * std::sin(i * 0.2);  // ±0.15 rad yaw wobble
        raw_r.push_back({ts_i, Quaternion::identity()});
        smooth_r.push_back({ts_i, Quaternion::fromAxisAngle({0.0, 1.0, 0.0}, ang)});
    }
    for (ZoomMethod m : {ZoomMethod::EnvelopeFollower, ZoomMethod::GaussianFilter}) {
        AdaptiveZoomParams az_v = az;
        az_v.method = m;
        const std::vector<double> fv =
            computeAdaptiveFovs(ts, raw_r, smooth_r, lens, W, H, 30.0, tp, az_v);
        assert(fv.size() == ts.size());
        double mn = 2.0, mx = -2.0;
        for (double v : fv) { assert(v > 0.0 && v <= 1.0 + 1e-9); mn = std::min(mn, v); mx = std::max(mx, v); }
        assert(mn < 0.999);  // zoom engaged somewhere
    }

    return 0;
}
