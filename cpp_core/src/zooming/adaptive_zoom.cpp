#include "gyroflow/zooming/adaptive_zoom.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

#include "gyroflow/distortion/opencv_fisheye.hpp"
#include "gyroflow/mat3.hpp"

namespace gyroflow {

namespace {

using Pt = std::pair<float, float>;

constexpr double kInvalid = -1000000.0;

// Points placed clockwise around a rectangle of size (w,h), trimmed by `margin`.
// Mirrors FovIterative::points_around_rect with w_div = h_div = 31.
std::vector<Pt> pointsAroundRect(double w, double h, double margin, int w_div = 31,
                                 int h_div = 31) {
    w -= margin * 2.0;
    h -= margin * 2.0;
    const int wcnt = std::max(2, w_div) - 1;
    const int hcnt = std::max(2, h_div) - 1;
    const double wstep = w / wcnt;
    const double hstep = h / hcnt;

    std::vector<Pt> p;
    p.reserve(static_cast<std::size_t>(wcnt + hcnt) * 2);
    for (int i = 0; i < wcnt; ++i) p.emplace_back(i * wstep, 0.0);
    for (int i = 0; i < hcnt; ++i) p.emplace_back(w, i * hstep);
    for (int i = 0; i < wcnt; ++i) p.emplace_back((wcnt - i) * wstep, h);
    for (int i = 0; i < hcnt; ++i) p.emplace_back(0.0, (hcnt - i) * hstep);
    for (auto& pt : p) {
        pt.first += static_cast<float>(margin);
        pt.second += static_cast<float>(margin);
    }
    return p;
}

// Linear interpolation of `steps` points between each pair (interpolate_points).
std::vector<Pt> interpolatePoints(const std::vector<Pt>& pts, int steps) {
    const int d = steps + 1;
    const int new_len = d * static_cast<int>(pts.size()) - steps;
    std::vector<Pt> out;
    out.reserve(new_len);
    for (int i = 0; i < new_len; ++i) {
        const int idx1 = i / d;
        const int idx2 = std::min(idx1 + 1, static_cast<int>(pts.size()) - 1);
        const float f = static_cast<float>(i % d) / static_cast<float>(d);
        out.emplace_back(pts[idx1].first + f * (pts[idx2].first - pts[idx1].first),
                         pts[idx1].second + f * (pts[idx2].second - pts[idx1].second));
    }
    return out;
}

struct NearestEdge {
    std::optional<std::size_t> idx;
    double bw;  // half-width
    double bh;  // half-height
};

NearestEdge nearestEdge(const std::vector<Pt>& polygon, double cx, double cy,
                        NearestEdge best, double inv_aspect) {
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const double apx = std::abs(polygon[i].first - cx);
        const double apy = std::abs(polygon[i].second - cy);
        if (apx < best.bw && apy < best.bh) {
            if (apy > apx * inv_aspect) {
                best = {i, apy / inv_aspect, apy};
            } else {
                best = {i, apx, apx * inv_aspect};
            }
        }
    }
    return best;
}

// Forward-maps source points to stabilized output coordinates (undistort_points).
// newK is constant for the frame (fov == 1, output == input dims); R varies per point via
// rolling shutter (the point's coordinate on the readout axis selects the readout time).
std::vector<Pt> undistortPoints(const std::vector<Pt>& src, const Mat3& newK,
                                const Quaternion& smoothedInv,
                                const std::vector<TimeQuat>& raw, double start_ts,
                                double row_readout, bool horizontal, double readout,
                                const std::array<double, 2>& f,
                                const std::array<double, 2>& c,
                                const std::array<double, 4>& k) {
    std::vector<Pt> out;
    out.reserve(src.size());
    for (const auto& p : src) {
        const double axis = horizontal ? p.first : p.second;
        const double quat_time = readout > 0.0 ? start_ts + row_readout * axis : start_ts;
        const Quaternion rawAtRow = sampleQuaternion(raw, quat_time);
        const Quaternion quat = smoothedInv * rawAtRow;

        Mat3 R = Mat3::fromQuaternion(quat);
        R.at(0, 1) *= -1.0;
        R.at(0, 2) *= -1.0;
        R.at(1, 0) *= -1.0;
        R.at(2, 0) *= -1.0;
        const Mat3 M = newK * R;

        const double pwx = (p.first - c[0]) / f[0];
        const double pwy = (p.second - c[1]) / f[1];
        const auto und = OpenCVFisheye::undistortPoint({pwx, pwy}, k);
        if (!und) {
            out.emplace_back(static_cast<float>(kInvalid), static_cast<float>(kInvalid));
            continue;
        }
        // reproject: M * (u, v, 1), then perspective divide.
        const double X = M.at(0, 0) * und->first + M.at(0, 1) * und->second + M.at(0, 2);
        const double Y = M.at(1, 0) * und->first + M.at(1, 1) * und->second + M.at(1, 2);
        const double W = M.at(2, 0) * und->first + M.at(2, 1) * und->second + M.at(2, 2);
        // Reject points behind the virtual camera (W <= 0). The render kernel skips these
        // (its `_w > 0` guard); without the same guard here the polygon folds across the
        // horizon and the inscribed-rect FOV is computed against bogus far-away points.
        if (W <= 0.0) {
            out.emplace_back(static_cast<float>(kInvalid), static_cast<float>(kInvalid));
            continue;
        }
        out.emplace_back(static_cast<float>(X / W), static_cast<float>(Y / W));
    }
    return out;
}

// --- GaussianFilter path (zoom_dynamic.rs static-window helpers, method == 0) ---

// zoom_dynamic::get_frames_per_window: floor(window * fps), forced odd.
std::size_t framesPerWindow(double window_s, double fps) {
    long frames = static_cast<long>(std::floor(window_s * fps));
    if (frames < 1) frames = 1;
    if (frames % 2 == 0) frames += 1;
    return static_cast<std::size_t>(frames);
}

// zoom_dynamic::min_rolling: rolling minimum over `window` (valid positions only).
std::vector<double> minRolling(const std::vector<double>& a, std::size_t window) {
    std::vector<double> out;
    if (window == 0 || a.size() < window) return out;
    out.reserve(a.size() - window + 1);
    for (std::size_t i = 0; i + window <= a.size(); ++i) {
        double m = a[i];
        for (std::size_t j = 1; j < window; ++j) m = std::min(m, a[i + j]);
        out.push_back(m);
    }
    return out;
}

// zoom_dynamic::convolve: valid (no-pad) cross-correlation; filter is symmetric so it
// equals convolution.
std::vector<double> convolveValid(const std::vector<double>& v, const std::vector<double>& filter) {
    std::vector<double> out;
    if (filter.empty() || v.size() < filter.size()) return out;
    out.reserve(v.size() - filter.size() + 1);
    for (std::size_t i = 0; i + filter.size() <= v.size(); ++i) {
        double s = 0.0;
        for (std::size_t j = 0; j < filter.size(); ++j) s += v[i + j] * filter[j];
        out.push_back(s);
    }
    return out;
}

// zoom_dynamic::gaussian_window_normalized: m taps over [-m/2, m/2] (integer division),
// normalized to sum 1.
std::vector<double> gaussianWindowNormalized(std::size_t m, double std_dev) {
    const long half = static_cast<long>(m) / 2;
    const double sig2 = 2.0 * std_dev * std_dev;
    std::vector<double> w;
    w.reserve(static_cast<std::size_t>(2 * half + 1));
    for (long x = -half; x <= half; ++x) {
        w.push_back(std::exp(-static_cast<double>(x * x) / sig2));
    }
    double sum = 0.0;
    for (double v : w) sum += v;
    if (sum != 0.0) for (double& v : w) v /= sum;
    return w;
}

// zoom_dynamic::pad_edge: extend with the first/last value on each side.
std::vector<double> padEdge(const std::vector<double>& arr, std::size_t left, std::size_t right) {
    const double first = arr.empty() ? 0.0 : arr.front();
    const double last = arr.empty() ? 0.0 : arr.back();
    std::vector<double> out(arr.size() + left + right);
    for (std::size_t i = 0; i < left; ++i) out[i] = first;
    std::copy(arr.begin(), arr.end(), out.begin() + static_cast<std::ptrdiff_t>(left));
    for (std::size_t i = left + arr.size(); i < out.size(); ++i) out[i] = last;
    return out;
}

// GaussianFilter static-window smoothing: pad, rolling-min over the window, pad, Gaussian
// convolve. Output length == input length (window odd => 2*(window/2)+1 == window).
std::vector<double> gaussianFilterSmooth(const std::vector<double>& fov_values,
                                         double window_s, double fps) {
    if (fov_values.empty()) return fov_values;
    const std::size_t frames = framesPerWindow(window_s, fps);
    const std::size_t half = frames / 2;
    const std::vector<double> pad = padEdge(fov_values, half, half);
    const std::vector<double> fov_min = minRolling(pad, frames);
    const std::vector<double> min_pad = padEdge(fov_min, half, half);
    const std::vector<double> gaussian =
        gaussianWindowNormalized(frames, static_cast<double>(frames) / 6.0);
    return convolveValid(min_pad, gaussian);
}

// EnvelopeFollower with a constant alpha (zoom_dynamic::envelope_follower).
// Tracks the minimum so the crop never exceeds what avoids borders.
std::vector<double> envelopeFollower(const std::vector<double>& a, double coeff) {
    const std::size_t n = a.size();
    if (n == 0) return {};
    std::vector<double> rev(n);
    double q = a[n - 1];
    for (std::size_t kk = 0; kk < n; ++kk) {
        const double x = a[n - 1 - kk];
        q = std::min(x, x * coeff + q * (1.0 - coeff));
        rev[kk] = q;
    }
    std::vector<double> out(n);
    q = rev[n - 1];
    for (std::size_t kk = 0; kk < n; ++kk) {
        const double x = rev[n - 1 - kk];
        q = std::min(x, x * coeff + q * (1.0 - coeff));
        out[kk] = q;
    }
    return out;
}

} // namespace

std::vector<double> computeAdaptiveFovs(const std::vector<double>& frame_timestamps_ms,
                                        const std::vector<TimeQuat>& raw,
                                        const std::vector<TimeQuat>& smoothed,
                                        const LensProfile& lens, int width, int height,
                                        double fps, const TransformParams& tp,
                                        const AdaptiveZoomParams& az,
                                        std::vector<double>* raw_fovs_out) {
    const std::size_t nf = frame_timestamps_ms.size();
    if (nf == 0) return {};

    const double fx = lens.camera_matrix.fx;
    const double fy = lens.camera_matrix.fy;
    const std::array<double, 2> f{fx, fy};
    const std::array<double, 2> c{lens.camera_matrix.cx, lens.camera_matrix.cy};
    std::array<double, 4> k{0, 0, 0, 0};
    for (std::size_t i = 0; i < 4 && i < lens.distortion_coeffs.size(); ++i)
        k[i] = lens.distortion_coeffs[i];

    // new_K with fov == 1 and output == input dimensions (as in calculate_fovs).
    Mat3 newK = Mat3::identity();
    newK.at(0, 0) = fx;
    newK.at(1, 1) = fy;
    newK.at(0, 2) = width / 2.0;
    newK.at(1, 2) = height / 2.0;

    // Inscribed-rectangle aspect uses the OUTPUT dimensions; everything else (border ring,
    // new_K centre, search centre, fov denominator) stays in INPUT dims. Gyroflow computes
    // the FOV with output_width/height temporarily set to the input size (zooming/mod.rs:48)
    // so output_dim.0 collapses to the input width, while output_inv_aspect keeps the real
    // output aspect ratio (fov_iterative.rs: output_inv_aspect = output_dim.1/output_dim.0).
    const int ow = tp.output_width > 0
                       ? tp.output_width
                       : (lens.output_width > 0 ? lens.output_width : width);
    const int oh = tp.output_height > 0
                       ? tp.output_height
                       : (lens.output_height > 0 ? lens.output_height : height);
    const double out_w = width;  // output_dim.0 == input width (the ratio cancels)
    const double inv_aspect = static_cast<double>(oh) / static_cast<double>(ow);
    const double cx = width / 2.0;
    const double cy = height / 2.0;

    const bool horizontal = isHorizontal(tp.frame_readout_direction);
    const double readout = std::max(0.0, tp.frame_readout_time_ms);
    const int readout_axis_len = horizontal ? width : height;
    const double row_readout = readout_axis_len > 0 ? readout / readout_axis_len : 0.0;

    // Dense border ring. A single full scan over a dense polygon is more robust than
    // Gyroflow's local vertex refinement (which can lock onto the wrong edge), and the
    // undistorted border is smooth so density alone gives ample precision.
    const std::vector<Pt> rect = pointsAroundRect(width, height, az.fov_algorithm_margin,
                                                  121, 121);

    auto subtract_center = [&](std::vector<Pt>& poly) {
        for (auto& p : poly) {
            p.first -= static_cast<float>(az.center_offset.first * width);
            p.second -= static_cast<float>(az.center_offset.second * height);
        }
    };

    std::vector<double> fov_values(nf, 1.0);
    for (std::size_t fi = 0; fi < nf; ++fi) {
        const double ts = frame_timestamps_ms[fi];
        const double start_ts = ts - readout / 2.0;
        const Quaternion smoothedInv = sampleQuaternion(smoothed, ts).inverse();

        std::vector<Pt> polygon = undistortPoints(rect, newK, smoothedInv, raw, start_ts,
                                                  row_readout, horizontal, readout, f, c, k);
        subtract_center(polygon);

        const NearestEdge nearest =
            nearestEdge(polygon, cx, cy, {std::nullopt, 1000000.0, 1000000.0 * inv_aspect},
                        inv_aspect);
        fov_values[fi] = nearest.bw * 2.0 / out_w;
    }

    // Snapshot the instantaneous per-frame inscribed FOV (the "required" FOV) before any
    // temporal smoothing / max_zoom clamp, for analysis (black-border cause).
    if (raw_fovs_out) *raw_fovs_out = fov_values;

    // Temporal smoothing (zoom_dynamic::compute, non-keyframed static window).
    if (az.method == ZoomMethod::GaussianFilter) {
        // method == 0: rolling-min over the window, then Gaussian convolution.
        fov_values = gaussianFilterSmooth(fov_values, az.window_s, fps);
    } else {
        // method == 1: two-pass min-tracking envelope follower (Gyroflow default).
        const double first_alpha = 1.0 - std::exp(-(1.0 / fps) / az.window_s);
        const double second_alpha = 1.0 - std::exp(-(1.0 / fps) / 0.2);
        fov_values = envelopeFollower(fov_values, first_alpha);
        fov_values = envelopeFollower(fov_values, second_alpha);
    }

    // max_zoom clamp: zoom = 1/fov must not exceed max_zoom_percent/100.
    if (az.max_zoom_percent > 50.0) {
        const double min_fov = 100.0 / az.max_zoom_percent;
        for (double& v : fov_values) v = std::max(v, min_fov);
    }

    return fov_values;
}

} // namespace gyroflow
