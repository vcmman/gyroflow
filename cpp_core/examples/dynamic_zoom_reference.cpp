// =============================================================================================
//  dynamic_zoom_reference.cpp
//
//  Self-contained REFERENCE implementation of Gyroflow's dynamic ("adaptive") zoom, written for
//  porting to an embedded / board-side codebase. No external dependencies (only <cmath>,
//  <vector>, <cstdio>); C++11. Compile & run the built-in demo with:
//
//      g++ -std=c++11 -O2 dynamic_zoom_reference.cpp -o dz && ./dz
//
//  ---------------------------------------------------------------------------------------------
//  WHAT THIS ALGORITHM DOES
//  ---------------------------------------------------------------------------------------------
//  A stabilized video is produced by rotating each frame to a smoothed camera orientation. That
//  rotation swings the real image around inside the output rectangle, so parts of the output can
//  fall outside the real image -> BLACK BORDERS. Dynamic zoom removes them by cropping/zooming in
//  just enough, *and it varies that zoom smoothly over time* so you crop only as much as each
//  moment needs (tight during violent motion, wide when steady) instead of a single worst-case
//  crop for the whole clip.
//
//  The output is one FOV multiplier per frame:  fov[i] in (0, 1].
//      * fov = 1.0  -> no zoom (use the whole frame).
//      * fov < 1.0  -> crop a centered fraction `fov` of the frame and upscale; zoom = 1 / fov.
//  The renderer then crops a centered window of size (fov*W) x (fov*H) and scales it to the
//  output resolution (equivalently: scale the virtual camera focal length by 1/fov).
//
//  ---------------------------------------------------------------------------------------------
//  ALGORITHM FLOW (four stages; see the numbered banners below)
//  ---------------------------------------------------------------------------------------------
//    STAGE 0  Build a dense ring of sample points around the source-image border (once).
//    STAGE 1  Per frame: map that border ring through the SAME stabilization warp the renderer
//             uses (lens undistort + smoothed rotation + rolling shutter), then find the largest
//             centered rectangle of the output aspect ratio that fits inside the mapped polygon.
//             Its width, as a fraction of the frame, is the frame's "required" FOV (the widest
//             crop with NO black border for that frame).   -> required_fov[i]
//    STAGE 2  Smooth required_fov[] over time so the crop changes gently, not per-frame jitter.
//             Two interchangeable methods (pick one; EnvelopeFollower is Gyroflow's default):
//               (a) EnvelopeFollower : two-pass min-tracking IIR envelope.
//               (b) GaussianFilter   : rolling-min over a window, then Gaussian convolution.
//             Both are minimum-tracking: the smoothed value never rises above what a frame needs,
//             so smoothing can only crop MORE, never re-introduce a border.
//    STAGE 3  Clamp by max_zoom: never zoom past a user limit (fov >= 1 / max_zoom). This is the
//             ONLY place a black border can be forced back in (when a frame needs more zoom than
//             the limit allows) -- everything else is border-free by construction.
//
//  ---------------------------------------------------------------------------------------------
//  WHAT YOU MUST PROVIDE WHEN PORTING  (the one codebase-specific piece)
//  ---------------------------------------------------------------------------------------------
//  STAGE 1 needs to map source pixels to stabilized-output pixels for a given frame. That mapping
//  IS your stabilization transform and depends on your lens model + smoothed quaternions, so it
//  is supplied as a callback (`MapBorderFn`). Everything else here is generic and portable as-is.
//  The demo at the bottom plugs in a trivial rotation so the file compiles and runs; replace it.
// =============================================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <vector>

namespace dz {

// ------------------------------------------------------------------------------------------- //
//  Basic types
// ------------------------------------------------------------------------------------------- //

struct Point2 {
    double x;
    double y;
};

// Temporal-smoothing method for STAGE 2.
enum class Method {
    EnvelopeFollower = 0,  // Gyroflow default (adaptive_zoom_method = 1)
    GaussianFilter   = 1,  // (adaptive_zoom_method = 0)
};

// ------------------------------------------------------------------------------------------- //
//  The stabilization-warp callback you implement (STAGE 1 input).
//
//  For frame `frame_index`, map each source-image point `src[j]` to its position in the
//  stabilized OUTPUT image, writing the result to `dst[j]`. If a point maps behind the virtual
//  camera / is invalid, set dst[j] = {INVALID, INVALID} (defined below) so it is ignored.
//
//  Coordinate frame (IMPORTANT): both src and dst are in pixels with the ORIGIN AT THE TOP-LEFT
//  and the image center at (width/2, height/2) -- i.e. use a virtual camera matrix whose focal
//  lengths are the sensor fx/fy and whose principal point is (width/2, height/2), with fov = 1
//  and output size == input size. (Gyroflow computes the FOV with output_dim temporarily set to
//  the input size; the real output aspect ratio enters only through STAGE 1's inv_aspect below.)
//
//  Your implementation must, per point:
//     1. undistort src[j] with the lens model (fisheye/rectilinear) -> normalized ray,
//     2. rotate by  R = smoothed_orientation(frame)^-1 * raw_orientation(t_row),
//        where t_row is the point's per-row readout time (rolling shutter); use the frame's
//        mid-exposure time if you ignore rolling shutter,
//     3. reproject through the virtual camera matrix above; drop points with w <= 0.
//  This is exactly the transform the render kernel applies, so the crop it yields is exact.
//
//  `user` is an opaque pointer for your own context (lens params, quaternion buffers, ...).
// ------------------------------------------------------------------------------------------- //
typedef void (*MapBorderFn)(int frame_index, const std::vector<Point2>& src,
                            std::vector<Point2>& dst, void* user);

constexpr double INVALID = -1.0e9;

// ------------------------------------------------------------------------------------------- //
//  Parameters
// ------------------------------------------------------------------------------------------- //
struct Params {
    int    width          = 0;      // source image width  (px)  -- border ring + FOV denominator
    int    height         = 0;      // source image height (px)
    int    output_width   = 0;      // final output width  (px)  -- sets the crop ASPECT RATIO
    int    output_height  = 0;      // final output height (px)
    double fps            = 30.0;   // frame rate (used to convert window_s / tau to samples)
    double window_s       = 4.0;    // temporal smoothing window (seconds). Larger = smoother crop
    double max_zoom_pct   = 130.0;  // zoom clamp (%). <= 50 disables the clamp
    Method method         = Method::EnvelopeFollower;

    double margin_px      = 2.0;    // trim this many px off the border ring (avoids exact edge)
    double center_off_x   = 0.0;    // optional crop-center offset, as a FRACTION of width/height
    double center_off_y   = 0.0;
    int    ring_divisions = 121;    // border ring density per side (denser = more robust inscribe)

    // In-camera / real-time finite look-ahead for STAGE 2 (EnvelopeFollower only):
    //   < 0  -> OFFLINE two-pass envelope over the whole clip (uses all future; smoothest).
    //   >= 0 -> real-time causal envelope that sees only this many seconds of future.
    //           0 = fully causal (crop SNAPS in on a shake -> a zoom pop); 1.0 = 1 s anticipation
    //           (crop RAMPS in before the shake). It does NOT change black borders (the min-guard
    //           already gives zero) — it makes the real-time zoom track smooth, near-offline.
    double look_ahead_s   = -1.0;
};

// ============================================================================================= //
//  STAGE 0 : border ring
//  Points placed clockwise around the source rectangle (trimmed by `margin`). A dense ring makes
//  the inscribe in STAGE 1 robust (the undistorted border is smooth, so density alone suffices).
// ============================================================================================= //
inline std::vector<Point2> pointsAroundRect(double w, double h, double margin, int divisions) {
    w -= margin * 2.0;
    h -= margin * 2.0;
    const int wcnt = (divisions < 2 ? 2 : divisions) - 1;
    const int hcnt = wcnt;
    const double wstep = w / wcnt;
    const double hstep = h / hcnt;

    std::vector<Point2> p;
    p.reserve(static_cast<size_t>(wcnt + hcnt) * 2);
    for (int i = 0; i < wcnt; ++i) p.push_back({i * wstep,        0.0});   // top    (L->R)
    for (int i = 0; i < hcnt; ++i) p.push_back({w,                i * hstep}); // right (T->B)
    for (int i = 0; i < wcnt; ++i) p.push_back({(wcnt - i) * wstep, h});   // bottom (R->L)
    for (int i = 0; i < hcnt; ++i) p.push_back({0.0,              (hcnt - i) * hstep}); // left
    for (auto& pt : p) { pt.x += margin; pt.y += margin; }
    return p;
}

// ============================================================================================= //
//  STAGE 1 helper : largest centered rectangle (of the output aspect) inside the mapped polygon.
//
//  Scan every mapped border point. Its offset from the crop center is (apx, apy). The biggest
//  centered rectangle of aspect (1 : inv_aspect) that stays inside the frame cannot extend past
//  the NEAREST border point on whichever axis binds first. We keep shrinking the best box to the
//  tightest constraint any point imposes. `inv_aspect = output_height / output_width`.
//
//  Returns the box HALF-WIDTH (in the same pixel units as the polygon). The frame's required FOV
//  is then  fov = (half_width * 2) / width.
// ============================================================================================= //
inline double inscribedHalfWidth(const std::vector<Point2>& polygon, double cx, double cy,
                                  double inv_aspect) {
    double best_bw = 1.0e6;                 // half-width  (start effectively unbounded)
    double best_bh = 1.0e6 * inv_aspect;    // half-height (kept consistent with the aspect)
    for (const auto& pt : polygon) {
        if (pt.x <= INVALID * 0.5 || pt.y <= INVALID * 0.5) continue;  // skip invalid points
        const double apx = std::fabs(pt.x - cx);
        const double apy = std::fabs(pt.y - cy);
        // Only points already inside the current best box can tighten it.
        if (apx < best_bw && apy < best_bh) {
            if (apy > apx * inv_aspect) {
                // the vertical offset binds first -> height-limited
                best_bw = apy / inv_aspect;
                best_bh = apy;
            } else {
                // the horizontal offset binds first -> width-limited
                best_bw = apx;
                best_bh = apx * inv_aspect;
            }
        }
    }
    return best_bw;
}

// ============================================================================================= //
//  STAGE 2a : EnvelopeFollower (two-pass, min-tracking IIR).  Gyroflow default.
//
//  One pass is a leaky integrator that can only be pulled DOWN quickly and rises slowly:
//        q = min( x,  coeff*x + (1-coeff)*q )
//  Run it backward then forward (two passes) so the response is symmetric / zero net phase. The
//  min() guarantees the envelope never exceeds the required FOV -> never re-creates a border.
//  `coeff` comes from a time constant tau:  coeff = 1 - exp(-(1/fps)/tau).
// ============================================================================================= //
inline std::vector<double> envelopeFollower(const std::vector<double>& a, double coeff) {
    const size_t n = a.size();
    if (n == 0) return {};
    std::vector<double> rev(n);
    double q = a[n - 1];
    for (size_t k = 0; k < n; ++k) {                 // pass 1: back -> front
        const double x = a[n - 1 - k];
        q = std::fmin(x, coeff * x + (1.0 - coeff) * q);
        rev[k] = q;
    }
    std::vector<double> out(n);
    q = rev[n - 1];
    for (size_t k = 0; k < n; ++k) {                 // pass 2: over the reversed result
        const double x = rev[n - 1 - k];
        q = std::fmin(x, coeff * x + (1.0 - coeff) * q);
        out[k] = q;
    }
    return out;
}

// ============================================================================================= //
//  STAGE 2b : GaussianFilter (rolling-min over a window, then Gaussian convolution).
//  Alternative to the envelope follower. Edge-padded so output length == input length.
// ============================================================================================= //
inline size_t framesPerWindow(double window_s, double fps) {
    long f = static_cast<long>(std::floor(window_s * fps));
    if (f < 1) f = 1;
    if (f % 2 == 0) f += 1;               // force odd so the window is symmetric
    return static_cast<size_t>(f);
}
inline std::vector<double> padEdge(const std::vector<double>& a, size_t left, size_t right) {
    const double first = a.empty() ? 0.0 : a.front();
    const double last  = a.empty() ? 0.0 : a.back();
    std::vector<double> out(a.size() + left + right);
    for (size_t i = 0; i < left; ++i) out[i] = first;
    for (size_t i = 0; i < a.size(); ++i) out[left + i] = a[i];
    for (size_t i = left + a.size(); i < out.size(); ++i) out[i] = last;
    return out;
}
inline std::vector<double> rollingMin(const std::vector<double>& a, size_t win) {
    std::vector<double> out;
    if (win == 0 || a.size() < win) return out;
    out.reserve(a.size() - win + 1);
    std::deque<size_t> mono;  // monotonic deque: front = window minimum; O(n) total
    for (size_t i = 0; i < a.size(); ++i) {
        while (!mono.empty() && a[mono.back()] >= a[i]) mono.pop_back();
        mono.push_back(i);
        if (i + 1 >= win) {
            if (mono.front() + win <= i) mono.pop_front();
            out.push_back(a[mono.front()]);
        }
    }
    return out;
}
inline std::vector<double> gaussianWindow(size_t m, double std_dev) {
    const long half = static_cast<long>(m) / 2;
    const double sig2 = 2.0 * std_dev * std_dev;
    std::vector<double> w;
    double sum = 0.0;
    for (long x = -half; x <= half; ++x) { double v = std::exp(-double(x * x) / sig2); w.push_back(v); sum += v; }
    if (sum != 0.0) for (double& v : w) v /= sum;
    return w;
}
inline std::vector<double> convolveValid(const std::vector<double>& v, const std::vector<double>& f) {
    std::vector<double> out;
    if (f.empty() || v.size() < f.size()) return out;
    out.reserve(v.size() - f.size() + 1);
    for (size_t i = 0; i + f.size() <= v.size(); ++i) {
        double s = 0.0;
        for (size_t j = 0; j < f.size(); ++j) s += v[i + j] * f[j];
        out.push_back(s);
    }
    return out;
}
inline std::vector<double> gaussianFilterSmooth(const std::vector<double>& fov, double window_s, double fps) {
    if (fov.empty()) return fov;
    const size_t win = framesPerWindow(window_s, fps);
    const size_t half = win / 2;
    const std::vector<double> pad     = padEdge(fov, half, half);
    const std::vector<double> minroll = rollingMin(pad, win);
    const std::vector<double> min_pad = padEdge(minroll, half, half);
    const std::vector<double> kernel  = gaussianWindow(win, double(win) / 6.0);
    return convolveValid(min_pad, kernel);
}

// ============================================================================================= //
//  STAGE 2c : real-time finite-look-ahead envelope (in-camera).  See Params::look_ahead_s.
//  A look-ahead MINIMUM of required FOV over [i, i+W] drives an asymmetric one-pole EMA (fast
//  tighten / slow open); the min(state, required) guard keeps applied <= required (no border).
//  W == 0 is fully causal (snaps in on shakes); W = 1 s ramps the crop in beforehand.
// ============================================================================================= //
// NOTE: hand-maintained port of the authoritative implementation in
// ../src/zooming/adaptive_zoom.cpp (envelopeLookAhead) — keep the two in sync.
inline std::vector<double> envelopeLookAhead(const std::vector<double>& req, double fps,
                                             double window_s, double look_ahead_s) {
    const size_t n = req.size();
    if (n == 0) return {};
    const size_t W = look_ahead_s > 0.0 ? static_cast<size_t>(std::lround(look_ahead_s * fps)) : 0;
    const double a_fast = 1.0 - std::exp(-(1.0 / fps) / 0.2);        // tighten (reactive)
    const double a_slow = 1.0 - std::exp(-(1.0 / fps) / window_s);   // open (smooth)

    // Sliding-window minimum over [i, i+W] with a monotonic deque (front = window min): each
    // index enters/leaves once -> O(n) total. On a board, replace std::deque with a fixed-size
    // ring buffer of W+1 indices.
    std::deque<size_t> mono;
    auto push = [&](size_t j) {
        while (!mono.empty() && req[mono.back()] >= req[j]) mono.pop_back();
        mono.push_back(j);
    };
    for (size_t j = 0; j <= std::min(W, n - 1); ++j) push(j);

    std::vector<double> out(n);
    double state = req[0];
    for (size_t i = 0; i < n; ++i) {
        if (mono.front() < i) mono.pop_front();      // expire indices left of the window
        const double target = req[mono.front()];     // min over [i, i+W]
        const double a = (target < state) ? a_fast : a_slow;
        state += a * (target - state);
        out[i] = std::fmin(state, req[i]);   // min-guard: never crop less than the frame needs
        const size_t incoming = i + 1 + W;
        if (incoming < n) push(incoming);
    }
    return out;
}

// ============================================================================================= //
//  computeDynamicZoom  --  the whole pipeline (STAGE 1 -> 2 -> 3).
//
//  INPUT :
//     p                 : Params (dimensions, fps, window, max_zoom, method, ...).
//     num_frames        : number of frames to compute.
//     map_border        : your stabilization-warp callback (see MapBorderFn above).
//     user              : opaque pointer passed straight through to map_border.
//     out_required_fov  : optional; if non-null, receives the per-frame REQUIRED fov before
//                         smoothing/clamp (STAGE 1 result) -- useful for black-border analysis.
//  OUTPUT (return value):
//     fov[i], i in [0, num_frames): the per-frame FOV multiplier to feed the renderer.
//                                   fov in (0,1]; zoom applied = 1/fov; crop = centered fov-frac.
// ============================================================================================= //
inline std::vector<double> computeDynamicZoom(const Params& p, int num_frames,
                                              MapBorderFn map_border, void* user,
                                              std::vector<double>* out_required_fov = nullptr) {
    if (num_frames <= 0 || p.width <= 0 || p.height <= 0) return {};

    // ---- STAGE 0 : border ring (built once, reused every frame) -----------------------------
    const std::vector<Point2> ring =
        pointsAroundRect(p.width, p.height, p.margin_px, p.ring_divisions);

    // Crop center and output aspect. cx,cy default to the image center, shifted by center_off.
    const double cx = p.width  / 2.0 - p.center_off_x * p.width;
    const double cy = p.height / 2.0 - p.center_off_y * p.height;
    const int ow = p.output_width  > 0 ? p.output_width  : p.width;
    const int oh = p.output_height > 0 ? p.output_height : p.height;
    const double inv_aspect = double(oh) / double(ow);
    const double fov_denom  = double(p.width);   // fov = half_width*2 / width (ratio cancels ow)

    // ---- STAGE 1 : per-frame required FOV (largest border-free centered crop) ----------------
    std::vector<double> required_fov(static_cast<size_t>(num_frames), 1.0);
    std::vector<Point2> mapped(ring.size());
    for (int fi = 0; fi < num_frames; ++fi) {
        map_border(fi, ring, mapped, user);                 // <-- your stabilization warp
        const double bw = inscribedHalfWidth(mapped, cx, cy, inv_aspect);
        required_fov[static_cast<size_t>(fi)] = bw * 2.0 / fov_denom;
    }
    if (out_required_fov) *out_required_fov = required_fov;

    // ---- STAGE 2 : temporal smoothing --------------------------------------------------------
    std::vector<double> fov;
    if (p.method == Method::GaussianFilter) {
        fov = gaussianFilterSmooth(required_fov, p.window_s, p.fps);
    } else if (p.look_ahead_s >= 0.0) {
        // Real-time in-camera envelope with a finite future window (STAGE 2c).
        fov = envelopeLookAhead(required_fov, p.fps, p.window_s, p.look_ahead_s);
    } else {
        // OFFLINE EnvelopeFollower: a wide window pass (tau = window_s) then a short reactive pass
        // (tau = 0.2 s) that lets the crop re-open a little faster once motion subsides.
        const double a1 = 1.0 - std::exp(-(1.0 / p.fps) / p.window_s);
        const double a2 = 1.0 - std::exp(-(1.0 / p.fps) / 0.2);
        fov = envelopeFollower(required_fov, a1);
        fov = envelopeFollower(fov, a2);
    }

    // ---- STAGE 3 : max_zoom clamp ------------------------------------------------------------
    // zoom = 1/fov must not exceed max_zoom_pct/100  ->  fov >= 100/max_zoom_pct.
    // This is the ONLY step that can leave a residual black border (when a frame genuinely needs
    // more zoom than the clamp allows). Raise max_zoom_pct to trade crop for fewer such frames.
    if (p.max_zoom_pct > 50.0) {
        const double min_fov = 100.0 / p.max_zoom_pct;
        for (double& v : fov) v = std::fmax(v, min_fov);
    }
    return fov;
}

} // namespace dz

// ============================================================================================= //
//  DEMO  --  replace mapBorderDemo() with your real stabilization warp.
//
//  This stand-in maps the border ring through a small, time-varying rotation about the image
//  center (a toy "shake") plus a pinhole reprojection, so the file compiles and runs and the crop
//  visibly breathes. It is NOT a lens model -- your port supplies undistort + smoothed rotation +
//  rolling shutter here.
// ============================================================================================= //
namespace {

struct DemoCtx {
    double width;
    double height;
    double focal;   // virtual focal length (px)
};

void mapBorderDemo(int frame_index, const std::vector<dz::Point2>& src,
                   std::vector<dz::Point2>& dst, void* user) {
    const DemoCtx* c = static_cast<const DemoCtx*>(user);
    // toy residual rotation the stabilizer "left in": a slow yaw + a faster pitch bob (radians)
    const double t = frame_index / 30.0;
    const double yaw   = 0.05 * std::sin(t * 0.7);
    const double pitch = 0.03 * std::sin(t * 6.0);
    const double roll  = 0.01 * std::sin(t * 3.0);
    // small-angle rotation matrix R ~ I + [w]x (good enough for a demo)
    const double r00 = 1.0,      r01 = -roll,  r02 = yaw;
    const double r10 = roll,     r11 = 1.0,    r12 = -pitch;
    const double r20 = -yaw,     r21 = pitch,  r22 = 1.0;
    const double cx = c->width / 2.0, cy = c->height / 2.0, f = c->focal;
    for (size_t j = 0; j < src.size(); ++j) {
        // back-project pixel to a ray, rotate, reproject (pinhole; no distortion in the demo)
        const double xn = (src[j].x - cx) / f;
        const double yn = (src[j].y - cy) / f;
        const double X = r00 * xn + r01 * yn + r02;
        const double Y = r10 * xn + r11 * yn + r12;
        const double W = r20 * xn + r21 * yn + r22;
        if (W <= 0.0) { dst[j] = {dz::INVALID, dz::INVALID}; continue; }
        dst[j] = {cx + f * (X / W), cy + f * (Y / W)};
    }
}

} // namespace

int main() {
    dz::Params p;
    p.width = 3840; p.height = 2880;          // 4:3 sensor
    p.output_width = 3840; p.output_height = 2160;  // 16:9 output crop
    p.fps = 30.0; p.window_s = 4.0; p.max_zoom_pct = 130.0;
    p.method = dz::Method::EnvelopeFollower;

    DemoCtx ctx{double(p.width), double(p.height), 1800.0};
    const int N = 150;

    std::vector<double> required;
    const std::vector<double> fov =
        dz::computeDynamicZoom(p, N, &mapBorderDemo, &ctx, &required);

    std::printf("frame   required_fov  applied_fov   zoom(1/fov)\n");
    for (int i = 0; i < N; i += 15) {
        std::printf("%5d   %10.4f   %10.4f   %8.3fx\n",
                    i, required[i], fov[i], 1.0 / fov[i]);
    }
    // Summary: dynamic zoom uses only as much crop as the clip needs, smoothly.
    double fmin = 1e9, fmean = 0.0;
    for (double v : fov) { fmin = std::fmin(fmin, v); fmean += v; }
    fmean /= fov.size();
    std::printf("\napplied fov: min %.4f (max zoom %.3fx), mean %.4f (mean zoom %.3fx)\n",
                fmin, 1.0 / fmin, fmean, 1.0 / fmean);
    return 0;
}
