#include "gyroflow/smoothing/crop_agc.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace gyroflow {

namespace {

// Centered sliding-window maximum via monotonic deque, O(n).
std::vector<double> windowMaxCentered(const std::vector<double>& x, std::size_t W) {
    const std::size_t n = x.size();
    std::vector<double> out(n);
    if (W <= 1) { out = x; return out; }
    const std::size_t half = W / 2;
    std::deque<std::size_t> dq;  // indices, values decreasing
    std::size_t r = 0;           // next index to push (window [i-half, i+half])
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t hi = std::min(n - 1, i + half);
        for (; r <= hi; ++r) {
            while (!dq.empty() && x[dq.back()] <= x[r]) dq.pop_back();
            dq.push_back(r);
        }
        const std::size_t lo = i >= half ? i - half : 0;
        while (!dq.empty() && dq.front() < lo) dq.pop_front();
        out[i] = x[dq.front()];
    }
    return out;
}

// Zero-phase (forward + backward) EMA, alpha from tau at the given rate.
void zeroPhaseEma(std::vector<double>& x, double rate_hz, double tau_s) {
    if (x.empty() || tau_s <= 0.0 || rate_hz <= 0.0) return;
    const double a = 1.0 - std::exp(-(1.0 / rate_hz) / tau_s);
    double v = x.front();
    for (std::size_t i = 0; i < x.size(); ++i) { v += a * (x[i] - v); x[i] = v; }
    v = x.back();
    for (std::size_t i = x.size(); i-- > 0;) { v += a * (x[i] - v); x[i] = v; }
}

} // namespace

std::vector<TimeQuat> applyCropBudgetAGC(const std::vector<TimeQuat>& raw,
                                         const std::vector<TimeQuat>& smoothed, double fps,
                                         double max_zoom, const CropDemandFn& demand_fn,
                                         const CropAGCParams& params, CropAGCReport* report) {
    CropAGCReport rep;
    std::vector<TimeQuat> out = smoothed;
    const std::size_t n = smoothed.size();
    if (n < 2 || raw.size() != n || fps <= 0.0 || max_zoom <= 1.0) {
        if (report) *report = rep;
        return out;
    }

    // Fundamental-only reference: zero-phase EMA of raw (§8j-4 / §8n-2 pattern).
    const double duration_s =
        (raw.back().timestamp_ms - raw.front().timestamp_ms) / 1000.0;
    const double gyro_rate = duration_s > 0.0 ? static_cast<double>(n - 1) / duration_s : fps;
    const double a_ref = 1.0 - std::exp(-(1.0 / gyro_rate) / std::max(params.ref_tau_s, 1e-4));
    std::vector<TimeQuat> ref(n);
    Quaternion q = raw.front().quat;
    for (std::size_t i = 0; i < n; ++i) {
        q = slerp(q, raw[i].quat, a_ref);
        ref[i].timestamp_ms = raw[i].timestamp_ms;
        ref[i].quat = q;
    }
    q = ref[n - 1].quat;
    for (std::size_t i = n; i-- > 0;) {
        q = slerp(q, ref[i].quat, a_ref);
        ref[i].quat = q.normalized();
    }

    // Per-frame demand of the smoothed path and of the reference.
    const std::vector<double> ds = demand_fn(smoothed);
    const std::vector<double> dr = demand_fn(ref);
    const std::size_t nf = std::min(ds.size(), dr.size());
    if (nf == 0) {
        if (report) *report = rep;
        return out;
    }
    const double target = 1.0 + (max_zoom - 1.0) * params.margin;
    for (std::size_t f = 0; f < nf; ++f) {
        rep.max_reqz_before = std::max(rep.max_reqz_before, ds[f]);
        if (ds[f] > max_zoom) ++rep.breach_before;
    }

    // Demand envelope -> per-frame gain. Envelope speed keeps the gain a compressor (§8l);
    // the pointwise peak-hold keeps the bound at isolated spikes the smoothing rounds off.
    std::vector<double> env =
        windowMaxCentered(ds, std::max<std::size_t>(1, static_cast<std::size_t>(
                                                           std::lround(params.window_s * fps))));
    zeroPhaseEma(env, fps, params.env_tau_s);
    std::vector<double> g(nf, 1.0);
    auto gainFor = [&](double demand, double ref_demand) {
        if (demand <= target) return 1.0;
        if (ref_demand >= target) return 0.0;
        return std::min(1.0, std::max(0.0, (target - ref_demand) / (demand - ref_demand)));
    };
    for (std::size_t f = 0; f < nf; ++f) g[f] = gainFor(std::max(env[f], ds[f]), dr[f]);

    // Apply + verify; tighten residual frames locally (demand is only locally linear in the
    // deviation, and mixing changes the path the demand was measured on).
    const auto apply = [&]() {
        for (std::size_t i = 0; i < n; ++i) {
            const double fpos = smoothed[i].timestamp_ms / 1000.0 * fps;
            const std::size_t f0 = std::min<std::size_t>(
                nf - 1, static_cast<std::size_t>(std::max(0.0, std::floor(fpos))));
            const std::size_t f1 = std::min(nf - 1, f0 + 1);
            const double w = std::min(1.0, std::max(0.0, fpos - static_cast<double>(f0)));
            const double gi = g[f0] * (1.0 - w) + g[f1] * w;
            out[i].quat = gi >= 1.0 ? smoothed[i].quat
                                    : slerp(ref[i].quat, smoothed[i].quat, gi).normalized();
        }
    };
    for (int round = 0; round < std::max(1, params.outer_iters); ++round) {
        apply();
        rep.outer_iters = round + 1;
        const std::vector<double> d = demand_fn(out);
        const std::size_t nd = std::min(nf, d.size());
        rep.breach_after = 0;
        rep.max_reqz_after = 0.0;
        bool any = false;
        for (std::size_t f = 0; f < nd; ++f) {
            rep.max_reqz_after = std::max(rep.max_reqz_after, d[f]);
            if (d[f] > max_zoom) ++rep.breach_after;
            if (d[f] > target) {
                any = true;
                const double shrink = gainFor(d[f], dr[f]);
                const std::size_t lo = f >= 2 ? f - 2 : 0;
                const std::size_t hi = std::min(nf - 1, f + 2);
                for (std::size_t j = lo; j <= hi; ++j) g[j] = std::min(g[j], g[f] * shrink);
            }
        }
        if (!any) break;
    }
    for (std::size_t f = 0; f < nf; ++f) {
        rep.min_gain = std::min(rep.min_gain, g[f]);
        if (g[f] < 1.0) ++rep.gained_frames;
    }
    if (report) *report = rep;
    return out;
}

} // namespace gyroflow
