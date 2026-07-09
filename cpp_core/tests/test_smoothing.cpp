// Smoothing tests: scalar default_algo sanity + per-axis behavior.
// The exact per-axis math is validated against Gyroflow golden metadata separately
// (tools/compare_gyroflow_metadata.py, --per-axis); here we assert the behavioral
// invariants that are cheap and deterministic.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "gyroflow/smoothing/default_algo.hpp"
#include "gyroflow/types.hpp"

using namespace gyroflow;

namespace {

double relAngle(const Quaternion& a, const Quaternion& b) {
    const Quaternion r = a.inverse() * b;
    const Quaternion n = r.normalized();
    return 2.0 * std::atan2(std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z), std::abs(n.w));
}

// Total variation = summed consecutive rotation angle (a proxy for residual jitter).
double totalVariation(const std::vector<TimeQuat>& s) {
    double t = 0.0;
    for (std::size_t i = 1; i < s.size(); ++i) t += relAngle(s[i - 1].quat, s[i].quat);
    return t;
}

// Series rotating about the Y axis: a smooth ramp + high-frequency jitter.
std::vector<TimeQuat> makeSeries() {
    const std::size_t n = 300;
    std::vector<TimeQuat> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double tt = static_cast<double>(i);
        const double ramp = 0.30 * tt / n;                  // slow drift
        const double jit = 0.04 * std::sin(tt * 2.0);       // fast jitter to be removed
        v[i].timestamp_ms = tt * (1000.0 / 30.0);
        v[i].quat = Quaternion::fromAxisAngle(Vec3{0.0, 1.0, 0.0}, ramp + jit);
    }
    return v;
}

bool allUnit(const std::vector<TimeQuat>& s) {
    for (const auto& q : s)
        if (std::abs(q.quat.norm() - 1.0) > 1e-9) return false;
    return true;
}

constexpr double PI = 3.14159265358979323846;

// Reciprocating "running bob": a large, fast oscillation about one axis (alternating
// direction). The stock velocity-dampening loosens on its high speed and follows it; DCR
// should recognise the alternating direction and keep it smoothed.
std::vector<TimeQuat> makeBob() {
    const std::size_t n = 300;
    std::vector<TimeQuat> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double tt = static_cast<double>(i);
        const double bob = 0.25 * std::sin(tt * (2.0 * PI / 10.0));  // period 10 samples
        v[i].timestamp_ms = tt * (1000.0 / 30.0);
        v[i].quat = Quaternion::fromAxisAngle(Vec3{0.0, 1.0, 0.0}, bob);
    }
    return v;
}

// Sustained pan: a fast, consistent-direction ramp (no reversal). DCR must NOT tighten here
// (direction is consistent => gate ~1), so the output should match the un-gated pan.
std::vector<TimeQuat> makePan() {
    const std::size_t n = 300;
    std::vector<TimeQuat> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double tt = static_cast<double>(i);
        const double ramp = 10.0 * tt / n;  // ~10 rad over the clip => genuinely fast
        v[i].timestamp_ms = tt * (1000.0 / 30.0);
        v[i].quat = Quaternion::fromAxisAngle(Vec3{0.0, 1.0, 0.0}, ramp);
    }
    return v;
}

// Max per-sample rotation angle between two equally-sized series.
double maxDiff(const std::vector<TimeQuat>& a, const std::vector<TimeQuat>& b) {
    double m = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) m = std::max(m, relAngle(a[i].quat, b[i].quat));
    return m;
}

} // namespace

int main() {
    const auto raw = makeSeries();
    const double dur = raw.back().timestamp_ms - raw.front().timestamp_ms;
    const double rawTV = totalVariation(raw);

    // --- scalar path ---
    {
        DefaultAlgoParams p;  // defaults: scalar, second pass on
        const auto s = smoothDefault(raw, dur, p);
        assert(s.size() == raw.size());
        assert(allUnit(s));
        assert(totalVariation(s) < rawTV);  // smoothing removes jitter
    }

    // --- per-axis path: runs, stays unit, removes jitter ---
    {
        DefaultAlgoParams p;
        p.per_axis = true;
        p.smoothness_pitch = p.smoothness_yaw = p.smoothness_roll = 0.5;
        const auto s = smoothDefault(raw, dur, p);
        assert(s.size() == raw.size());
        assert(allUnit(s));
        assert(totalVariation(s) < rawTV);
    }

    // --- per-axis monotonicity: larger smoothness on the active (Y) axis => smoother.
    // Rotation about Y maps to euler component [1], scaled by smoothness_yaw.
    {
        DefaultAlgoParams loose;
        loose.per_axis = true;
        loose.smoothness_pitch = loose.smoothness_roll = 0.5;
        loose.smoothness_yaw = 0.1;  // small slider => less smoothing
        DefaultAlgoParams tight = loose;
        tight.smoothness_yaw = 0.9;  // large slider => more smoothing

        const double tvLoose = totalVariation(smoothDefault(raw, dur, loose));
        const double tvTight = totalVariation(smoothDefault(raw, dur, tight));
        assert(tvTight < tvLoose);
        assert(tvLoose < rawTV);
    }

    // --- DCR gating: keeps a reciprocating bob smoothed (scalar + per-axis) ---
    {
        const auto bob = makeBob();
        const double bdur = bob.back().timestamp_ms - bob.front().timestamp_ms;
        const double bobTV = totalVariation(bob);

        // scalar
        DefaultAlgoParams off;              // DCR off (default)
        DefaultAlgoParams on = off; on.dcr = true;
        const auto sOff = smoothDefault(bob, bdur, off);
        const auto sOn  = smoothDefault(bob, bdur, on);
        assert(allUnit(sOn));
        assert(totalVariation(sOn) < totalVariation(sOff));  // DCR removes more of the bob
        assert(totalVariation(sOn) < bobTV);

        // per-axis
        DefaultAlgoParams poff; poff.per_axis = true;
        DefaultAlgoParams pon = poff; pon.dcr = true;
        const auto pOff = smoothDefault(bob, bdur, poff);
        const auto pOn  = smoothDefault(bob, bdur, pon);
        assert(allUnit(pOn));
        assert(totalVariation(pOn) < totalVariation(pOff));
    }

    // --- DCR gating: leaves a consistent-direction pan essentially unchanged ---
    {
        const auto pan = makePan();
        const double pdur = pan.back().timestamp_ms - pan.front().timestamp_ms;
        DefaultAlgoParams off;
        DefaultAlgoParams on = off; on.dcr = true;
        const auto sOff = smoothDefault(pan, pdur, off);
        const auto sOn  = smoothDefault(pan, pdur, on);
        // gate ~1 on a sustained pan => outputs nearly identical (small tolerance for the
        // truncated-window ratio near the very ends).
        assert(maxDiff(sOff, sOn) < 0.01);
    }

    // --- finite look-ahead (look_ahead_s > 0): W >= n must reproduce the offline sweep
    //     bit-for-bit; a short window must still smooth (runs, unit, removes jitter) ---
    {
        DefaultAlgoParams off;                                     // offline (look_ahead 0)
        const auto sOff = smoothDefault(raw, dur, off);
        DefaultAlgoParams huge = off;
        huge.look_ahead_s = 1000.0;                                // W >= n => same as offline
        const auto sHuge = smoothDefault(raw, dur, huge);
        assert(maxDiff(sOff, sHuge) < 1e-12);

        DefaultAlgoParams la = off;
        la.look_ahead_s = 0.5;                                     // finite in-camera window
        const auto sLa = smoothDefault(raw, dur, la);
        assert(sLa.size() == raw.size());
        assert(allUnit(sLa));
        assert(totalVariation(sLa) < rawTV);                       // still removes jitter
    }

    // --- deviation clamp: bounds angle(smoothed, raw) by B; 0 = bit-identical off ---
    {
        const auto bob = makeBob();
        const double bdur = bob.back().timestamp_ms - bob.front().timestamp_ms;
        DefaultAlgoParams off;                       // clamp off (default)
        DefaultAlgoParams on = off; on.deviation_clamp_deg = 3.0;
        const auto sOff = smoothDefault(bob, bdur, off);
        const auto sOn  = smoothDefault(bob, bdur, on);
        assert(allUnit(sOn));
        const double B = 3.0 * PI / 180.0;
        for (std::size_t i = 0; i < bob.size(); ++i)
            assert(relAngle(bob[i].quat, sOn[i].quat) <= B + 1e-9);   // never beyond the box
        // the un-clamped smoother deviates beyond 3 deg somewhere on this bob (else no-op test)
        double mx = 0.0;
        for (std::size_t i = 0; i < bob.size(); ++i)
            mx = std::max(mx, relAngle(bob[i].quat, sOff[i].quat));
        assert(mx > B);
        DefaultAlgoParams zero = off; zero.deviation_clamp_deg = 0.0;
        assert(maxDiff(smoothDefault(bob, bdur, zero), sOff) < 1e-15); // 0 == off, bit-identical
    }

    std::printf("test_smoothing: OK\n");
    return 0;
}
