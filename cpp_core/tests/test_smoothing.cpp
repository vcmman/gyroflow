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

    std::printf("test_smoothing: OK\n");
    return 0;
}
