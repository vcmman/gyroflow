#include "gyroflow/smoothing/default_algo.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace gyroflow {

namespace {

constexpr double kMaxVelocity = 500.0;     // deg/s
constexpr double kFovReference = 120.0;    // diagonal FOV reference (deg)
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

// Rotation angle of a quaternion, in [0, pi] (matches nalgebra UnitQuaternion::angle).
double quatAngle(const Quaternion& qin) {
    const Quaternion q = qin.normalized();
    const double vnorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    return 2.0 * std::atan2(vnorm, std::abs(q.w));
}

struct Euler {
    double roll, pitch, yaw;  // components [0],[1],[2]: rotations about X, Y, Z
};

// Euler angles (roll, pitch, yaw) matching nalgebra Rotation3::euler_angles() — the intrinsic
// X-Y-Z (Rz(yaw)·Ry(pitch)·Rx(roll)) sequence used by UnitQuaternion::euler_angles(). Derived
// from the quaternion->matrix elements (m20 = -sin(pitch); roll = atan2(m21, m22);
// yaw = atan2(m10, m00)); atan2 is invariant to the positive cos(pitch) divisor.
Euler quatToEuler(const Quaternion& qin) {
    const Quaternion q = qin.normalized();
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    const double m20 = 2.0 * (x * z - w * y);
    if (std::abs(m20) < 1.0) {
        const double m21 = 2.0 * (y * z + w * x);
        const double m22 = 1.0 - 2.0 * (x * x + y * y);
        const double m10 = 2.0 * (x * y + w * z);
        const double m00 = 1.0 - 2.0 * (y * y + z * z);
        return {std::atan2(m21, m22), -std::asin(m20), std::atan2(m10, m00)};
    }
    // Gimbal lock (|pitch| == pi/2): roll/yaw not separable; follow nalgebra's branches.
    const double m01 = 2.0 * (x * y - w * z);
    const double m02 = 2.0 * (x * z + w * y);
    constexpr double kHalfPi = 1.57079632679489661923;
    if (m20 <= -1.0) return {std::atan2(m01, m02), kHalfPi, 0.0};
    return {std::atan2(-m01, -m02), -kHalfPi, 0.0};
}

// Inverse of quatToEuler: nalgebra UnitQuaternion::from_euler_angles(roll, pitch, yaw)
// = qz(yaw) * qy(pitch) * qx(roll).
Quaternion eulerToQuat(double roll, double pitch, double yaw) {
    const double sr = std::sin(roll * 0.5), cr = std::cos(roll * 0.5);
    const double sp = std::sin(pitch * 0.5), cp = std::cos(pitch * 0.5);
    const double sy = std::sin(yaw * 0.5), cy = std::cos(yaw * 0.5);
    return {cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
}

} // namespace

namespace {

using V3 = std::array<double, 3>;

// --- Direction-Consistency-Ratio (DCR) gating -------------------------------------------
// See DefaultAlgoParams for the rationale. DCR is scale-invariant, so the omega arrays below
// stay in raw radians (no deg/s conversion needed). Both helpers run in O(n) via prefix sums,
// which matters because gyro streams here are ~1 kHz (tens of thousands of samples).

// Signed rotation vector (scaled axis, radians) of a quaternion, forced into the canonical
// w>=0 hemisphere so successive small rotations keep a consistent sign.
V3 quatRotVec(const Quaternion& qin) {
    Quaternion q = qin.normalized();
    double w = q.w, x = q.x, y = q.y, z = q.z;
    if (w < 0.0) { w = -w; x = -x; y = -y; z = -z; }
    const double vnorm = std::sqrt(x * x + y * y + z * z);
    if (vnorm < 1e-12) return {0.0, 0.0, 0.0};
    const double s = 2.0 * std::atan2(vnorm, w) / vnorm;
    return {x * s, y * s, z * s};
}

// Half window (in samples) for the sliding consistency estimate.
long dcrHalfWindow(double sample_rate, const DefaultAlgoParams& p) {
    return std::max<long>(1, std::lround(p.dcr_window_s * sample_rate / 2.0));
}

// Per-axis gate: velocity[i][c] *= (|sum(omega_c)| / sum(|omega_c|))^power over the window.
void applyDcrPerAxis(const std::vector<TimeQuat>& quats, std::vector<V3>& velocity,
                     double sample_rate, const DefaultAlgoParams& p) {
    const std::size_t n = quats.size();
    if (n < 3) return;
    std::vector<V3> omega(n, V3{0.0, 0.0, 0.0});
    for (std::size_t i = 1; i < n; ++i) {
        const Euler e = quatToEuler(quats[i - 1].quat.inverse() * quats[i].quat);
        omega[i] = {e.roll, e.pitch, e.yaw};
    }
    std::vector<V3> psig(n + 1, V3{0.0, 0.0, 0.0});
    std::vector<V3> pabs(n + 1, V3{0.0, 0.0, 0.0});
    for (std::size_t i = 0; i < n; ++i)
        for (int c = 0; c < 3; ++c) {
            psig[i + 1][c] = psig[i][c] + omega[i][c];
            pabs[i + 1][c] = pabs[i][c] + std::abs(omega[i][c]);
        }
    const long h = dcrHalfWindow(sample_rate, p);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t lo = (static_cast<long>(i) > h) ? i - static_cast<std::size_t>(h) : 0;
        const std::size_t hi = std::min<std::size_t>(n - 1, i + static_cast<std::size_t>(h));
        for (int c = 0; c < 3; ++c) {
            const double a = pabs[hi + 1][c] - pabs[lo][c];
            double dcr = (a > 1e-9) ? std::abs(psig[hi + 1][c] - psig[lo][c]) / a : 1.0;
            if (p.dcr_power != 1.0) dcr = std::pow(dcr, p.dcr_power);
            velocity[i][c] *= dcr;
        }
    }
}

// Scalar gate: velocity[i] *= (||sum(omega_vec)|| / sum(||omega_vec||))^power over the window.
void applyDcrScalar(const std::vector<TimeQuat>& quats, std::vector<double>& velocity,
                    double sample_rate, const DefaultAlgoParams& p) {
    const std::size_t n = quats.size();
    if (n < 3) return;
    std::vector<V3> omega(n, V3{0.0, 0.0, 0.0});
    for (std::size_t i = 1; i < n; ++i)
        omega[i] = quatRotVec(quats[i - 1].quat.inverse() * quats[i].quat);
    std::vector<V3> psig(n + 1, V3{0.0, 0.0, 0.0});
    std::vector<double> pabs(n + 1, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        pabs[i + 1] = pabs[i] + std::sqrt(omega[i][0] * omega[i][0] + omega[i][1] * omega[i][1] +
                                          omega[i][2] * omega[i][2]);
        for (int c = 0; c < 3; ++c) psig[i + 1][c] = psig[i][c] + omega[i][c];
    }
    const long h = dcrHalfWindow(sample_rate, p);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t lo = (static_cast<long>(i) > h) ? i - static_cast<std::size_t>(h) : 0;
        const std::size_t hi = std::min<std::size_t>(n - 1, i + static_cast<std::size_t>(h));
        const double a = pabs[hi + 1] - pabs[lo];
        const double sx = psig[hi + 1][0] - psig[lo][0];
        const double sy = psig[hi + 1][1] - psig[lo][1];
        const double sz = psig[hi + 1][2] - psig[lo][2];
        double dcr = (a > 1e-9) ? std::sqrt(sx * sx + sy * sy + sz * sz) / a : 1.0;
        if (p.dcr_power != 1.0) dcr = std::pow(dcr, p.dcr_power);
        velocity[i] *= dcr;
    }
}

// Per-axis smoothing — faithful port of the per_axis branch of
// src/core/smoothing/default_algo.rs. Smooths the three euler components of each relative
// rotation independently (own velocity scaling per axis), via euler decompose / recompose
// instead of slerp. Kept fully separate from the scalar path.
std::vector<TimeQuat> smoothPerAxis(const std::vector<TimeQuat>& quats, double duration_ms,
                                    const DefaultAlgoParams& p) {
    const std::size_t n = quats.size();
    const double sample_rate = static_cast<double>(n) / (duration_ms / 1000.0);
    const double rad_to_deg_per_sec = sample_rate * kRadToDeg;
    auto get_alpha = [&](double tc) { return 1.0 - std::exp(-(1.0 / sample_rate) / tc); };
    const double alpha_smoothness = get_alpha(p.max_smoothness);
    const double alpha_0_1s = get_alpha(p.alpha_0_1s);

    // ---- per-axis velocity (deg/s) from euler components ----
    std::vector<V3> velocity(n, V3{0.0, 0.0, 0.0});
    for (std::size_t i = 1; i < n; ++i) {
        const Euler e = quatToEuler(quats[i - 1].quat.inverse() * quats[i].quat);
        velocity[i] = {std::abs(e.roll) * rad_to_deg_per_sec,
                       std::abs(e.pitch) * rad_to_deg_per_sec,
                       std::abs(e.yaw) * rad_to_deg_per_sec};
    }
    // smooth velocity (forward then backward) with alpha_0_1s, per component
    for (std::size_t i = 1; i < n; ++i)
        for (int c = 0; c < 3; ++c)
            velocity[i][c] = velocity[i - 1][c] * (1.0 - alpha_0_1s) + velocity[i][c] * alpha_0_1s;
    for (std::size_t i = n - 1; i-- > 0;)
        for (int c = 0; c < 3; ++c)
            velocity[i][c] = velocity[i + 1][c] * (1.0 - alpha_0_1s) + velocity[i][c] * alpha_0_1s;

    // normalize per axis (no master smoothness; per-axis sliders only — matches Rust)
    const double fov_ratio = p.camera_diagonal_fov / kFovReference;
    V3 max_velocity = {kMaxVelocity * p.smoothness_pitch * fov_ratio,
                       kMaxVelocity * p.smoothness_yaw * fov_ratio,
                       kMaxVelocity * p.smoothness_roll * fov_ratio};
    for (int c = 0; c < 3; ++c) {
        if (p.second_pass) max_velocity[c] *= 0.5;
        if (max_velocity[c] <= 0.0) max_velocity[c] = 1e-9;
    }
    for (std::size_t i = 0; i < n; ++i)
        for (int c = 0; c < 3; ++c) velocity[i][c] /= max_velocity[c];

    // DCR gate: only let the velocity-dampening loosen where motion is directionally
    // consistent (pan), not where it reciprocates (bob). Off by default => parity preserved.
    if (p.dcr) applyDcrPerAxis(quats, velocity, sample_rate, p);

    // ---- per-axis adaptive pass (euler decompose, scale each component, recompose) ----
    auto adaptive_pass = [&](const std::vector<Quaternion>& in, const std::vector<V3>& ratio) {
        auto step = [&](Quaternion& q, const Quaternion& x, const V3& r) {
            V3 f;
            for (int c = 0; c < 3; ++c)
                f[c] = std::min(1.0, alpha_smoothness * (1.0 - r[c]) + alpha_0_1s * r[c]);
            const Euler er = quatToEuler(q.inverse() * x);
            q = q * eulerToQuat(er.roll * f[0], er.pitch * f[1], er.yaw * f[2]);
        };
        std::vector<Quaternion> fwd(n);
        Quaternion q = in.front();
        for (std::size_t i = 0; i < n; ++i) { step(q, in[i], ratio[i]); fwd[i] = q; }
        std::vector<Quaternion> out(n);
        q = fwd.back();
        for (std::size_t i = n; i-- > 0;) { step(q, fwd[i], ratio[i]); out[i] = q; }
        return out;
    };

    std::vector<Quaternion> raw(n);
    for (std::size_t i = 0; i < n; ++i) raw[i] = quats[i].quat;

    std::vector<Quaternion> smoothed = adaptive_pass(raw, velocity);

    if (p.second_pass) {
        // ---- per-axis distance between smoothed and raw ----
        std::vector<V3> distance(n, V3{0.0, 0.0, 0.0});
        V3 max_distance = {0.0, 0.0, 0.0};
        for (std::size_t i = 0; i < n; ++i) {
            const Euler e = quatToEuler(raw[i].inverse() * smoothed[i]);
            distance[i] = {std::abs(e.roll), std::abs(e.pitch), std::abs(e.yaw)};
            for (int c = 0; c < 3; ++c) max_distance[c] = std::max(max_distance[c], distance[i][c]);
        }
        for (int c = 0; c < 3; ++c) if (max_distance[c] <= 0.0) max_distance[c] = 1e-9;
        // normalize, discard below 0.5
        for (std::size_t i = 0; i < n; ++i)
            for (int c = 0; c < 3; ++c) {
                distance[i][c] /= max_distance[c];
                if (distance[i][c] < 0.5) distance[i][c] = 0.0;
            }
        // smooth distance (forward then backward)
        for (std::size_t i = 1; i < n; ++i)
            for (int c = 0; c < 3; ++c)
                distance[i][c] = distance[i - 1][c] * (1.0 - alpha_0_1s) + distance[i][c] * alpha_0_1s;
        for (std::size_t i = n - 1; i-- > 0;)
            for (int c = 0; c < 3; ++c)
                distance[i][c] = distance[i + 1][c] * (1.0 - alpha_0_1s) + distance[i][c] * alpha_0_1s;
        // renormalize to 0.5 .. 1.0
        max_distance = {0.0, 0.0, 0.0};
        for (std::size_t i = 0; i < n; ++i)
            for (int c = 0; c < 3; ++c) max_distance[c] = std::max(max_distance[c], distance[i][c]);
        for (int c = 0; c < 3; ++c) if (max_distance[c] <= 0.0) max_distance[c] = 1e-9;
        for (std::size_t i = 0; i < n; ++i)
            for (int c = 0; c < 3; ++c) {
                distance[i][c] /= max_distance[c];
                distance[i][c] = (distance[i][c] + 1.0) / 2.0;
            }

        std::vector<V3> combined(n);
        for (std::size_t i = 0; i < n; ++i)
            for (int c = 0; c < 3; ++c) combined[i][c] = velocity[i][c] * distance[i][c];

        smoothed = adaptive_pass(smoothed, combined);
    }

    std::vector<TimeQuat> out(n);
    for (std::size_t i = 0; i < n; ++i) {
        out[i].timestamp_ms = quats[i].timestamp_ms;
        out[i].quat = smoothed[i].normalized();
    }
    return out;
}

} // namespace

std::vector<TimeQuat> smoothDefault(const std::vector<TimeQuat>& quats, double duration_ms,
                                    const DefaultAlgoParams& p) {
    const std::size_t n = quats.size();
    if (n == 0 || duration_ms <= 0.0) return quats;

    if (p.per_axis) return smoothPerAxis(quats, duration_ms, p);

    const double sample_rate = static_cast<double>(n) / (duration_ms / 1000.0);
    const double rad_to_deg_per_sec = sample_rate * kRadToDeg;

    auto get_alpha = [&](double time_constant) {
        return 1.0 - std::exp(-(1.0 / sample_rate) / time_constant);
    };
    const double alpha_smoothness = get_alpha(p.max_smoothness);
    const double alpha_0_1s = get_alpha(p.alpha_0_1s);

    // ---- velocity (deg/s) ----
    std::vector<double> velocity(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) {
        const Quaternion dist = quats[i - 1].quat.inverse() * quats[i].quat;
        velocity[i] = quatAngle(dist) * rad_to_deg_per_sec;
    }

    // smooth velocity (forward then backward) with alpha_0_1s
    for (std::size_t i = 1; i < n; ++i)
        velocity[i] = velocity[i - 1] * (1.0 - alpha_0_1s) + velocity[i] * alpha_0_1s;
    for (std::size_t i = n - 1; i-- > 0;)
        velocity[i] = velocity[i + 1] * (1.0 - alpha_0_1s) + velocity[i] * alpha_0_1s;

    // normalize velocity by max velocity
    const double fov_ratio = p.camera_diagonal_fov / kFovReference;
    double max_velocity = kMaxVelocity * p.smoothness * fov_ratio;
    if (p.second_pass) max_velocity *= 0.5;  // match max-zoom behaviour of single pass
    if (max_velocity <= 0.0) max_velocity = 1e-9;
    for (std::size_t i = 0; i < n; ++i) velocity[i] /= max_velocity;

    // DCR gate: only let the velocity-dampening loosen where motion is directionally
    // consistent (pan), not where it reciprocates (bob). Off by default => parity preserved.
    if (p.dcr) applyDcrScalar(quats, velocity, sample_rate, p);

    // ---- first plain-3D pass with velocity-adaptive alpha ----
    auto adaptive_pass = [&](const std::vector<Quaternion>& in,
                             const std::vector<double>& ratio) {
        std::vector<Quaternion> fwd(n);
        Quaternion q = in.front();
        for (std::size_t i = 0; i < n; ++i) {
            const double val = alpha_smoothness * (1.0 - ratio[i]) + alpha_0_1s * ratio[i];
            q = slerp(q, in[i], std::min(val, 1.0));
            fwd[i] = q;
        }
        std::vector<Quaternion> out(n);
        q = fwd.back();
        for (std::size_t i = n; i-- > 0;) {
            const double val = alpha_smoothness * (1.0 - ratio[i]) + alpha_0_1s * ratio[i];
            q = slerp(q, fwd[i], std::min(val, 1.0));
            out[i] = q;
        }
        return out;
    };

    std::vector<Quaternion> raw(n);
    for (std::size_t i = 0; i < n; ++i) raw[i] = quats[i].quat;

    std::vector<Quaternion> smoothed = adaptive_pass(raw, velocity);

    if (p.second_pass) {
        // ---- distance between smoothed and raw ----
        std::vector<double> distance(n, 0.0);
        double max_distance = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const double d = quatAngle(raw[i].inverse() * smoothed[i]);
            distance[i] = d;
            max_distance = std::max(max_distance, d);
        }
        if (max_distance <= 0.0) max_distance = 1e-9;
        // normalize, discard below 0.5
        for (std::size_t i = 0; i < n; ++i) {
            distance[i] /= max_distance;
            if (distance[i] < 0.5) distance[i] = 0.0;
        }
        // smooth distance (forward then backward)
        for (std::size_t i = 1; i < n; ++i)
            distance[i] = distance[i - 1] * (1.0 - alpha_0_1s) + distance[i] * alpha_0_1s;
        for (std::size_t i = n - 1; i-- > 0;)
            distance[i] = distance[i + 1] * (1.0 - alpha_0_1s) + distance[i] * alpha_0_1s;
        // renormalize to 0.5 .. 1.0
        max_distance = 0.0;
        for (std::size_t i = 0; i < n; ++i) max_distance = std::max(max_distance, distance[i]);
        if (max_distance <= 0.0) max_distance = 1e-9;
        for (std::size_t i = 0; i < n; ++i) {
            distance[i] /= max_distance;
            distance[i] = (distance[i] + 1.0) / 2.0;
        }

        std::vector<double> combined(n);
        for (std::size_t i = 0; i < n; ++i) combined[i] = velocity[i] * distance[i];

        smoothed = adaptive_pass(smoothed, combined);
    }

    std::vector<TimeQuat> out(n);
    for (std::size_t i = 0; i < n; ++i) {
        out[i].timestamp_ms = quats[i].timestamp_ms;
        out[i].quat = smoothed[i].normalized();
    }
    return out;
}

} // namespace gyroflow
