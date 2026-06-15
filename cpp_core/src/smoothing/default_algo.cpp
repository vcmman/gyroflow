#include "gyroflow/smoothing/default_algo.hpp"

#include <algorithm>
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

} // namespace

std::vector<TimeQuat> smoothDefault(const std::vector<TimeQuat>& quats, double duration_ms,
                                    const DefaultAlgoParams& p) {
    const std::size_t n = quats.size();
    if (n == 0 || duration_ms <= 0.0) return quats;

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
