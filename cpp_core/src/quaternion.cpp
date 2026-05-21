#include "gyroflow/types.hpp"

#include <algorithm>
#include <cmath>

namespace gyroflow {

namespace {

constexpr double kEpsilon = 1e-12;

double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

} // namespace

double Quaternion::norm() const {
    return std::sqrt(w * w + x * x + y * y + z * z);
}

Quaternion Quaternion::normalized() const {
    const double n = norm();
    if (n <= kEpsilon) {
        return Quaternion::identity();
    }
    return {w / n, x / n, y / n, z / n};
}

Quaternion Quaternion::inverse() const {
    const double n2 = w * w + x * x + y * y + z * z;
    if (n2 <= kEpsilon) {
        return Quaternion::identity();
    }
    return {w / n2, -x / n2, -y / n2, -z / n2};
}

Quaternion Quaternion::identity() {
    return {};
}

Quaternion Quaternion::fromAxisAngle(Vec3 axis, double radians) {
    const double axis_norm = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (axis_norm <= kEpsilon) {
        return Quaternion::identity();
    }

    const double half = radians * 0.5;
    const double scale = std::sin(half) / axis_norm;
    return {std::cos(half), axis.x * scale, axis.y * scale, axis.z * scale};
}

Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs) {
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

double dot(const Quaternion& lhs, const Quaternion& rhs) {
    return lhs.w * rhs.w + lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Quaternion slerp(const Quaternion& a, const Quaternion& b, double t) {
    Quaternion from = a.normalized();
    Quaternion to = b.normalized();
    double cos_theta = dot(from, to);

    if (cos_theta < 0.0) {
        to = {-to.w, -to.x, -to.y, -to.z};
        cos_theta = -cos_theta;
    }

    t = clamp(t, 0.0, 1.0);
    if (cos_theta > 0.9995) {
        return Quaternion{
            from.w + t * (to.w - from.w),
            from.x + t * (to.x - from.x),
            from.y + t * (to.y - from.y),
            from.z + t * (to.z - from.z),
        }.normalized();
    }

    const double theta = std::acos(clamp(cos_theta, -1.0, 1.0));
    const double sin_theta = std::sin(theta);
    const double wa = std::sin((1.0 - t) * theta) / sin_theta;
    const double wb = std::sin(t * theta) / sin_theta;

    return Quaternion{
        from.w * wa + to.w * wb,
        from.x * wa + to.x * wb,
        from.y * wa + to.y * wb,
        from.z * wa + to.z * wb,
    }.normalized();
}

Quaternion sampleQuaternion(const std::vector<TimeQuat>& samples, double timestamp_ms) {
    if (samples.empty()) {
        return Quaternion::identity();
    }
    if (timestamp_ms <= samples.front().timestamp_ms) {
        return samples.front().quat;
    }
    if (timestamp_ms >= samples.back().timestamp_ms) {
        return samples.back().quat;
    }

    const auto upper = std::upper_bound(
        samples.begin(),
        samples.end(),
        timestamp_ms,
        [](double ts, const TimeQuat& sample) {
            return ts < sample.timestamp_ms;
        });

    const auto lower = upper - 1;
    const double span = upper->timestamp_ms - lower->timestamp_ms;
    if (span <= kEpsilon) {
        return lower->quat;
    }

    const double t = (timestamp_ms - lower->timestamp_ms) / span;
    return slerp(lower->quat, upper->quat, t);
}

} // namespace gyroflow
