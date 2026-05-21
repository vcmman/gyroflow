#pragma once

#include <cstddef>
#include <vector>

namespace gyroflow {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double norm() const;
    Quaternion normalized() const;
    Quaternion inverse() const;

    static Quaternion identity();
    static Quaternion fromAxisAngle(Vec3 axis, double radians);
};

Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);
double dot(const Quaternion& lhs, const Quaternion& rhs);
Quaternion slerp(const Quaternion& a, const Quaternion& b, double t);

struct TimeQuat {
    double timestamp_ms = 0.0;
    Quaternion quat;
};

Quaternion sampleQuaternion(const std::vector<TimeQuat>& samples, double timestamp_ms);
std::vector<TimeQuat> smoothQuaternionsPlain(const std::vector<TimeQuat>& samples, double smoothness_ms);

} // namespace gyroflow
