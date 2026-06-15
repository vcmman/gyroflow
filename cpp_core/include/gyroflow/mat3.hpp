#pragma once

#include <array>
#include <optional>

#include "gyroflow/types.hpp"

namespace gyroflow {

// Minimal row-major 3x3 matrix. Kept dependency-light (no Eigen) since the core library
// is meant to have no external dependencies.
struct Mat3 {
    // m[row*3 + col]
    std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};

    double& at(int r, int c) { return m[r * 3 + c]; }
    double at(int r, int c) const { return m[r * 3 + c]; }

    static Mat3 identity() { return Mat3{}; }

    // Active rotation matrix from a unit quaternion, matching nalgebra's
    // UnitQuaternion::to_rotation_matrix convention used by Gyroflow.
    static Mat3 fromQuaternion(const Quaternion& qin) {
        const Quaternion q = qin.normalized();
        const double w = q.w, x = q.x, y = q.y, z = q.z;
        Mat3 r;
        r.m = {
            1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y),
            2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
            2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y),
        };
        return r;
    }

    double determinant() const {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) -
               m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    // Returns the inverse, or std::nullopt if (near-)singular.
    std::optional<Mat3> inverse() const {
        const double det = determinant();
        if (det == 0.0 || !(det == det) /* NaN */) return std::nullopt;
        const double inv = 1.0 / det;
        Mat3 r;
        r.m[0] = (m[4] * m[8] - m[5] * m[7]) * inv;
        r.m[1] = (m[2] * m[7] - m[1] * m[8]) * inv;
        r.m[2] = (m[1] * m[5] - m[2] * m[4]) * inv;
        r.m[3] = (m[5] * m[6] - m[3] * m[8]) * inv;
        r.m[4] = (m[0] * m[8] - m[2] * m[6]) * inv;
        r.m[5] = (m[2] * m[3] - m[0] * m[5]) * inv;
        r.m[6] = (m[3] * m[7] - m[4] * m[6]) * inv;
        r.m[7] = (m[1] * m[6] - m[0] * m[7]) * inv;
        r.m[8] = (m[0] * m[4] - m[1] * m[3]) * inv;
        return r;
    }
};

inline Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 r;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i * 3 + j] = a.m[i * 3 + 0] * b.m[0 * 3 + j] +
                             a.m[i * 3 + 1] * b.m[1 * 3 + j] +
                             a.m[i * 3 + 2] * b.m[2 * 3 + j];
        }
    }
    return r;
}

} // namespace gyroflow
