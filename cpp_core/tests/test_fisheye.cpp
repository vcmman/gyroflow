#include "gyroflow/distortion/opencv_fisheye.hpp"

#include <array>
#include <cassert>
#include <cmath>

namespace {
bool close(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}
} // namespace

int main() {
    using namespace gyroflow;

    // Zero coefficients => identity (just the perspective divide by z).
    {
        const std::array<double, 4> k{0.0, 0.0, 0.0, 0.0};
        const auto d = OpenCVFisheye::distortPoint(0.3, -0.2, 2.0, k);
        assert(close(d.first, 0.15));
        assert(close(d.second, -0.1));

        const auto u = OpenCVFisheye::undistortPoint({0.15, -0.1}, k);
        assert(u.has_value());
        assert(close(u->first, 0.15));
        assert(close(u->second, -0.1));
    }

    // Round-trip distort -> undistort with realistic fisheye coefficients.
    {
        const std::array<double, 4> k{-0.02, 0.003, -0.0007, 0.0001};
        const double pts[][2] = {{0.0, 0.0}, {0.2, 0.1}, {-0.4, 0.3}, {0.6, -0.5}};
        for (const auto& p : pts) {
            const auto d = OpenCVFisheye::distortPoint(p[0], p[1], 1.0, k);
            const auto u = OpenCVFisheye::undistortPoint(d, k);
            assert(u.has_value());
            assert(close(u->first, p[0], 1e-5));
            assert(close(u->second, p[1], 1e-5));
        }
    }

    // The perspective divide must be applied before distortion.
    {
        const std::array<double, 4> k{-0.02, 0.003, 0.0, 0.0};
        const auto a = OpenCVFisheye::distortPoint(0.4, 0.2, 2.0, k);
        const auto b = OpenCVFisheye::distortPoint(0.2, 0.1, 1.0, k);
        assert(close(a.first, b.first));
        assert(close(a.second, b.second));
    }

    return 0;
}
