#include "gyroflow/distortion/opencv_fisheye.hpp"

#include <algorithm>
#include <cmath>

namespace gyroflow {

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

std::pair<double, double> OpenCVFisheye::distortPoint(double x, double y, double z,
                                                      const std::array<double, 4>& k) {
    x = x / z;
    y = y / z;
    if (k[0] == 0.0 && k[1] == 0.0 && k[2] == 0.0 && k[3] == 0.0) {
        return {x, y};
    }

    const double r = std::sqrt(x * x + y * y);

    const double theta = std::atan(r);
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;

    const double theta_d =
        theta * (1.0 + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);

    const double scale = (r == 0.0) ? 1.0 : theta_d / r;

    return {x * scale, y * scale};
}

std::optional<std::pair<double, double>> OpenCVFisheye::undistortPoint(
    std::pair<double, double> point, const std::array<double, 4>& k) {
    if (k[0] == 0.0 && k[1] == 0.0 && k[2] == 0.0 && k[3] == 0.0) {
        return point;
    }

    constexpr double kEps = 1e-6;

    double theta_d = std::sqrt(point.first * point.first + point.second * point.second);

    // The camera model is only valid up to 180 degrees FOV; clip so we still produce
    // plausible results for super-fisheye images > 180 deg.
    theta_d = std::max(-kPi, std::min(theta_d, kPi));

    bool converged = false;
    double theta = theta_d;
    double scale = 0.0;

    if (std::abs(theta_d) > kEps) {
        theta = 0.0;

        // Compensate distortion iteratively (Newton's method).
        for (int i = 0; i < 10; ++i) {
            const double theta2 = theta * theta;
            const double theta4 = theta2 * theta2;
            const double theta6 = theta4 * theta2;
            const double theta8 = theta6 * theta2;
            const double k0_theta2 = k[0] * theta2;
            const double k1_theta4 = k[1] * theta4;
            const double k2_theta6 = k[2] * theta6;
            const double k3_theta8 = k[3] * theta8;
            // new_theta = theta - theta_fix, theta_fix = f0(theta) / f0'(theta)
            double theta_fix =
                (theta * (1.0 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                (1.0 + 3.0 * k0_theta2 + 5.0 * k1_theta4 + 7.0 * k2_theta6 + 9.0 * k3_theta8);

            theta_fix = std::max(-0.9, std::min(theta_fix, 0.9));

            theta = theta - theta_fix;
            if (std::abs(theta_fix) < kEps) {
                converged = true;
                break;
            }
        }

        scale = std::tan(theta) / theta_d;
    } else {
        converged = true;
    }

    // theta is monotonic; if it flipped sign during the optimization it converged on the
    // opposite side of the camera center, so reject it.
    const bool theta_flipped =
        (theta_d < 0.0 && theta > 0.0) || (theta_d > 0.0 && theta < 0.0);

    if (converged && !theta_flipped) {
        return std::make_pair(point.first * scale, point.second * scale);
    }
    return std::nullopt;
}

} // namespace gyroflow
