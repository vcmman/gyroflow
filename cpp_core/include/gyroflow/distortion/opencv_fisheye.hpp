#pragma once

#include <array>
#include <optional>
#include <utility>

namespace gyroflow {

// OpenCV fisheye distortion model (the default model for DJI clips).
//
// Ported from Gyroflow's Rust implementation:
//   src/core/stabilization/distortion_models/opencv_fisheye.rs
// which itself is adapted from OpenCV modules/calib3d/src/fisheye.cpp.
//
// Coefficient layout matches OpenCV / Gyroflow: k = {k0, k1, k2, k3}, applied to the
// polynomial theta_d = theta * (1 + k0*theta^2 + k1*theta^4 + k2*theta^6 + k3*theta^8).
struct OpenCVFisheye {
    // Forward distortion: project a camera-space point (x, y, z) to a normalized image
    // point (before multiplying by focal length and adding the principal point).
    //
    // Mirrors `distort_point(x, y, z, params)`. The caller is responsible for checking
    // z > 0 (points behind the camera are invalid); this matches the kernel's `_w > 0`
    // guard in cpu_undistort.rs and is intentionally not enforced here.
    static std::pair<double, double> distortPoint(double x, double y, double z,
                                                  const std::array<double, 4>& k);

    // Inverse distortion: recover the undistorted normalized point from a distorted one.
    // Uses the 10-iteration Newton solve from OpenCV. Returns std::nullopt when the
    // solver fails to converge or theta flips sign (i.e. the point is not recoverable),
    // matching the Rust `Option` return. Needed for adaptive zoom / STMap, not the
    // forward remap.
    static std::optional<std::pair<double, double>> undistortPoint(
        std::pair<double, double> point, const std::array<double, 4>& k);
};

} // namespace gyroflow
