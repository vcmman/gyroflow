#include "gyroflow/stabilization/frame_transform.hpp"

#include <algorithm>
#include <cmath>

#include "gyroflow/mat3.hpp"

namespace gyroflow {

FrameTransform computeFrameTransform(double frame_center_ts_ms,
                                     const std::vector<TimeQuat>& rawQuats,
                                     const std::vector<TimeQuat>& smoothedQuats,
                                     const LensProfile& lens, int width, int height,
                                     const TransformParams& params) {
    FrameTransform t;
    t.width = width;
    t.height = height;
    t.output_width = lens.output_width > 0 ? lens.output_width : width;
    t.output_height = lens.output_height > 0 ? lens.output_height : height;

    const double fx = lens.camera_matrix.fx;
    const double fy = lens.camera_matrix.fy;
    const double cx = lens.camera_matrix.cx;
    const double cy = lens.camera_matrix.cy;

    // Source intrinsics (scaled_k in Gyroflow): f and c used to map the distorted point
    // back to source pixels.
    t.f = {fx, fy};
    t.c = {cx, cy};
    for (std::size_t i = 0; i < 4 && i < lens.distortion_coeffs.size(); ++i) {
        t.k[i] = lens.distortion_coeffs[i];
    }

    // Output (virtual) camera matrix new_K (get_new_k with no horizontal stretch).
    const double fov = params.fov != 0.0 ? params.fov : 1.0;
    Mat3 newK = Mat3::identity();
    newK.at(0, 0) = fx / fov;
    newK.at(1, 1) = fy / fov;
    newK.at(0, 2) = t.output_width / 2.0;
    newK.at(1, 2) = t.output_height / 2.0;
    newK.at(2, 2) = 1.0;

    const bool horizontal = isHorizontal(params.frame_readout_direction);
    t.horizontal_readout = horizontal;

    const double readout = std::max(0.0, params.frame_readout_time_ms);
    const int readout_axis_len = horizontal ? width : height;
    const double row_readout = readout_axis_len > 0 ? readout / readout_axis_len : 0.0;
    const double start_ts = frame_center_ts_ms - readout / 2.0;

    // Smoothed orientation is sampled once at the frame center.
    const Quaternion smoothedOri = sampleQuaternion(smoothedQuats, frame_center_ts_ms);
    const Quaternion smoothedInv = smoothedOri.inverse();

    const int rows = readout > 0.0 ? readout_axis_len : 1;
    t.matrices.resize(static_cast<std::size_t>(rows));

    for (int r = 0; r < rows; ++r) {
        const double quat_time = readout > 0.0 ? start_ts + row_readout * r : start_ts;
        const Quaternion rawAtRow = sampleQuaternion(rawQuats, quat_time);

        // quat = smoothed(ts)^-1 * raw(row_ts)  (== compensation form in frame_transform.rs)
        const Quaternion quat = smoothedInv * rawAtRow;

        Mat3 R = Mat3::fromQuaternion(quat);
        // Axis sign flips for a non-inverted framebuffer (frame_transform.rs:240).
        R.at(0, 1) *= -1.0;
        R.at(0, 2) *= -1.0;
        R.at(1, 0) *= -1.0;
        R.at(2, 0) *= -1.0;

        const Mat3 M = newK * R;
        const auto inv = M.inverse();
        t.matrices[r] = inv ? inv->m : Mat3::identity().m;
    }

    return t;
}

} // namespace gyroflow
