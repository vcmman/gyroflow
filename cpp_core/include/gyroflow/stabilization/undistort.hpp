#pragma once

#include <array>
#include <cstdint>

#include "gyroflow/stabilization/frame_transform.hpp"

namespace gyroflow {

// A view over a packed, interleaved 8-bit image buffer (e.g. BGR/RGB, `channels` per
// pixel). The core library stays free of OpenCV; the caller (video tool) owns the buffers.
struct ImageBuffer {
    std::uint8_t* data = nullptr;
    int width = 0;
    int height = 0;
    int stride = 0;     // bytes per row
    int channels = 3;   // bytes per pixel
};

// Apply the stabilization transform: for each output pixel, map through the per-row 3x3
// matrix, fisheye-distort, convert to source pixels, and bilinearly sample the input.
// Pixels that map behind the camera (w <= 0) or outside the source are filled with
// `background`. Ports rotate_and_distort + the bilinear branch of cpu_undistort.rs.
//
// num_threads <= 0 uses std::thread::hardware_concurrency().
void undistortFrame(const FrameTransform& transform, const ImageBuffer& src,
                    ImageBuffer& dst, const std::array<std::uint8_t, 3>& background,
                    int num_threads = 0);

} // namespace gyroflow
