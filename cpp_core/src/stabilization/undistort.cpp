#include "gyroflow/stabilization/undistort.hpp"

#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

#include "gyroflow/distortion/opencv_fisheye.hpp"

namespace gyroflow {

namespace {

// Bilinear sample of an interleaved 8-bit image. Returns false if the sample point is
// outside the image (the caller then writes the background colour).
inline bool sampleBilinear(const ImageBuffer& img, double u, double v,
                           std::uint8_t* out_pixel) {
    if (u < 0.0 || v < 0.0 || u > img.width - 1.0 || v > img.height - 1.0) {
        return false;
    }
    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const int x1 = std::min(x0 + 1, img.width - 1);
    const int y1 = std::min(y0 + 1, img.height - 1);
    const double fx = u - x0;
    const double fy = v - y0;

    const std::uint8_t* row0 = img.data + static_cast<std::size_t>(y0) * img.stride;
    const std::uint8_t* row1 = img.data + static_cast<std::size_t>(y1) * img.stride;
    const int ch = img.channels;
    const std::uint8_t* p00 = row0 + x0 * ch;
    const std::uint8_t* p01 = row0 + x1 * ch;
    const std::uint8_t* p10 = row1 + x0 * ch;
    const std::uint8_t* p11 = row1 + x1 * ch;

    const double w00 = (1.0 - fx) * (1.0 - fy);
    const double w01 = fx * (1.0 - fy);
    const double w10 = (1.0 - fx) * fy;
    const double w11 = fx * fy;

    for (int c = 0; c < ch; ++c) {
        const double val = p00[c] * w00 + p01[c] * w01 + p10[c] * w10 + p11[c] * w11;
        out_pixel[c] = static_cast<std::uint8_t>(std::lround(std::clamp(val, 0.0, 255.0)));
    }
    return true;
}

void processRows(const FrameTransform& t, const ImageBuffer& src, ImageBuffer& dst,
                 const std::array<std::uint8_t, 3>& bg, int y_begin, int y_end) {
    const int matrix_count = static_cast<int>(t.matrices.size());
    const int ch = dst.channels;

    for (int y = y_begin; y < y_end; ++y) {
        std::uint8_t* out_row = dst.data + static_cast<std::size_t>(y) * dst.stride;
        for (int x = 0; x < dst.width; ++x) {
            std::uint8_t* out_pixel = out_row + x * ch;

            int idx = t.horizontal_readout ? x : y;
            if (idx >= matrix_count) idx = matrix_count - 1;
            if (idx < 0) idx = 0;
            const std::array<double, 9>& m = t.matrices[idx];

            const double _x = x * m[0] + y * m[1] + m[2];
            const double _y = x * m[3] + y * m[4] + m[5];
            const double _w = x * m[6] + y * m[7] + m[8];

            bool ok = false;
            if (_w > 0.0) {
                auto uv = OpenCVFisheye::distortPoint(_x, _y, _w, t.k);
                const double u = uv.first * t.f[0] + t.c[0];
                const double v = uv.second * t.f[1] + t.c[1];
                ok = sampleBilinear(src, u, v, out_pixel);
            }
            if (!ok) {
                for (int c = 0; c < ch && c < 3; ++c) out_pixel[c] = bg[c];
                for (int c = 3; c < ch; ++c) out_pixel[c] = 255;
            }
        }
    }
}

} // namespace

void undistortFrame(const FrameTransform& transform, const ImageBuffer& src,
                    ImageBuffer& dst, const std::array<std::uint8_t, 3>& background,
                    int num_threads) {
    if (transform.matrices.empty() || dst.data == nullptr || src.data == nullptr) return;

    if (num_threads <= 0) {
        num_threads = static_cast<int>(std::thread::hardware_concurrency());
        if (num_threads <= 0) num_threads = 1;
    }
    num_threads = std::min(num_threads, std::max(1, dst.height));

    if (num_threads == 1) {
        processRows(transform, src, dst, background, 0, dst.height);
        return;
    }

    std::vector<std::thread> pool;
    pool.reserve(num_threads);
    const int chunk = (dst.height + num_threads - 1) / num_threads;
    for (int t = 0; t < num_threads; ++t) {
        const int y0 = t * chunk;
        const int y1 = std::min(y0 + chunk, dst.height);
        if (y0 >= y1) break;
        pool.emplace_back(processRows, std::cref(transform), std::cref(src), std::ref(dst),
                          std::cref(background), y0, y1);
    }
    for (auto& th : pool) th.join();
}

} // namespace gyroflow
