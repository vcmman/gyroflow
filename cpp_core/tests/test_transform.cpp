#include "gyroflow/lens_profile.hpp"
#include "gyroflow/stabilization/frame_transform.hpp"
#include "gyroflow/stabilization/undistort.hpp"
#include "gyroflow/types.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

int main() {
    using namespace gyroflow;

    // 8x6 BGR image with a deterministic gradient pattern.
    const int W = 8, H = 6, C = 3;
    std::vector<std::uint8_t> srcBuf(static_cast<std::size_t>(W) * H * C);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            std::uint8_t* p = srcBuf.data() + (y * W + x) * C;
            p[0] = static_cast<std::uint8_t>(10 + x * 7);
            p[1] = static_cast<std::uint8_t>(20 + y * 11);
            p[2] = static_cast<std::uint8_t>(x * 13 + y * 5);
        }
    }

    // Lens whose principal point is the image center and zero distortion => identity map.
    LensProfile lens;
    lens.calib_width = lens.output_width = W;
    lens.calib_height = lens.output_height = H;
    lens.camera_matrix = {5.0, 5.0, W / 2.0, H / 2.0};
    lens.distortion_coeffs = {0.0, 0.0, 0.0, 0.0};
    lens.distortion_model = DistortionModel::OpenCVFisheye;

    // Identity attitude => no rotation. Output must equal input.
    std::vector<TimeQuat> raw = {{0.0, Quaternion::identity()}, {100.0, Quaternion::identity()}};
    std::vector<TimeQuat> smooth = raw;

    TransformParams tp;
    tp.fov = 1.0;
    tp.frame_readout_time_ms = 0.0;  // no rolling shutter -> single matrix
    const FrameTransform t = computeFrameTransform(50.0, raw, smooth, lens, W, H, tp);
    assert(t.matrices.size() == 1);
    assert(t.horizontal_readout == false);

    std::vector<std::uint8_t> dstBuf(srcBuf.size(), 0);
    ImageBuffer src{srcBuf.data(), W, H, W * C, C};
    ImageBuffer dst{dstBuf.data(), W, H, W * C, C};
    undistortFrame(t, src, dst, {0, 0, 0}, 1);

    for (std::size_t i = 0; i < srcBuf.size(); ++i) {
        assert(srcBuf[i] == dstBuf[i]);  // identity transform reproduces the input exactly
    }

    // Rolling shutter on -> one matrix per scanline.
    TransformParams rs;
    rs.frame_readout_time_ms = 8.0;
    rs.frame_readout_direction = ReadoutDirection::TopToBottom;
    const FrameTransform trs = computeFrameTransform(50.0, raw, smooth, lens, W, H, rs);
    assert(static_cast<int>(trs.matrices.size()) == H);
    assert(trs.horizontal_readout == false);

    // ---- Output dimension != input dimension ----------------------------------------
    // Render a 16:9-style crop (output shorter than the sensor). The per-row matrix count
    // must stay in INPUT dims (one per source scanline), the source intrinsics unchanged,
    // and the reported output dims must reflect the crop.
    const int OW = 8, OH = 4;  // input 8x6 -> output 8x4
    TransformParams op;
    op.fov = 1.0;
    op.frame_readout_time_ms = 8.0;
    op.frame_readout_direction = ReadoutDirection::TopToBottom;
    op.output_width = OW;
    op.output_height = OH;
    // Use a real rotation so new_K * R is non-trivial.
    const Quaternion rot = Quaternion::fromAxisAngle({0.3, 1.0, 0.2}, 0.15);
    std::vector<TimeQuat> rraw = {{0.0, rot}, {100.0, rot}};
    std::vector<TimeQuat> rsm = {{0.0, Quaternion::identity()}, {100.0, Quaternion::identity()}};
    const FrameTransform ot = computeFrameTransform(50.0, rraw, rsm, lens, W, H, op);
    assert(ot.output_width == OW && ot.output_height == OH);
    assert(static_cast<int>(ot.matrices.size()) == H);  // matrices indexed by SOURCE rows
    assert(ot.f[0] == lens.camera_matrix.fx && ot.c[1] == lens.camera_matrix.cy);

    // fov-scaling identity with output centre != input centre: a point p rendered at fov=f
    // maps to the same source ray as the point (out_centre + (p - out_centre)*f) rendered at
    // fov=1, where out_centre = (OW/2, OH/2). (i_r_f = i_r_1 * T^-1, T scaling about centre.)
    auto buildIr = [&](double fovv) {
        TransformParams p = op;
        p.fov = fovv;
        p.frame_readout_time_ms = 0.0;  // single matrix, isolate the fov scaling
        return computeFrameTransform(50.0, rraw, rsm, lens, W, H, p).matrices[0];
    };
    auto ray = [](const std::array<double, 9>& m, double x, double y) {
        const double X = m[0] * x + m[1] * y + m[2];
        const double Y = m[3] * x + m[4] * y + m[5];
        const double Wd = m[6] * x + m[7] * y + m[8];
        return std::pair<double, double>{X / Wd, Y / Wd};
    };
    const std::array<double, 9> ir1 = buildIr(1.0);
    const std::array<double, 9> irf = buildIr(0.5);
    const double ocx = OW / 2.0, ocy = OH / 2.0;
    for (auto [px, py] : std::vector<std::pair<double, double>>{{1.0, 1.0}, {6.0, 3.0}, {0.0, 0.0}}) {
        const auto rf = ray(irf, px, py);
        const auto r1 = ray(ir1, ocx + (px - ocx) * 0.5, ocy + (py - ocy) * 0.5);
        assert(std::abs(rf.first - r1.first) < 1e-9);
        assert(std::abs(rf.second - r1.second) < 1e-9);
    }

    return 0;
}
