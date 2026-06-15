#include "gyroflow/lens_profile.hpp"
#include "gyroflow/stabilization/frame_transform.hpp"
#include "gyroflow/stabilization/undistort.hpp"
#include "gyroflow/types.hpp"

#include <cassert>
#include <cstdint>
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

    return 0;
}
