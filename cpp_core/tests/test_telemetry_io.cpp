#include "gyroflow/telemetry_io.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {
bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}
} // namespace

int main() {
    using namespace gyroflow;

    const std::string ok = R"JSON({
        "detected_source": "DJI Avata 2",
        "fps": 59.94,
        "width": 3840,
        "height": 2160,
        "frame_readout_time_ms": 8.5,
        "frame_readout_direction": "BottomToTop",
        "quaternions": [
            [0,       1.0, 0.0, 0.0, 0.0],
            [16683, 0.9998, 0.01, 0.0, 0.0],
            [33366, 0.9995, 0.02, 0.0, 0.0]
        ],
        "lens_profile": {
            "camera_brand": "DJI",
            "camera_model": "Avata 2",
            "calib_width": 3840,
            "calib_height": 2160,
            "camera_matrix": [[2000.0, 0.0, 1920.0], [0.0, 2001.0, 1080.0], [0.0, 0.0, 1.0]],
            "distortion_model": "opencv_fisheye",
            "distortion_coeffs": [-0.02, 0.003, -0.0007, 0.0001]
        }
    })JSON";

    const FileMetadata meta = loadTelemetryFromJsonString(ok);

    assert(meta.detected_source == "DJI Avata 2");
    assert(close(meta.fps, 59.94));
    assert(meta.width == 3840 && meta.height == 2160);
    assert(close(meta.frame_readout_time_ms, 8.5));
    assert(meta.frame_readout_direction == ReadoutDirection::BottomToTop);

    // Microsecond timestamps converted to milliseconds.
    assert(meta.quaternions.size() == 3);
    assert(close(meta.quaternions[0].timestamp_ms, 0.0));
    assert(close(meta.quaternions[1].timestamp_ms, 16.683));
    // Quaternions are normalized on load.
    assert(close(meta.quaternions[0].quat.norm(), 1.0));

    assert(meta.lens_profile.has_value());
    const LensProfile& lp = *meta.lens_profile;
    assert(lp.camera_brand == "DJI");
    assert(lp.distortion_model == DistortionModel::OpenCVFisheye);
    assert(close(lp.camera_matrix.fx, 2000.0));
    assert(close(lp.camera_matrix.fy, 2001.0));
    assert(close(lp.camera_matrix.cx, 1920.0));
    assert(close(lp.camera_matrix.cy, 1080.0));
    assert(lp.distortion_coeffs.size() == 4);
    assert(close(lp.distortion_coeffs[0], -0.02));
    // output_* defaults to calib_* when omitted.
    assert(lp.output_width == 3840 && lp.output_height == 2160);

    // Flat fx/fy/cx/cy form is also accepted.
    {
        const std::string flat = R"JSON({
            "fps": 30.0, "width": 1920, "height": 1080,
            "quaternions": [[0, 1, 0, 0, 0]],
            "lens_profile": { "fx": 1000, "fy": 1000, "cx": 960, "cy": 540,
                              "distortion_coeffs": [0,0,0,0] }
        })JSON";
        const FileMetadata m2 = loadTelemetryFromJsonString(flat);
        assert(close(m2.lens_profile->camera_matrix.cx, 960.0));
    }

    // Missing required fields must throw.
    bool threw = false;
    try {
        loadTelemetryFromJsonString(R"({"width":1,"height":1,"quaternions":[[0,1,0,0,0]]})");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw); // missing fps

    return 0;
}
