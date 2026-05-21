#include "gyroflow/lens_profile.hpp"
#include "gyroflow/stabilization.hpp"
#include "gyroflow/types.hpp"

#include <cassert>
#include <cmath>

namespace {

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    using namespace gyroflow;

    const Quaternion q0 = Quaternion::identity();
    const Quaternion q1 = Quaternion::fromAxisAngle({0.0, 0.0, 1.0}, M_PI);
    const Quaternion halfway = slerp(q0, q1, 0.5);
    assert(close(halfway.norm(), 1.0));

    std::vector<TimeQuat> raw = {
        {0.0, q0},
        {10.0, q1},
    };
    std::vector<TimeQuat> smooth = smoothQuaternionsPlain(raw, 100.0);
    assert(smooth.size() == raw.size());

    const auto samples = computeStabilizationSamples(raw, smooth);
    assert(samples.size() == raw.size());
    assert(close(samples.front().timestamp_ms, 0.0));

    const auto rows = computeRollingShutterRows(raw, smooth, 5.0, 4.0, 4, 1);
    assert(rows.size() == 4);
    assert(close(rows.front().timestamp_ms, 3.0));
    assert(close(rows.back().timestamp_ms, 6.0));

    const LensProfile profile = makeDefaultLensProfile(3840, 2880);
    assert(profile.distortion_model == DistortionModel::OpenCVFisheye);
    assert(close(profile.camera_matrix.fx, 3072.0));
    assert(close(profile.camera_matrix.cx, 1920.0));

    return 0;
}
