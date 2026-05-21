#include "gyroflow/lens_profile.hpp"
#include "gyroflow/stabilization.hpp"
#include "gyroflow/types.hpp"

#include <iostream>
#include <vector>

int main() {
    using namespace gyroflow;

    const LensProfile profile = makeDefaultLensProfile(3840, 2880);
    std::vector<TimeQuat> raw = {
        {0.0, Quaternion::identity()},
        {16.6667, Quaternion::fromAxisAngle({0.0, 0.0, 1.0}, 0.02)},
    };
    const std::vector<TimeQuat> smooth = smoothQuaternionsPlain(raw, 250.0);
    const auto rows = computeRollingShutterRows(
        raw,
        smooth,
        16.6667,
        profile.frame_readout_time_ms,
        profile.calib_height,
        240);

    std::cout << "gyroflow_cpp_core probe\n";
    std::cout << "default distortion model: " << distortionModelName(profile.distortion_model) << "\n";
    std::cout << "camera matrix fx=" << profile.camera_matrix.fx
              << " fy=" << profile.camera_matrix.fy
              << " cx=" << profile.camera_matrix.cx
              << " cy=" << profile.camera_matrix.cy << "\n";
    std::cout << "rolling shutter row samples: " << rows.size() << "\n";
    return 0;
}
