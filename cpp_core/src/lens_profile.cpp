#include "gyroflow/lens_profile.hpp"

namespace gyroflow {

DistortionModel distortionModelFromName(const std::string& name) {
    if (name.empty() || name == "opencv_fisheye") {
        return DistortionModel::OpenCVFisheye;
    }
    if (name == "opencv_standard") {
        return DistortionModel::OpenCVStandard;
    }
    return DistortionModel::Unknown;
}

const char* distortionModelName(DistortionModel model) {
    switch (model) {
        case DistortionModel::OpenCVFisheye:
            return "opencv_fisheye";
        case DistortionModel::OpenCVStandard:
            return "opencv_standard";
        case DistortionModel::Unknown:
            return "unknown";
    }
    return "unknown";
}

LensProfile makeDefaultLensProfile(int width, int height) {
    LensProfile profile;
    profile.calib_width = width;
    profile.calib_height = height;
    profile.output_width = width;
    profile.output_height = height;
    profile.camera_matrix.fx = static_cast<double>(width) * 0.8;
    profile.camera_matrix.fy = static_cast<double>(width) * 0.8;
    profile.camera_matrix.cx = static_cast<double>(width) / 2.0;
    profile.camera_matrix.cy = static_cast<double>(height) / 2.0;
    profile.distortion_model = DistortionModel::OpenCVFisheye;
    return profile;
}

} // namespace gyroflow
