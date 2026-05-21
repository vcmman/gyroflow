#pragma once

#include <array>
#include <string>
#include <vector>

namespace gyroflow {

enum class DistortionModel {
    OpenCVFisheye,
    OpenCVStandard,
    Unknown,
};

struct CameraMatrix {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct LensProfile {
    std::string camera_brand;
    std::string camera_model;
    std::string lens_model;

    int calib_width = 0;
    int calib_height = 0;
    int output_width = 0;
    int output_height = 0;

    CameraMatrix camera_matrix;
    std::vector<double> distortion_coeffs;
    DistortionModel distortion_model = DistortionModel::OpenCVFisheye;

    double frame_readout_time_ms = 0.0;
};

DistortionModel distortionModelFromName(const std::string& name);
const char* distortionModelName(DistortionModel model);

LensProfile makeDefaultLensProfile(int width, int height);

} // namespace gyroflow
