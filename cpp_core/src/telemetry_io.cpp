#include "gyroflow/telemetry_io.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace gyroflow {

using nlohmann::json;

namespace {

ReadoutDirection parseReadoutDirection(const std::string& s) {
    if (s == "TopToBottom") return ReadoutDirection::TopToBottom;
    if (s == "BottomToTop") return ReadoutDirection::BottomToTop;
    if (s == "LeftToRight") return ReadoutDirection::LeftToRight;
    if (s == "RightToLeft") return ReadoutDirection::RightToLeft;
    throw std::runtime_error("Unknown frame_readout_direction: " + s);
}

LensProfile parseLensProfile(const json& lp) {
    LensProfile profile;

    profile.camera_brand = lp.value("camera_brand", std::string{});
    profile.camera_model = lp.value("camera_model", std::string{});
    profile.lens_model = lp.value("lens_model", std::string{});

    profile.calib_width = lp.value("calib_width", 0);
    profile.calib_height = lp.value("calib_height", 0);
    profile.output_width = lp.value("output_width", profile.calib_width);
    profile.output_height = lp.value("output_height", profile.calib_height);

    // Intrinsics: prefer a 3x3 camera_matrix (Gyroflow fisheye_params layout), else fall
    // back to flat fx/fy/cx/cy keys.
    if (lp.contains("camera_matrix")) {
        const json& m = lp.at("camera_matrix");
        if (!m.is_array() || m.size() < 3) {
            throw std::runtime_error("lens_profile.camera_matrix must be a 3x3 array");
        }
        profile.camera_matrix.fx = m.at(0).at(0).get<double>();
        profile.camera_matrix.cx = m.at(0).at(2).get<double>();
        profile.camera_matrix.fy = m.at(1).at(1).get<double>();
        profile.camera_matrix.cy = m.at(1).at(2).get<double>();
    } else {
        profile.camera_matrix.fx = lp.value("fx", 0.0);
        profile.camera_matrix.fy = lp.value("fy", 0.0);
        profile.camera_matrix.cx = lp.value("cx", 0.0);
        profile.camera_matrix.cy = lp.value("cy", 0.0);
    }

    profile.distortion_model =
        distortionModelFromName(lp.value("distortion_model", std::string{}));

    if (lp.contains("distortion_coeffs")) {
        profile.distortion_coeffs =
            lp.at("distortion_coeffs").get<std::vector<double>>();
    }

    profile.frame_readout_time_ms = lp.value("frame_readout_time_ms", 0.0);

    return profile;
}

FileMetadata parse(const json& root) {
    FileMetadata meta;

    meta.detected_source = root.value("detected_source", std::string{});

    if (!root.contains("fps")) throw std::runtime_error("telemetry JSON missing 'fps'");
    meta.fps = root.at("fps").get<double>();

    if (!root.contains("width") || !root.contains("height")) {
        throw std::runtime_error("telemetry JSON missing 'width'/'height'");
    }
    meta.width = root.at("width").get<int>();
    meta.height = root.at("height").get<int>();

    meta.frame_readout_time_ms = root.value("frame_readout_time_ms", 0.0);
    if (root.contains("frame_readout_direction")) {
        meta.frame_readout_direction =
            parseReadoutDirection(root.at("frame_readout_direction").get<std::string>());
    }

    if (!root.contains("quaternions") || !root.at("quaternions").is_array() ||
        root.at("quaternions").empty()) {
        throw std::runtime_error("telemetry JSON missing non-empty 'quaternions'");
    }
    const json& quats = root.at("quaternions");
    meta.quaternions.reserve(quats.size());
    for (const json& q : quats) {
        if (!q.is_array() || q.size() < 5) {
            throw std::runtime_error(
                "each quaternion entry must be [t_us, w, x, y, z]");
        }
        TimeQuat tq;
        // Stored as microseconds in the bridge; core works in milliseconds.
        tq.timestamp_ms = q.at(0).get<double>() / 1000.0;
        tq.quat.w = q.at(1).get<double>();
        tq.quat.x = q.at(2).get<double>();
        tq.quat.y = q.at(3).get<double>();
        tq.quat.z = q.at(4).get<double>();
        tq.quat = tq.quat.normalized();
        meta.quaternions.push_back(tq);
    }

    if (root.contains("lens_profile")) {
        meta.lens_profile = parseLensProfile(root.at("lens_profile"));
        // Prefer the readout time from telemetry; fall back to the lens profile.
        if (meta.frame_readout_time_ms == 0.0) {
            meta.frame_readout_time_ms = meta.lens_profile->frame_readout_time_ms;
        }
    }

    return meta;
}

} // namespace

FileMetadata loadTelemetryFromJsonString(const std::string& jsonStr) {
    json root;
    try {
        root = json::parse(jsonStr);
    } catch (const json::parse_error& e) {
        throw std::runtime_error(std::string("Failed to parse telemetry JSON: ") +
                                 e.what());
    }
    return parse(root);
}

FileMetadata loadTelemetryFromJsonFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Failed to open telemetry JSON file: " + path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return loadTelemetryFromJsonString(ss.str());
}

} // namespace gyroflow
