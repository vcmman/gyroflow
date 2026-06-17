// Validation dumper: emits the smoothed quaternions and adaptive FOVs that the C++ core
// computes, so they can be diffed against Rust Gyroflow's `--export-metadata 3:...` output
// (org_quat / stab_quat / fov_scale). No OpenCV / video needed — everything comes from the
// telemetry bridge JSON.
//
// Conventions matched to Gyroflow (so the same frame index lines up):
//   * adaptive FOV per frame is computed at ts = frame*1000/fps   (recompute_adaptive_zoom_static)
//   * quaternions are sampled at ts = frame*1000/fps + readout/2  (gyro_export: middle_timestamp)
//   * Gyroflow's exported stab_quat = org * smoothed^-1, so the smoothed orientation is
//     recovered as smoothed = stab_quat^-1 * org. This tool emits the raw `smoothed` series
//     sampled directly, plus org, so the Python side can apply the same relation.
//
// Usage:
//   gyroflow_cpp_validate <bridge.json> [--frames N] [--max-zoom 130] [--keep-sensor]
//                         [--output-size WxH]
// Output: CSV to stdout: frame,ts_ms,ow,ox,oy,oz,sw,sx,sy,sz,fov

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "gyroflow/smoothing/default_algo.hpp"
#include "gyroflow/stabilization/frame_transform.hpp"
#include "gyroflow/telemetry_io.hpp"
#include "gyroflow/zooming/adaptive_zoom.hpp"

using namespace gyroflow;

int main(int argc, char** argv) {
    std::string bridge;
    long frames = 0;
    double max_zoom = 130.0;
    bool keep_sensor = false;
    int out_w_override = 0, out_h_override = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* n) -> std::string {
            if (i + 1 >= argc) { std::cerr << "Missing value for " << n << "\n"; std::exit(2); }
            return argv[++i];
        };
        if (a == "--frames") frames = std::stol(next("--frames"));
        else if (a == "--max-zoom") max_zoom = std::stod(next("--max-zoom"));
        else if (a == "--keep-sensor") keep_sensor = true;
        else if (a == "--output-size") {
            const std::string s = next("--output-size");
            const std::size_t xp = s.find_first_of("xX*");
            out_w_override = std::stoi(s.substr(0, xp));
            out_h_override = std::stoi(s.substr(xp + 1));
        } else if (!a.empty() && a[0] != '-') bridge = a;
        else { std::cerr << "Unknown option: " << a << "\n"; return 2; }
    }
    if (bridge.empty()) {
        std::cerr << "Usage: gyroflow_cpp_validate <bridge.json> [--frames N] [--max-zoom 130]"
                  << " [--keep-sensor] [--output-size WxH]\n";
        return 2;
    }

    FileMetadata meta;
    try {
        meta = loadTelemetryFromJsonFile(bridge);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load telemetry: " << e.what() << "\n";
        return 1;
    }
    if (!meta.lens_profile || meta.quaternions.size() < 2) {
        std::cerr << "Telemetry missing lens_profile or quaternions\n";
        return 1;
    }
    const LensProfile& lens = *meta.lens_profile;
    const int width = meta.width, height = meta.height;
    const double fps = meta.fps;
    const double readout = meta.frame_readout_time_ms;
    if (width <= 0 || height <= 0 || fps <= 0.0) {
        std::cerr << "Telemetry missing width/height/fps\n";
        return 1;
    }
    if (frames <= 0) frames = static_cast<long>(std::llround(
        (meta.quaternions.back().timestamp_ms) / 1000.0 * fps));

    // Output framing (default: lens output_dimension, matching Gyroflow).
    int out_w = lens.output_width > 0 ? lens.output_width : width;
    int out_h = lens.output_height > 0 ? lens.output_height : height;
    if (keep_sensor) { out_w = width; out_h = height; }
    if (out_w_override > 0 && out_h_override > 0) { out_w = out_w_override; out_h = out_h_override; }

    // Smooth the attitude with the same parameters as the stabilizer CLI.
    const double duration_ms =
        meta.quaternions.back().timestamp_ms - meta.quaternions.front().timestamp_ms;
    DefaultAlgoParams sp;
    const double diag = std::sqrt(static_cast<double>(width) * width +
                                  static_cast<double>(height) * height);
    sp.camera_diagonal_fov =
        2.0 * std::atan(diag / (2.0 * lens.camera_matrix.fy)) * 180.0 / 3.14159265358979323846;
    const std::vector<TimeQuat> smoothed = smoothDefault(meta.quaternions, duration_ms, sp);

    // Adaptive FOVs over [0, frames), at ts = frame*1000/fps (Gyroflow convention).
    TransformParams tp;
    tp.fov = 1.0;
    tp.frame_readout_time_ms = readout;
    tp.frame_readout_direction = meta.frame_readout_direction;
    tp.output_width = out_w;
    tp.output_height = out_h;

    std::vector<double> ts_all(static_cast<std::size_t>(frames));
    for (long j = 0; j < frames; ++j) ts_all[j] = static_cast<double>(j) * 1000.0 / fps;
    AdaptiveZoomParams az;
    az.max_zoom_percent = max_zoom;
    const std::vector<double> fovs =
        computeAdaptiveFovs(ts_all, meta.quaternions, smoothed, lens, width, height, fps, tp, az);

    std::printf("frame,ts_ms,ow,ox,oy,oz,sw,sx,sy,sz,fov\n");
    for (long j = 0; j < frames; ++j) {
        const double ts = static_cast<double>(j) * 1000.0 / fps + readout / 2.0;
        const Quaternion o = sampleQuaternion(meta.quaternions, ts);
        const Quaternion s = sampleQuaternion(smoothed, ts);
        const double fov = (j < static_cast<long>(fovs.size())) ? fovs[j] : 1.0;
        std::printf("%ld,%.4f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.6f\n", j, ts, o.w, o.x,
                    o.y, o.z, s.w, s.x, s.y, s.z, fov);
    }
    return 0;
}
