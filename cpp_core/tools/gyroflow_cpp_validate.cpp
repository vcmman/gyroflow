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
    std::string zoom_method = "envelope";  // envelope (1, default) | gaussian (0)
    double zoom_look_ahead = -1.0;         // <0 = offline; >=0 = real-time FOV look-ahead (s)
    bool per_axis = false;
    double sm_pitch = 0.5, sm_yaw = 0.5, sm_roll = 0.5;
    double master_smoothness = 0.5;
    double dev_clamp = 0.0, dev_clamp_soft = 0.0, dev_ref_tau = 0.03;
    bool dcr = false;
    double dcr_window = 0.5, dcr_power = 1.0;
    double look_ahead = 0.0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* n) -> std::string {
            if (i + 1 >= argc) { std::cerr << "Missing value for " << n << "\n"; std::exit(2); }
            return argv[++i];
        };
        if (a == "--frames") frames = std::stol(next("--frames"));
        else if (a == "--max-zoom") max_zoom = std::stod(next("--max-zoom"));
        else if (a == "--zoom-method") zoom_method = next("--zoom-method");
        else if (a == "--zoom-look-ahead") zoom_look_ahead = std::stod(next("--zoom-look-ahead"));
        else if (a == "--smoothness") master_smoothness = std::stod(next("--smoothness"));
        else if (a == "--deviation-clamp") dev_clamp = std::stod(next("--deviation-clamp"));
        else if (a == "--deviation-clamp-soft") dev_clamp_soft = std::stod(next("--deviation-clamp-soft"));
        else if (a == "--deviation-clamp-ref-tau") dev_ref_tau = std::stod(next("--deviation-clamp-ref-tau"));
        else if (a == "--per-axis") per_axis = true;
        else if (a == "--smoothness-pitch") sm_pitch = std::stod(next("--smoothness-pitch"));
        else if (a == "--smoothness-yaw") sm_yaw = std::stod(next("--smoothness-yaw"));
        else if (a == "--smoothness-roll") sm_roll = std::stod(next("--smoothness-roll"));
        else if (a == "--dcr") dcr = true;
        else if (a == "--dcr-window") dcr_window = std::stod(next("--dcr-window"));
        else if (a == "--dcr-power") dcr_power = std::stod(next("--dcr-power"));
        else if (a == "--look-ahead") look_ahead = std::stod(next("--look-ahead"));
        else if (a == "--enhanced") dcr = true;  // recommended preset (SMOOTHING_RND §8e) = DCR on
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
        std::cerr << "Usage: gyroflow_cpp_validate <bridge.json> [--frames N] [--max-zoom 130]\n"
                  << "  Smoothing: [--enhanced] [--dcr [--dcr-window 0.5] [--dcr-power 1.0]]\n"
                  << "             [--per-axis --smoothness-pitch/-yaw/-roll 0..1] [--look-ahead 0]\n"
                  << "  Zoom:      [--zoom-method envelope|gaussian] [--zoom-look-ahead -1]\n"
                  << "  Framing:   [--keep-sensor] [--output-size WxH]\n";
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
    sp.smoothness = master_smoothness;
    sp.deviation_clamp_deg = dev_clamp;
    sp.deviation_clamp_soft_deg = dev_clamp_soft;
    sp.deviation_clamp_ref_tau_s = dev_ref_tau;
    sp.per_axis = per_axis;
    sp.smoothness_pitch = sm_pitch;
    sp.smoothness_yaw = sm_yaw;
    sp.smoothness_roll = sm_roll;
    sp.dcr = dcr;
    sp.dcr_window_s = dcr_window;
    sp.dcr_power = dcr_power;
    sp.look_ahead_s = look_ahead;
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
    az.method = (zoom_method == "gaussian" || zoom_method == "0")
                    ? ZoomMethod::GaussianFilter
                    : ZoomMethod::EnvelopeFollower;
    az.look_ahead_s = zoom_look_ahead;
    if (az.method == ZoomMethod::GaussianFilter && zoom_look_ahead >= 0.0)
        std::cerr << "Warning: --zoom-look-ahead applies to the envelope method only; "
                     "ignored with --zoom-method gaussian\n";
    std::vector<double> raw_fovs;
    const std::vector<double> fovs =
        computeAdaptiveFovs(ts_all, meta.quaternions, smoothed, lens, width, height, fps, tp, az,
                            &raw_fovs);

    std::printf("frame,ts_ms,ow,ox,oy,oz,sw,sx,sy,sz,fov,raw_fov\n");
    for (long j = 0; j < frames; ++j) {
        const double ts = static_cast<double>(j) * 1000.0 / fps + readout / 2.0;
        const Quaternion o = sampleQuaternion(meta.quaternions, ts);
        const Quaternion s = sampleQuaternion(smoothed, ts);
        const double fov = (j < static_cast<long>(fovs.size())) ? fovs[j] : 1.0;
        const double raw_fov = (j < static_cast<long>(raw_fovs.size())) ? raw_fovs[j] : 1.0;
        std::printf("%ld,%.4f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.6f,%.6f\n", j, ts, o.w, o.x,
                    o.y, o.z, s.w, s.x, s.y, s.z, fov, raw_fov);
    }
    return 0;
}
