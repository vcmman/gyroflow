// Headless DJI video stabilizer (Phase 1).
//
// Reads a DJI MP4 + a telemetry JSON sidecar (see telemetry_io.hpp), smooths the attitude
// with the default algorithm, builds per-frame rolling-shutter transforms, undistorts each
// frame on the CPU, and writes a stabilized MP4 via OpenCV.
//
// Usage:
//   gyroflow_cpp_stabilize <input.mp4> --telemetry <file.json> -o <output.mp4>
//                          [--fov 1.0] [--max-frames N] [--threads N]

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "gyroflow/smoothing/default_algo.hpp"
#include "gyroflow/stabilization/frame_transform.hpp"
#include "gyroflow/stabilization/undistort.hpp"
#include "gyroflow/telemetry_io.hpp"
#include "gyroflow/zooming/adaptive_zoom.hpp"

using namespace gyroflow;

namespace {

void usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " <input.mp4> --telemetry <file.json> -o <output.mp4>\n"
              << "  [--max-zoom 130] [--no-adaptive-zoom] [--fov 1.0]"
              << " [--max-frames N] [--threads N]\n"
              << "  Adaptive zoom is on by default; --fov sets a static zoom and disables it.\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string input, telemetry, output;
    double fov = 1.0;
    bool adaptive_zoom = true;
    double max_zoom = 130.0;
    long max_frames = 0;
    int threads = 0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--telemetry" || a == "-g") telemetry = next("--telemetry");
        else if (a == "-o" || a == "--output") output = next("-o");
        else if (a == "--no-adaptive-zoom") adaptive_zoom = false;
        else if (a == "--max-zoom") max_zoom = std::stod(next("--max-zoom"));
        else if (a == "--fov") { fov = std::stod(next("--fov")); adaptive_zoom = false; }
        else if (a == "--max-frames") max_frames = std::stol(next("--max-frames"));
        else if (a == "--threads") threads = std::stoi(next("--threads"));
        else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else if (!a.empty() && a[0] == '-') { std::cerr << "Unknown option: " << a << "\n"; return 2; }
        else input = a;
    }

    if (input.empty() || telemetry.empty() || output.empty()) {
        usage(argv[0]);
        return 2;
    }

    FileMetadata meta;
    try {
        meta = loadTelemetryFromJsonFile(telemetry);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load telemetry: " << e.what() << "\n";
        return 1;
    }
    if (!meta.lens_profile) {
        std::cerr << "Telemetry JSON has no lens_profile (required for undistortion)\n";
        return 1;
    }
    if (meta.quaternions.size() < 2) {
        std::cerr << "Telemetry JSON has too few quaternions\n";
        return 1;
    }
    cv::VideoCapture cap(input);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open input video: " << input << "\n";
        return 1;
    }
    const double fps = cap.get(cv::CAP_PROP_FPS);
    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const long total = static_cast<long>(cap.get(cv::CAP_PROP_FRAME_COUNT));

    // Phase 2 renders the full sensor: output == input dimensions. The lens profile's
    // output_dimension (e.g. Gyroflow's 16:9 crop of a 4:3 sensor) is intentionally
    // ignored so new_K's principal point, the undistort kernel's output buffer, and the
    // adaptive-zoom inscribed-rect all share one coordinate system. Matching Gyroflow's
    // cropped output_dimension is a follow-up.
    LensProfile lens = *meta.lens_profile;
    lens.output_width = width;
    lens.output_height = height;

    std::cout << "Input: " << width << "x" << height << " @ " << fps << " fps, "
              << total << " frames\n";
    std::cout << "Source: " << meta.detected_source << ", readout="
              << meta.frame_readout_time_ms << " ms\n";

    // Smooth the attitude (default algorithm).
    const double duration_ms =
        meta.quaternions.back().timestamp_ms - meta.quaternions.front().timestamp_ms;
    DefaultAlgoParams sp;
    const double diag = std::sqrt(static_cast<double>(width) * width +
                                  static_cast<double>(height) * height);
    sp.camera_diagonal_fov =
        2.0 * std::atan(diag / (2.0 * lens.camera_matrix.fy)) * 180.0 / 3.14159265358979323846;
    std::cout << "Smoothing " << meta.quaternions.size() << " quaternions (diag FOV "
              << sp.camera_diagonal_fov << " deg)...\n";
    const std::vector<TimeQuat> smoothed = smoothDefault(meta.quaternions, duration_ms, sp);

    cv::VideoWriter writer(output, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
                           cv::Size(width, height));
    if (!writer.isOpened()) {
        std::cerr << "Failed to open output writer: " << output << "\n";
        return 1;
    }

    TransformParams tp;
    tp.fov = fov;
    tp.frame_readout_time_ms = meta.frame_readout_time_ms;
    tp.frame_readout_direction = meta.frame_readout_direction;

    // Precompute per-frame adaptive zoom over the whole clip (the temporal smoothing needs
    // neighbouring frames even when only a subset is rendered).
    std::vector<double> fovs;
    if (adaptive_zoom && total > 0) {
        std::vector<double> ts_all(static_cast<std::size_t>(total));
        for (long j = 0; j < total; ++j) ts_all[j] = static_cast<double>(j) * 1000.0 / fps;
        AdaptiveZoomParams az;
        az.max_zoom_percent = max_zoom;
        std::cout << "Computing adaptive zoom (window " << az.window_s << " s, max zoom "
                  << max_zoom << "%)...\n";
        fovs = computeAdaptiveFovs(ts_all, meta.quaternions, smoothed, lens, width, height,
                                   fps, tp, az);
        double mn = 1e9, mx = -1e9;
        for (double v : fovs) { mn = std::min(mn, v); mx = std::max(mx, v); }
        std::cout << "Adaptive FOV range: [" << mn << ", " << mx << "] (zoom "
                  << (1.0 / mx) << "x .. " << (1.0 / mn) << "x)\n";
        if (const char* dp = std::getenv("DUMP_FOVS")) {
            FILE* fp = std::fopen(dp, "w");
            if (fp) { for (std::size_t j = 0; j < fovs.size(); ++j) std::fprintf(fp, "%zu,%.6f\n", j, fovs[j]); std::fclose(fp); }
        }
    } else {
        std::cout << "Static FOV: " << fov << "\n";
    }

    const std::array<std::uint8_t, 3> background{0, 0, 0};

    cv::Mat frame;
    cv::Mat out(height, width, CV_8UC3);
    long i = 0;
    while (cap.read(frame)) {
        if (frame.empty()) break;
        if (!frame.isContinuous()) frame = frame.clone();

        const double ts_ms = static_cast<double>(i) * 1000.0 / fps;
        tp.fov = (!fovs.empty() && i < static_cast<long>(fovs.size())) ? fovs[i] : fov;
        const FrameTransform t =
            computeFrameTransform(ts_ms, meta.quaternions, smoothed, lens, width, height, tp);

        ImageBuffer src{frame.data, width, height, static_cast<int>(frame.step), 3};
        ImageBuffer dst{out.data, width, height, static_cast<int>(out.step), 3};
        undistortFrame(t, src, dst, background, threads);

        writer.write(out);

        if ((i % 30) == 0) {
            std::printf("\rframe %ld%s ", i, total > 0 ? ("/" + std::to_string(total)).c_str() : "");
            std::fflush(stdout);
        }
        ++i;
        if (max_frames > 0 && i >= max_frames) break;
    }
    std::printf("\rProcessed %ld frames -> %s\n", i, output.c_str());

    writer.release();
    cap.release();
    return 0;
}
