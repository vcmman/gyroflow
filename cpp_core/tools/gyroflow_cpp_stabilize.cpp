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
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
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

bool ffmpegAvailable(const std::string& bin) {
    return std::system((bin + " -version >/dev/null 2>&1").c_str()) == 0;
}

// Spawns ffmpeg reading raw BGR24 frames from a pipe and encoding to `output`. When
// `input` is given and `audio` is set, ffmpeg muxes the original's audio track.
FILE* openFfmpegPipe(const std::string& bin, const std::string& output,
                     const std::string& input, int w, int h, double fps,
                     const std::string& codec, int crf, bool audio) {
    const std::string venc = (codec == "h265" || codec == "hevc") ? "libx265" : "libx264";
    std::ostringstream cmd;
    cmd << "'" << bin << "' -y -hide_banner -loglevel error"
        << " -f rawvideo -pixel_format bgr24"
        << " -video_size " << w << "x" << h
        << " -framerate " << fps << " -i -";
    const bool with_audio = audio && !input.empty();
    if (with_audio) cmd << " -i '" << input << "'";
    cmd << " -map 0:v:0";
    if (with_audio) cmd << " -map 1:a:0? -c:a copy -shortest";
    cmd << " -c:v " << venc << " -preset medium -crf " << crf
        << " -pix_fmt yuv420p -movflags +faststart"
        << " '" << output << "'";
    return popen(cmd.str().c_str(), "w");
}

void usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " <input.mp4> --telemetry <file.json> -o <output.mp4>\n"
              << "  Stabilization: [--max-zoom 130] [--zoom-method envelope|gaussian]"
              << " [--zoom-look-ahead -1]  (>=0 = in-camera FOV look-ahead s; smooth zoom, no pops)"
              << " [--no-adaptive-zoom] [--fov 1.0]\n"
              << "                 [--per-axis --smoothness-pitch/-yaw/-roll 0..1]\n"
              << "                 [--enhanced]  (recommended preset: DCR on; -31..35% vertical shake)\n"
              << "                 [--dcr [--dcr-window 0.5] [--dcr-power 1.0]]"
              << "  (direction-consistency gate; keeps smoothing on reciprocating shake)\n"
              << "                 [--look-ahead 0]  (>0 = in-camera finite future buffer, s;"
              << " 0 = offline)\n"
              << "  Framing:       [--keep-sensor] [--output-size WxH]"
              << " (default: lens output_dimension)\n"
              << "  Encoding:      [--codec h264|h265] [--crf 18] [--no-audio]"
              << " [--no-ffmpeg] [--ffmpeg-bin PATH]\n"
              << "  Misc:          [--max-frames N] [--threads N]\n"
              << "  Adaptive zoom is on by default; --fov sets a static zoom and disables it.\n"
              << "  ffmpeg (H.264/H.265 + audio copy) is used when available; --no-ffmpeg uses OpenCV mp4v.\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string input, telemetry, output;
    double fov = 1.0;
    bool adaptive_zoom = true;
    double max_zoom = 130.0;
    std::string zoom_method = "envelope";  // envelope (1, default) | gaussian (0)
    double zoom_look_ahead = -1.0;         // <0 = offline; >=0 = real-time FOV look-ahead (s)
    bool per_axis = false;
    double sm_pitch = 0.5, sm_yaw = 0.5, sm_roll = 0.5;
    double master_smoothness = 0.5, dev_clamp = 0.0;
    bool dcr = false;
    double dcr_window = 0.5, dcr_power = 1.0;
    double look_ahead = 0.0;   // 0 = offline; >0 = in-camera finite look-ahead (s)
    long max_frames = 0;
    int threads = 0;

    // Output framing. By default the rendered size matches Gyroflow's lens output_dimension
    // (e.g. a 16:9 crop of a 4:3 sensor). --keep-sensor renders the full input sensor;
    // --output-size WxH overrides both.
    bool keep_sensor = false;
    int out_w_override = 0;
    int out_h_override = 0;

    // Encoder options. ffmpeg is used when available (proper H.264/H.265 compression and
    // audio passthrough); --no-ffmpeg falls back to OpenCV's mp4v writer.
    bool use_ffmpeg = true;
    std::string ffmpeg_bin = "ffmpeg";
    std::string codec = "h264";  // h264 (libx264) or h265 (libx265)
    int crf = 18;
    bool audio = true;  // copy audio track from the input

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
        else if (a == "--zoom-method") zoom_method = next("--zoom-method");
        else if (a == "--zoom-look-ahead") zoom_look_ahead = std::stod(next("--zoom-look-ahead"));
        else if (a == "--smoothness") master_smoothness = std::stod(next("--smoothness"));
        else if (a == "--deviation-clamp") dev_clamp = std::stod(next("--deviation-clamp"));
        else if (a == "--per-axis") per_axis = true;
        else if (a == "--smoothness-pitch") sm_pitch = std::stod(next("--smoothness-pitch"));
        else if (a == "--smoothness-yaw") sm_yaw = std::stod(next("--smoothness-yaw"));
        else if (a == "--smoothness-roll") sm_roll = std::stod(next("--smoothness-roll"));
        else if (a == "--dcr") dcr = true;
        else if (a == "--dcr-window") dcr_window = std::stod(next("--dcr-window"));
        else if (a == "--dcr-power") dcr_power = std::stod(next("--dcr-power"));
        else if (a == "--look-ahead") look_ahead = std::stod(next("--look-ahead"));
        // Recommended stabilization preset (SMOOTHING_RND §8e): the validated Tier-1 stack.
        // Currently = DCR on (−31…35% rendered vertical shake, black border ~unchanged from
        // default, +8…13% crop). Per-axis vertical smoothing was evaluated and *excluded* — it
        // only helped one clip and forced black borders / more crop. Keeps the golden default
        // (scalar, dcr off) intact; explicit flags after --enhanced still override.
        else if (a == "--enhanced") dcr = true;
        else if (a == "--fov") { fov = std::stod(next("--fov")); adaptive_zoom = false; }
        else if (a == "--max-frames") max_frames = std::stol(next("--max-frames"));
        else if (a == "--threads") threads = std::stoi(next("--threads"));
        else if (a == "--keep-sensor") keep_sensor = true;
        else if (a == "--output-size") {
            const std::string s = next("--output-size");
            const std::size_t xpos = s.find_first_of("xX*");
            if (xpos == std::string::npos) {
                std::cerr << "--output-size expects WxH, got: " << s << "\n";
                return 2;
            }
            out_w_override = std::stoi(s.substr(0, xpos));
            out_h_override = std::stoi(s.substr(xpos + 1));
        }
        else if (a == "--no-ffmpeg") use_ffmpeg = false;
        else if (a == "--ffmpeg-bin") ffmpeg_bin = next("--ffmpeg-bin");
        else if (a == "--codec") codec = next("--codec");
        else if (a == "--crf") crf = std::stoi(next("--crf"));
        else if (a == "--no-audio") audio = false;
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

    LensProfile lens = *meta.lens_profile;

    // Resolve the rendered output framing. Default: match Gyroflow and crop to the lens
    // output_dimension (e.g. 3840x2160 16:9 from a 3840x2880 4:3 sensor). --keep-sensor
    // renders the full input sensor; --output-size overrides both. These dims drive new_K's
    // principal point, the inscribed-rect aspect, and the output buffer, while the source
    // intrinsics and per-row matrix count stay in input dims.
    int out_w = lens.output_width > 0 ? lens.output_width : width;
    int out_h = lens.output_height > 0 ? lens.output_height : height;
    if (keep_sensor) { out_w = width; out_h = height; }
    if (out_w_override > 0 && out_h_override > 0) {
        out_w = out_w_override;
        out_h = out_h_override;
    }

    std::cout << "Input: " << width << "x" << height << " @ " << fps << " fps, "
              << total << " frames\n";
    std::cout << "Output: " << out_w << "x" << out_h
              << (out_w == width && out_h == height ? " (full sensor)" : " (cropped)") << "\n";
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
    sp.smoothness = master_smoothness;
    sp.deviation_clamp_deg = dev_clamp;
    sp.per_axis = per_axis;
    sp.smoothness_pitch = sm_pitch;
    sp.smoothness_yaw = sm_yaw;
    sp.smoothness_roll = sm_roll;
    sp.dcr = dcr;
    sp.dcr_window_s = dcr_window;
    sp.dcr_power = dcr_power;
    sp.look_ahead_s = look_ahead;
    std::cout << "Smoothing " << meta.quaternions.size() << " quaternions (diag FOV "
              << sp.camera_diagonal_fov << " deg"
              << (dcr ? (", DCR gate on (window " + std::to_string(dcr_window) + " s, power " +
                         std::to_string(dcr_power) + ")")
                      : std::string())
              << (look_ahead > 0.0 ? (", look-ahead " + std::to_string(look_ahead) + " s (in-camera)")
                                   : std::string(", offline (unlimited look-ahead)"))
              << ")...\n";
    const std::vector<TimeQuat> smoothed = smoothDefault(meta.quaternions, duration_ms, sp);

    // Choose the encoder: ffmpeg pipe (H.264/H.265 + audio) when available, else OpenCV.
    const bool ffmpeg_ok = use_ffmpeg && ffmpegAvailable(ffmpeg_bin);
    FILE* ff_pipe = nullptr;
    cv::VideoWriter writer;
    if (ffmpeg_ok) {
        ff_pipe = openFfmpegPipe(ffmpeg_bin, output, input, out_w, out_h, fps, codec, crf, audio);
        if (!ff_pipe) {
            std::cerr << "Failed to start ffmpeg pipe\n";
            return 1;
        }
        std::cout << "Encoder: ffmpeg " << ((codec == "h265" || codec == "hevc") ? "libx265" : "libx264")
                  << " crf " << crf << (audio ? " + audio copy" : "") << "\n";
    } else {
        if (use_ffmpeg) std::cout << "ffmpeg not found, falling back to OpenCV mp4v writer\n";
        writer.open(output, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, cv::Size(out_w, out_h));
        if (!writer.isOpened()) {
            std::cerr << "Failed to open output writer: " << output << "\n";
            return 1;
        }
    }

    TransformParams tp;
    tp.fov = fov;
    tp.frame_readout_time_ms = meta.frame_readout_time_ms;
    tp.frame_readout_direction = meta.frame_readout_direction;
    tp.output_width = out_w;
    tp.output_height = out_h;

    // Precompute per-frame adaptive zoom over the whole clip (the temporal smoothing needs
    // neighbouring frames even when only a subset is rendered).
    std::vector<double> fovs;
    if (adaptive_zoom && total > 0) {
        std::vector<double> ts_all(static_cast<std::size_t>(total));
        for (long j = 0; j < total; ++j) ts_all[j] = static_cast<double>(j) * 1000.0 / fps;
        AdaptiveZoomParams az;
        az.max_zoom_percent = max_zoom;
        az.method = (zoom_method == "gaussian" || zoom_method == "0")
                        ? ZoomMethod::GaussianFilter
                        : ZoomMethod::EnvelopeFollower;
        az.look_ahead_s = zoom_look_ahead;
        if (az.method == ZoomMethod::GaussianFilter && zoom_look_ahead >= 0.0)
            std::cerr << "Warning: --zoom-look-ahead applies to the envelope method only; "
                         "ignored with --zoom-method gaussian\n";
        std::cout << "Computing adaptive zoom (window " << az.window_s << " s, max zoom "
                  << max_zoom << "%, method "
                  << (az.method == ZoomMethod::GaussianFilter ? "gaussian" : "envelope")
                  << ")...\n";
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
    cv::Mat out(out_h, out_w, CV_8UC3);
    long i = 0;
    while (cap.read(frame)) {
        if (frame.empty()) break;
        if (!frame.isContinuous()) frame = frame.clone();

        const double ts_ms = static_cast<double>(i) * 1000.0 / fps;
        tp.fov = (!fovs.empty() && i < static_cast<long>(fovs.size())) ? fovs[i] : fov;
        const FrameTransform t =
            computeFrameTransform(ts_ms, meta.quaternions, smoothed, lens, width, height, tp);

        ImageBuffer src{frame.data, width, height, static_cast<int>(frame.step), 3};
        ImageBuffer dst{out.data, out_w, out_h, static_cast<int>(out.step), 3};
        undistortFrame(t, src, dst, background, threads);

        if (ff_pipe) {
            // Write tightly-packed BGR24 rows (skip any cv::Mat row padding).
            for (int y = 0; y < out_h; ++y) {
                if (std::fwrite(out.ptr(y), 1, static_cast<std::size_t>(out_w) * 3, ff_pipe) !=
                    static_cast<std::size_t>(out_w) * 3) {
                    std::cerr << "\nffmpeg pipe write failed (frame " << i << ")\n";
                    pclose(ff_pipe);
                    return 1;
                }
            }
        } else {
            writer.write(out);
        }

        if ((i % 30) == 0) {
            std::printf("\rframe %ld%s ", i, total > 0 ? ("/" + std::to_string(total)).c_str() : "");
            std::fflush(stdout);
        }
        ++i;
        if (max_frames > 0 && i >= max_frames) break;
    }
    std::printf("\rProcessed %ld frames -> %s\n", i, output.c_str());

    if (ff_pipe) {
        const int rc = pclose(ff_pipe);
        if (rc != 0) { std::cerr << "ffmpeg exited with code " << rc << "\n"; return 1; }
    } else {
        writer.release();
    }
    cap.release();
    return 0;
}
