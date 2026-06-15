// gyroflow_autosync — UI-free C++ "autosync time" tool + timestamp-sync accuracy evaluator.
//
// Loads DJI fused-quaternion telemetry (extracted from a DJI MP4), derives an angular-velocity
// signal from the quaternions, and runs Gyroflow's time-offset finder to report the timestamp
// delay (ms) between two motion signals.
//
// Modes:
//   selftest  Inject known time offsets into a video-rate copy of the signal and measure how
//             accurately they are recovered -> timestamp-sync precision (ground-truthed).
//   compare   Find the offset between the full-rate signal and a video-rate resample of it
//             (true offset 0) -> baseline bias floor.
//   omega     Dump the quaternion-derived angular velocity as CSV (cross-check vs analysis CSV).
//
// Build: part of cpp_core (see CMakeLists.txt). Example:
//   ./gyroflow_autosync selftest --quat ../../data/dji_quaternions_full.csv --fps 30 --noise 1.5

#include "gyroflow/timesync.hpp"
#include "gyroflow/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace gyroflow;

namespace {

struct Args {
    std::string mode;
    std::string quat_path;
    double fps = 30.0;
    double search = 200.0;
    double initial = 0.0;
    double lpf = 20.0;
    bool swap_xy = false;
    double noise = 0.0;
    std::vector<double> inject = {-30.0, -15.0, -5.0, 0.0, 5.0, 15.0, 30.0};
    double range_a = 0.0;
    double range_b = 0.0; // 0,0 => whole signal
};

std::vector<double> parseList(const std::string& s) {
    std::vector<double> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) out.push_back(std::atof(tok.c_str()));
    }
    return out;
}

void usage() {
    std::cerr <<
        "Usage: gyroflow_autosync <selftest|compare|omega> --quat <csv> [options]\n"
        "  --quat PATH      DJI quaternion CSV (dji_quaternions_full.csv or dji_camera_data.csv)\n"
        "  --fps F          video frame rate for the synthesised video-side signal (default 30)\n"
        "  --search MS      offset search size in ms (default 200)\n"
        "  --initial MS     initial/rough offset in ms (default 0)\n"
        "  --lpf HZ         low-pass cutoff in Hz (default 20)\n"
        "  --swap-xy        swap x/y when deriving angular velocity (Gyroflow estimated_gyro convention)\n"
        "  --inject LIST    selftest: comma-separated injected offsets in ms (default -30,-15,-5,0,5,15,30)\n"
        "  --noise SIGMA    selftest: Gaussian noise stddev (deg/s) added to the video signal (default 0)\n"
        "  --range A,B      analyse only timestamps in [A,B] ms (default whole signal)\n";
}

// Split a CSV line into trimmed fields.
std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string f;
    while (std::getline(ss, f, ',')) {
        while (!f.empty() && (f.back() == '\r' || f.back() == ' ')) f.pop_back();
        out.push_back(f);
    }
    return out;
}

int indexOf(const std::vector<std::string>& header, const std::string& name) {
    for (std::size_t i = 0; i < header.size(); ++i) if (header[i] == name) return static_cast<int>(i);
    return -1;
}

// Auto-detect DJI CSV format and load a quaternion time series.
bool loadQuaternions(const std::string& path, std::vector<TimeQuat>& out) {
    std::ifstream in(path);
    if (!in) { std::cerr << "error: cannot open " << path << "\n"; return false; }

    std::string header_line;
    if (!std::getline(in, header_line)) { std::cerr << "error: empty file\n"; return false; }
    const std::vector<std::string> h = splitCsv(header_line);

    // Timestamp column: prefer quat_timestamp_ms (full), else timestamp_ms (camera_data).
    int ts = indexOf(h, "quat_timestamp_ms");
    if (ts < 0) ts = indexOf(h, "timestamp_ms");
    // Quaternion columns: quat_* (full) or org_quat_* (camera_data).
    int qw = indexOf(h, "quat_w"), qx = indexOf(h, "quat_x"), qy = indexOf(h, "quat_y"), qz = indexOf(h, "quat_z");
    if (qw < 0) { qw = indexOf(h, "org_quat_w"); qx = indexOf(h, "org_quat_x"); qy = indexOf(h, "org_quat_y"); qz = indexOf(h, "org_quat_z"); }

    if (ts < 0 || qw < 0 || qx < 0 || qy < 0 || qz < 0) {
        std::cerr << "error: unrecognised CSV header (need timestamp + quaternion columns)\n";
        return false;
    }

    const int need = std::max({ts, qw, qx, qy, qz});
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const std::vector<std::string> f = splitCsv(line);
        if (static_cast<int>(f.size()) <= need) continue;
        TimeQuat tq;
        tq.timestamp_ms = std::atof(f[ts].c_str());
        tq.quat = Quaternion{ std::atof(f[qw].c_str()), std::atof(f[qx].c_str()),
                              std::atof(f[qy].c_str()), std::atof(f[qz].c_str()) }.normalized();
        out.push_back(tq);
    }
    // Ensure strictly increasing timestamps for interpolation/search.
    std::stable_sort(out.begin(), out.end(), [](const TimeQuat& a, const TimeQuat& b) {
        return a.timestamp_ms < b.timestamp_ms;
    });
    return !out.empty();
}

// Video-frame timestamps at `fps` spanning [a, b] ms.
std::vector<double> frameTimestamps(double a, double b, double fps) {
    std::vector<double> ts;
    if (fps <= 0.0 || b <= a) return ts;
    const double dt = 1000.0 / fps;
    for (double t = a; t <= b + 1e-6; t += dt) ts.push_back(t);
    return ts;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 2; }
    Args a;
    a.mode = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--quat") a.quat_path = next();
        else if (k == "--fps") a.fps = std::atof(next().c_str());
        else if (k == "--search") a.search = std::atof(next().c_str());
        else if (k == "--initial") a.initial = std::atof(next().c_str());
        else if (k == "--lpf") a.lpf = std::atof(next().c_str());
        else if (k == "--swap-xy") a.swap_xy = true;
        else if (k == "--noise") a.noise = std::atof(next().c_str());
        else if (k == "--inject") a.inject = parseList(next());
        else if (k == "--range") { auto v = parseList(next()); if (v.size() == 2) { a.range_a = v[0]; a.range_b = v[1]; } }
        else { std::cerr << "unknown arg: " << k << "\n"; usage(); return 2; }
    }
    if (a.quat_path.empty()) { std::cerr << "error: --quat is required\n"; usage(); return 2; }

    std::vector<TimeQuat> quats;
    if (!loadQuaternions(a.quat_path, quats)) return 1;

    // Full-rate angular velocity from the fused quaternions (this plays the role of the IMU gyro).
    const std::vector<TimeImu> omega = quaternionsToAngularVelocity(quats, a.swap_xy, /*degrees=*/true);

    const double first_ts = omega.front().timestamp_ms;
    const double last_ts = omega.back().timestamp_ms;
    const double imu_rate = (omega.size() > 1) ? (omega.size() - 1) / ((last_ts - first_ts) / 1000.0) : 0.0;

    double ra = (a.range_b > a.range_a) ? a.range_a : first_ts;
    double rb = (a.range_b > a.range_a) ? a.range_b : last_ts;

    std::cout << std::fixed << std::setprecision(4);
    std::cerr << "Loaded " << quats.size() << " quaternions, span [" << first_ts << ", " << last_ts
              << "] ms, IMU rate ~" << std::setprecision(2) << imu_rate << " Hz\n" << std::setprecision(4);

    if (a.mode == "omega") {
        std::cout << "timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s\n";
        for (const auto& s : omega) {
            if (s.timestamp_ms < ra || s.timestamp_ms > rb) continue;
            std::cout << s.timestamp_ms << "," << s.gyro[0] << "," << s.gyro[1] << "," << s.gyro[2] << "\n";
        }
        return 0;
    }

    if (a.mode == "compare") {
        const std::vector<double> fts = frameTimestamps(ra, rb, a.fps);
        std::vector<TimeImu> video = resampleAngularVelocity(omega, fts);
        const double ma = maxAngle(video);
        std::cerr << "Video signal: " << video.size() << " frames @ " << a.fps
                  << " fps, max |omega| = " << ma << " deg/s\n";
        if (ma < 3.0) { std::cerr << "warning: motion below 3 deg/s gate; result unreliable\n"; }
        OffsetResult r = findOffset(video, omega, a.initial, a.search, a.fps, imu_rate, a.lpf);
        if (r.found) {
            std::cout << "recovered_offset_ms=" << r.offset_ms << " cost=" << r.cost
                      << " matched=" << r.matched << "/" << video.size() << "\n";
            std::cout << "(true offset is 0; recovered value is the algorithm's bias floor)\n";
        } else {
            std::cout << "no acceptable offset found\n";
            return 1;
        }
        return 0;
    }

    if (a.mode == "selftest") {
        const std::vector<double> fts = frameTimestamps(ra, rb, a.fps);
        std::mt19937 rng(12345);                 // fixed seed -> reproducible
        std::normal_distribution<double> noise(0.0, a.noise);

        std::cout << "injected_ms,recovered_ms,error_ms,cost,matched,frames\n";
        std::vector<double> errs;
        std::size_t ok = 0;
        for (double inj : a.inject) {
            // Sample the true motion as the camera would have seen it, delayed by `inj`.
            std::vector<double> shifted;
            shifted.reserve(fts.size());
            for (double t : fts) shifted.push_back(t - inj);
            std::vector<TimeImu> video = resampleAngularVelocity(omega, shifted);
            for (std::size_t k = 0; k < video.size(); ++k) {
                video[k].timestamp_ms = fts[k]; // video timestamps stay on the video clock
                if (a.noise > 0.0) {
                    video[k].gyro[0] += noise(rng);
                    video[k].gyro[1] += noise(rng);
                    video[k].gyro[2] += noise(rng);
                }
            }
            OffsetResult r = findOffset(video, omega, a.initial, a.search, a.fps, imu_rate, a.lpf);
            if (r.found) {
                const double err = r.offset_ms - inj;
                errs.push_back(err);
                ++ok;
                std::cout << inj << "," << r.offset_ms << "," << err << "," << r.cost
                          << "," << r.matched << "," << video.size() << "\n";
            } else {
                std::cout << inj << ",NA,NA,NA,0," << video.size() << "\n";
            }
        }

        if (!errs.empty()) {
            double sum = 0.0, sq = 0.0, maxabs = 0.0;
            for (double e : errs) { sum += e; sq += e * e; maxabs = std::max(maxabs, std::abs(e)); }
            const double mean = sum / errs.size();
            const double rms = std::sqrt(sq / errs.size());
            std::cerr << "\n=== Timestamp-sync precision (" << ok << "/" << a.inject.size()
                      << " recovered) ===\n"
                      << "mean error  = " << mean << " ms\n"
                      << "RMS error   = " << rms << " ms\n"
                      << "max |error| = " << maxabs << " ms\n";
        } else {
            std::cerr << "no offsets recovered\n";
            return 1;
        }
        return 0;
    }

    std::cerr << "unknown mode: " << a.mode << "\n";
    usage();
    return 2;
}
