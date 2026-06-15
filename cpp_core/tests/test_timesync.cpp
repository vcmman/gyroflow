#include "gyroflow/timesync.hpp"
#include "gyroflow/types.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace gyroflow;

namespace {

constexpr double kPi = 3.14159265358979323846;

int g_failures = 0;
// Works regardless of NDEBUG (Release strips assert()).
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); ++g_failures; } } while (0)

bool close(double a, double b, double eps) { return std::abs(a - b) <= eps; }

// 1) Lowpass: DC passes through (unit gain); a tone above cutoff is attenuated; Nyquist guard.
void testLowpass() {
    const double fs = 1000.0;
    {
        std::vector<TimeImu> data;
        for (int i = 0; i < 500; ++i) data.push_back({i * 1.0, true, {1.0, -2.0, 0.5}});
        CHECK(Lowpass::filterGyroForwardBackward(20.0, fs, data), "lpf valid");
        CHECK(close(data[250].gyro[0], 1.0, 1e-6), "DC x");
        CHECK(close(data[250].gyro[1], -2.0, 1e-6), "DC y");
        CHECK(close(data[250].gyro[2], 0.5, 1e-6), "DC z");
    }
    {
        std::vector<TimeImu> data;
        for (int i = 0; i < 1000; ++i) {
            const double v = std::sin(2.0 * kPi * 150.0 * (i / fs));
            data.push_back({i * 1.0, true, {v, 0.0, 0.0}});
        }
        std::vector<TimeImu> filtered = data;
        CHECK(Lowpass::filterGyroForwardBackward(20.0, fs, filtered), "lpf valid 2");
        double amp_in = 0.0, amp_out = 0.0;
        for (std::size_t i = 400; i < 600; ++i) {
            amp_in = std::max(amp_in, std::abs(data[i].gyro[0]));
            amp_out = std::max(amp_out, std::abs(filtered[i].gyro[0]));
        }
        CHECK(amp_out < amp_in * 0.05, "150Hz attenuated >95%");
    }
    {
        std::vector<TimeImu> data = {{0.0, true, {7.0, 7.0, 7.0}}, {1.0, true, {7.0, 7.0, 7.0}}};
        CHECK(!Lowpass::filterGyroForwardBackward(20.0, 30.0, data), "Nyquist guard rejects");
        CHECK(close(data[0].gyro[0], 7.0, 1e-12), "Nyquist guard leaves data");
    }
    std::printf("testLowpass done\n");
}

// 2) quaternionsToAngularVelocity: constant-rate yaw -> constant known omega.
void testOmega() {
    const double rate_deg_s = 45.0;
    const double rate_rad_s = rate_deg_s * kPi / 180.0;
    std::vector<TimeQuat> quats;
    for (int i = 0; i <= 1000; ++i) {
        const double angle = rate_rad_s * (i / 1000.0);
        quats.push_back({i * 1.0, Quaternion::fromAxisAngle({0.0, 0.0, 1.0}, angle)});
    }
    const auto omega = quaternionsToAngularVelocity(quats, /*swap_xy=*/false, /*degrees=*/true);
    CHECK(omega.size() == quats.size(), "omega size");
    CHECK(close(omega[500].gyro[0], 0.0, 1e-6), "omega x ~0");
    CHECK(close(omega[500].gyro[1], 0.0, 1e-6), "omega y ~0");
    CHECK(close(omega[500].gyro[2], rate_deg_s, 1e-3), "omega z == rate");

    // swap_xy moves z-rate? No: swap only x<->y. Pitch-rate test.
    std::vector<TimeQuat> pq;
    for (int i = 0; i <= 1000; ++i)
        pq.push_back({i * 1.0, Quaternion::fromAxisAngle({0.0, 1.0, 0.0}, rate_rad_s * (i / 1000.0))});
    const auto po = quaternionsToAngularVelocity(pq, /*swap_xy=*/true, true);
    CHECK(close(po[500].gyro[0], rate_deg_s, 1e-3), "swap_xy: y-rate -> x");
    std::printf("testOmega done (wz=%.5f deg/s)\n", omega[500].gyro[2]);
}

// Build a realistic band-limited random-walk attitude (AR(1) body rates -> integrated quats).
std::vector<TimeQuat> makeRandomWalkAttitude(int n, double fs, unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> nd(0.0, 1.0);
    double ax = 0, ay = 0, az = 0;
    std::vector<TimeQuat> quats;
    Quaternion cur = Quaternion::identity();
    for (int i = 0; i < n; ++i) {
        ax = 0.99 * ax + 0.06 * nd(rng);
        ay = 0.99 * ay + 0.06 * nd(rng);
        az = 0.99 * az + 0.06 * nd(rng);
        const double mag = std::sqrt(ax * ax + ay * ay + az * az);
        cur = (cur * Quaternion::fromAxisAngle({ax, ay, az}, mag * (1.0 / fs))).normalized();
        quats.push_back({i * 1.0, cur});
    }
    return quats;
}

// Sample `omega` at video fps shifted by `inject` ms, with optional Gaussian noise (deg/s).
std::vector<TimeImu> makeVideoSignal(const std::vector<TimeImu>& omega, double span_ms, double fps,
                                     double inject, double noise_sigma, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, noise_sigma);
    std::vector<double> shifted, vts;
    for (double t = 0.0; t <= span_ms; t += 1000.0 / fps) { vts.push_back(t); shifted.push_back(t - inject); }
    auto video = resampleAngularVelocity(omega, shifted);
    for (std::size_t k = 0; k < video.size(); ++k) {
        video[k].timestamp_ms = vts[k];
        if (noise_sigma > 0.0) for (int c = 0; c < 3; ++c) video[k].gyro[c] += noise(rng);
    }
    return video;
}

// 3) findOffset: realistic random-walk motion, inject known offsets, recover them.
void testRecovery() {
    const double fs = 1000.0;
    const int N = 6000;
    const auto quats = makeRandomWalkAttitude(N, fs, 777);
    const auto omega = quaternionsToAngularVelocity(quats, false, true);
    const double imu_rate = (omega.size() - 1) / ((omega.back().timestamp_ms - omega.front().timestamp_ms) / 1000.0);

    std::mt19937 rng(0); // unused (no noise)
    const double fps = 30.0, span = (N - 1) * 1.0;
    double max_err = 0.0;
    for (double inject : {-20.0, -7.3, 0.0, 8.5, 18.0}) {
        auto video = makeVideoSignal(omega, span, fps, inject, 0.0, rng);
        OffsetResult r = findOffset(video, omega, 0.0, 80.0, fps, imu_rate, 20.0);
        CHECK(r.found, "offset found");
        const double err = r.offset_ms - inject;
        max_err = std::max(max_err, std::abs(err));
        std::printf("  inject=%6.2f recovered=%8.3f err=%+.3f ms\n", inject, r.offset_ms, err);
    }
    CHECK(max_err < 0.6, "recovery within 0.6 ms");
    std::printf("testRecovery done (max err %.3f ms)\n", max_err);
}

// 4) Noise robustness: adding Gaussian sensor noise must barely move the recovered offset
//    (the 20 Hz forward-backward low-pass + least-squares averaging suppress it).
void testNoiseRobustness() {
    const double fs = 1000.0;
    const int N = 6000;
    const auto quats = makeRandomWalkAttitude(N, fs, 777); // well-conditioned fixture (sharp minimum)
    const auto omega = quaternionsToAngularVelocity(quats, false, true);
    const double imu_rate = (omega.size() - 1) / ((omega.back().timestamp_ms - omega.front().timestamp_ms) / 1000.0);

    const double fps = 30.0, span = (N - 1) * 1.0, inject = 8.5;

    std::mt19937 rng0(99);
    const double clean = findOffset(makeVideoSignal(omega, span, fps, inject, 0.0, rng0),
                                    omega, 0.0, 80.0, fps, imu_rate, 20.0).offset_ms;
    std::printf("  noise=0.0 recovered=%.3f (err=%.3f ms)\n", clean, clean - inject);
    CHECK(std::abs(clean - inject) < 0.6, "clean recovery within 0.6 ms");

    for (double sigma : {1.0, 3.0, 5.0}) {
        std::mt19937 rng(99);
        OffsetResult r = findOffset(makeVideoSignal(omega, span, fps, inject, sigma, rng),
                                    omega, 0.0, 80.0, fps, imu_rate, 20.0);
        CHECK(r.found, "offset found (noisy)");
        std::printf("  noise=%.1f deg/s recovered=%.3f (drift from clean=%.3f ms)\n",
                    sigma, r.offset_ms, r.offset_ms - clean);
        CHECK(std::abs(r.offset_ms - clean) < 0.4, "noise drifts recovery < 0.4 ms");
    }
    std::printf("testNoiseRobustness done\n");
}

} // namespace

int main() {
    testLowpass();
    testOmega();
    testRecovery();
    testNoiseRobustness();
    if (g_failures == 0) { std::printf("all timesync tests passed\n"); return 0; }
    std::printf("%d timesync check(s) FAILED\n", g_failures);
    return 1;
}
