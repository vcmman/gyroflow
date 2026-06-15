#pragma once

// UI-free C++ port of Gyroflow's gyro<->video time-offset finder ("autosync time").
//
// References (Rust):
//   src/core/synchronization/find_offset/essential_matrix.rs  (find_offsets / calculate_cost)
//   src/core/filtering.rs                                      (Lowpass, forward-backward)
//   src/core/synchronization/mod.rs                            (estimated_gyro units/axes)
//
// This header exposes the numerically interesting core: given two angular-velocity time
// series (one from the video, one from the IMU), find the constant time offset that best
// aligns them. The video-side signal is synthesised from DJI fused quaternions rather than
// decoded from pixels, which also enables ground-truthed accuracy evaluation (inject a known
// offset, measure how precisely it is recovered).

#include "gyroflow/types.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace gyroflow {

// One IMU/gyro sample. `gyro` is angular velocity; units (deg/s vs rad/s) and axis order are
// whatever the producer chose, but must be consistent between the two compared signals.
struct TimeImu {
    double timestamp_ms = 0.0;
    bool has_gyro = false;
    std::array<double, 3> gyro{0.0, 0.0, 0.0};
};

// RBJ Butterworth biquad low-pass (Q = 1/sqrt(2)) in Direct-Form-II-Transposed, matching the
// `biquad` crate used by Gyroflow. Six independent channels (gyro xyz + accl xyz); we only use
// the first three.
class Lowpass {
public:
    Lowpass(double freq_hz, double sample_rate_hz);

    double run(std::size_t channel, double input);

    // Zero-phase filter (forward then backward pass) over the gyro of each sample, mirroring
    // crate::filtering::Lowpass::filter_gyro_forward_backward. Returns false if the parameters
    // are invalid (non-positive / cutoff >= Nyquist).
    static bool filterGyroForwardBackward(double freq_hz, double sample_rate_hz, std::vector<TimeImu>& data);

private:
    struct Biquad {
        double a1 = 0.0, a2 = 0.0, b0 = 1.0, b1 = 0.0, b2 = 0.0;
        double s1 = 0.0, s2 = 0.0;
        double run(double input);
    };
    std::array<Biquad, 6> filters_{};
    bool valid_ = false;
};

// Generate an angular-velocity series from a quaternion attitude series.
// For consecutive samples (t0,q0) and (t1,q1): dq = q0^-1 * q1, take its axis*angle rotation
// vector (body frame), divide by dt -> angular velocity. This is the same per-frame derivation
// Gyroflow uses to turn pose rotations into `estimated_gyro`.
//   swap_xy : swap the x and y components (Gyroflow stores estimated_gyro with x/y swapped).
//   degrees : convert rad/s -> deg/s (Gyroflow's estimated_gyro and raw IMU are in deg/s).
// Output length == samples.size(); the last sample copies the previous angular velocity.
std::vector<TimeImu> quaternionsToAngularVelocity(const std::vector<TimeQuat>& samples,
                                                  bool swap_xy = false,
                                                  bool degrees = true);

// Resample an angular-velocity series at the given timestamps (linear interpolation, clamped
// at the ends). Used to build a video-frame-rate signal from a full-rate one.
std::vector<TimeImu> resampleAngularVelocity(const std::vector<TimeImu>& series,
                                             const std::vector<double>& target_timestamps_ms);

struct OffsetResult {
    bool found = false;
    double offset_ms = 0.0;   // timestamp delay: gyro is sampled at (of_ts - offset_ms)
    double cost = 0.0;        // mean weighted squared angular-velocity difference at the optimum
    std::size_t matched = 0;  // number of OF samples that found a gyro match
};

// Port of essential_matrix::find_offsets for a single sync range / signal pair.
//   of    : the video-side angular-velocity signal (sparse, ~video fps), e.g. `estimated_gyro`.
//   gyro  : the IMU-side angular-velocity signal (dense, ~IMU rate), e.g. raw_imu.
// Both are 20 Hz zero-phase low-passed (lpf_hz) before matching. The search sweeps
// [initial_offset - search_size, initial_offset + search_size] at 1 ms, then refines +/-2 ms at
// 0.01 ms. Cost weights x,y by 70 and z by 100, averaged over matched samples; a candidate is
// rejected unless > half the OF samples matched. The max-angle<3 gate is applied by the caller.
OffsetResult findOffset(std::vector<TimeImu> of,
                        std::vector<TimeImu> gyro,
                        double initial_offset_ms,
                        double search_size_ms,
                        double of_sample_rate_hz,
                        double gyro_sample_rate_hz,
                        double lpf_hz = 20.0);

// Largest absolute gyro component across the series (Gyroflow's get_max_angle); used to skip
// sync points with no real movement (threshold 3 in estimated_gyro deg/s units).
double maxAngle(const std::vector<TimeImu>& data);

} // namespace gyroflow
