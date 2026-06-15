#include "gyroflow/timesync.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace gyroflow {

namespace {

constexpr double kPi = 3.14159265358979323846;
// biquad::Q_BUTTERWORTH_F64
constexpr double kButterworthQ = 0.7071067811865476;

} // namespace

// ----------------------------------------------------------------------------
// Lowpass — RBJ Butterworth biquad, Direct-Form-II-Transposed (matches `biquad` crate).
// ----------------------------------------------------------------------------

double Lowpass::Biquad::run(double input) {
    const double out = b0 * input + s1;
    s1 = b1 * input - a1 * out + s2;
    s2 = b2 * input - a2 * out;
    return out;
}

Lowpass::Lowpass(double freq_hz, double sample_rate_hz) {
    // biquad::Coefficients::from_params returns Err(OutsideNyquist) when 2*f0 > fs; Gyroflow
    // ignores that error and leaves the signal unfiltered. Reproduce that exactly.
    if (!(sample_rate_hz > 0.0) || !(freq_hz > 0.0) || 2.0 * freq_hz > sample_rate_hz) {
        return; // leave filters as pass-through identity, valid_ = false
    }
    // Coefficients::from_params(Type::LowPass, fs, f0, Q_BUTTERWORTH)
    const double omega = 2.0 * kPi * freq_hz / sample_rate_hz;
    const double omega_s = std::sin(omega);
    const double omega_c = std::cos(omega);
    const double alpha = omega_s / (2.0 * kButterworthQ);

    const double b0 = (1.0 - omega_c) * 0.5;
    const double b1 = 1.0 - omega_c;
    const double b2 = (1.0 - omega_c) * 0.5;
    const double a0 = 1.0 + alpha;
    const double a1 = -2.0 * omega_c;
    const double a2 = 1.0 - alpha;

    for (auto& f : filters_) {
        f.a1 = a1 / a0;
        f.a2 = a2 / a0;
        f.b0 = b0 / a0;
        f.b1 = b1 / a0;
        f.b2 = b2 / a0;
        f.s1 = 0.0;
        f.s2 = 0.0;
    }
    valid_ = true;
}

double Lowpass::run(std::size_t channel, double input) {
    if (channel >= filters_.size()) return input;
    return filters_[channel].run(input);
}

bool Lowpass::filterGyroForwardBackward(double freq_hz, double sample_rate_hz, std::vector<TimeImu>& data) {
    Lowpass forward(freq_hz, sample_rate_hz);
    Lowpass backward(freq_hz, sample_rate_hz);
    if (!forward.valid_ || !backward.valid_) return false;

    for (auto& x : data) {
        if (x.has_gyro) {
            x.gyro[0] = forward.run(0, x.gyro[0]);
            x.gyro[1] = forward.run(1, x.gyro[1]);
            x.gyro[2] = forward.run(2, x.gyro[2]);
        }
    }
    for (auto it = data.rbegin(); it != data.rend(); ++it) {
        if (it->has_gyro) {
            it->gyro[0] = backward.run(0, it->gyro[0]);
            it->gyro[1] = backward.run(1, it->gyro[1]);
            it->gyro[2] = backward.run(2, it->gyro[2]);
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
// Quaternion attitude series -> angular velocity.
// ----------------------------------------------------------------------------

std::vector<TimeImu> quaternionsToAngularVelocity(const std::vector<TimeQuat>& samples,
                                                  bool swap_xy,
                                                  bool degrees) {
    std::vector<TimeImu> out;
    out.reserve(samples.size());
    if (samples.empty()) return out;

    const double to_deg = degrees ? (180.0 / kPi) : 1.0;

    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        const double dt_s = (samples[i + 1].timestamp_ms - samples[i].timestamp_ms) / 1000.0;
        TimeImu s;
        s.timestamp_ms = samples[i].timestamp_ms;

        if (dt_s > 1e-9) {
            // Body-frame incremental rotation dq = q0^-1 * q1.
            Quaternion dq = (samples[i].quat.normalized().inverse() * samples[i + 1].quat.normalized()).normalized();
            if (dq.w < 0.0) { dq.w = -dq.w; dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; } // shortest arc
            const double v = std::sqrt(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z);
            const double angle = 2.0 * std::atan2(v, dq.w);
            double rx = 0.0, ry = 0.0, rz = 0.0;
            if (v > 1e-12) {
                const double k = angle / v; // (axis * angle) = (v/|v|) * angle
                rx = dq.x * k;
                ry = dq.y * k;
                rz = dq.z * k;
            }
            double wx = (rx / dt_s) * to_deg;
            double wy = (ry / dt_s) * to_deg;
            double wz = (rz / dt_s) * to_deg;
            if (swap_xy) std::swap(wx, wy);
            s.has_gyro = true;
            s.gyro = {wx, wy, wz};
        }
        out.push_back(s);
    }

    // Last sample copies the previous angular velocity (no forward difference available).
    TimeImu last;
    last.timestamp_ms = samples.back().timestamp_ms;
    if (!out.empty()) { last.has_gyro = out.back().has_gyro; last.gyro = out.back().gyro; }
    out.push_back(last);
    return out;
}

// ----------------------------------------------------------------------------
// Resample (linear interpolation, end-clamped).
// ----------------------------------------------------------------------------

std::vector<TimeImu> resampleAngularVelocity(const std::vector<TimeImu>& series,
                                             const std::vector<double>& target_timestamps_ms) {
    std::vector<TimeImu> out;
    out.reserve(target_timestamps_ms.size());
    if (series.empty()) return out;

    for (double ts : target_timestamps_ms) {
        TimeImu s;
        s.timestamp_ms = ts;
        s.has_gyro = true;
        if (ts <= series.front().timestamp_ms) {
            s.gyro = series.front().gyro;
        } else if (ts >= series.back().timestamp_ms) {
            s.gyro = series.back().gyro;
        } else {
            const auto upper = std::upper_bound(series.begin(), series.end(), ts,
                [](double t, const TimeImu& e) { return t < e.timestamp_ms; });
            const auto lower = upper - 1;
            const double span = upper->timestamp_ms - lower->timestamp_ms;
            const double f = span > 1e-12 ? (ts - lower->timestamp_ms) / span : 0.0;
            for (int k = 0; k < 3; ++k) {
                s.gyro[k] = lower->gyro[k] + (upper->gyro[k] - lower->gyro[k]) * f;
            }
        }
        out.push_back(s);
    }
    return out;
}

// ----------------------------------------------------------------------------
// Offset search — port of essential_matrix::find_offsets (single range).
// ----------------------------------------------------------------------------

double maxAngle(const std::vector<TimeImu>& data) {
    double m = 0.0;
    for (const auto& x : data) {
        if (!x.has_gyro) continue;
        m = std::max(m, std::abs(x.gyro[0]));
        m = std::max(m, std::abs(x.gyro[1]));
        m = std::max(m, std::abs(x.gyro[2]));
    }
    return m;
}

namespace {

// gyro_at_timestamp: first gyro sample with key >= ts (microsecond resolution), matching
// `gyro.range((ts*1000) as usize..).next()`.
const TimeImu* gyroAt(double ts_ms, const std::map<long long, TimeImu>& gyro) {
    const long long key = static_cast<long long>(ts_ms * 1000.0);
    auto it = gyro.lower_bound(key);
    if (it == gyro.end()) return nullptr;
    return &it->second;
}

// calculate_cost: mean weighted squared angular-velocity difference over matched samples.
double calculateCost(double offs, const std::vector<TimeImu>& of, const std::map<long long, TimeImu>& gyro) {
    double sum = 0.0;
    std::size_t matches = 0;
    for (const auto& o : of) {
        if (!o.has_gyro) continue;
        const TimeImu* g = gyroAt(o.timestamp_ms - offs, gyro);
        if (g && g->has_gyro) {
            ++matches;
            sum += (g->gyro[0] - o.gyro[0]) * (g->gyro[0] - o.gyro[0]) * 70.0;
            sum += (g->gyro[1] - o.gyro[1]) * (g->gyro[1] - o.gyro[1]) * 70.0;
            sum += (g->gyro[2] - o.gyro[2]) * (g->gyro[2] - o.gyro[2]) * 100.0;
        }
    }
    if (!of.empty() && matches > of.size() / 2) {
        return sum / static_cast<double>(matches);
    }
    return std::numeric_limits<double>::max();
}

} // namespace

OffsetResult findOffset(std::vector<TimeImu> of,
                        std::vector<TimeImu> gyro,
                        double initial_offset_ms,
                        double search_size_ms,
                        double of_sample_rate_hz,
                        double gyro_sample_rate_hz,
                        double lpf_hz) {
    OffsetResult result;
    if (of.empty() || gyro.empty()) return result;

    const double last_of_ts = of.back().timestamp_ms;
    const double first_of_ts = of.front().timestamp_ms;

    // Window the gyro to the relevant span (filter in shifted space, keep original timestamps).
    std::vector<TimeImu> gyro_item;
    gyro_item.reserve(gyro.size());
    for (const auto& x : gyro) {
        const double ts = x.timestamp_ms + initial_offset_ms;
        if (ts >= first_of_ts - search_size_ms && ts <= last_of_ts + search_size_ms) {
            gyro_item.push_back(x);
        }
    }
    if (gyro_item.empty()) return result;

    // 20 Hz zero-phase low-pass both signals.
    Lowpass::filterGyroForwardBackward(lpf_hz, of_sample_rate_hz, of);
    Lowpass::filterGyroForwardBackward(lpf_hz, gyro_sample_rate_hz, gyro_item);

    std::map<long long, TimeImu> gyro_bintree;
    for (const auto& x : gyro_item) {
        gyro_bintree[static_cast<long long>(x.timestamp_ms * 1000.0)] = x;
    }

    // Coarse sweep at 1 ms.
    const int steps = static_cast<int>(search_size_ms) * 2;
    double best_off = 0.0;
    double best_cost = std::numeric_limits<double>::max();
    bool have_best = false;
    for (int i = 0; i < steps; ++i) {
        const double offs = initial_offset_ms - search_size_ms + static_cast<double>(i);
        const double cost = calculateCost(offs, of, gyro_bintree);
        if (!have_best || cost < best_cost) { best_cost = cost; best_off = offs; have_best = true; }
    }
    if (!have_best) return result;

    // Refine to 0.01 ms over +/-2 ms around the coarse optimum.
    {
        const double refine_size = 2.0;
        const int refine_steps = static_cast<int>(refine_size * 100.0);
        const double step = refine_size / static_cast<double>(refine_steps);
        double r_off = best_off;
        double r_cost = std::numeric_limits<double>::max();
        bool have_r = false;
        for (int i = 0; i < refine_steps; ++i) {
            const double offs = best_off + (-refine_size + static_cast<double>(i) * step);
            const double cost = calculateCost(offs, of, gyro_bintree);
            if (!have_r || cost < r_cost) { r_cost = cost; r_off = offs; have_r = true; }
        }
        if (have_r) { best_off = r_off; best_cost = r_cost; }
    }

    // Count matches at the optimum (for reporting).
    std::size_t matched = 0;
    for (const auto& o : of) {
        if (!o.has_gyro) continue;
        const TimeImu* g = gyroAt(o.timestamp_ms - best_off, gyro_bintree);
        if (g && g->has_gyro) ++matched;
    }

    // Accept only offsets within 90% of the search range, as in the Rust version.
    if (std::abs(best_off - initial_offset_ms) < search_size_ms * 0.9 &&
        best_cost < std::numeric_limits<double>::max()) {
        result.found = true;
        result.offset_ms = best_off;
        result.cost = best_cost;
        result.matched = matched;
    }
    return result;
}

} // namespace gyroflow
