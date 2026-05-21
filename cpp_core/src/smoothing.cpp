#include "gyroflow/types.hpp"

#include <cmath>

namespace gyroflow {

std::vector<TimeQuat> smoothQuaternionsPlain(const std::vector<TimeQuat>& samples, double smoothness_ms) {
    if (samples.empty()) {
        return {};
    }
    if (samples.size() == 1 || smoothness_ms <= 0.0) {
        return samples;
    }

    std::vector<TimeQuat> out;
    out.reserve(samples.size());
    out.push_back(samples.front());

    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].timestamp_ms - samples[i - 1].timestamp_ms;
        const double alpha = 1.0 - std::exp(-std::max(0.0, dt) / smoothness_ms);
        out.push_back({
            samples[i].timestamp_ms,
            slerp(out.back().quat, samples[i].quat, alpha),
        });
    }

    for (std::size_t i = out.size() - 1; i > 0; --i) {
        const double dt = out[i].timestamp_ms - out[i - 1].timestamp_ms;
        const double alpha = 1.0 - std::exp(-std::max(0.0, dt) / smoothness_ms);
        out[i - 1].quat = slerp(out[i].quat, out[i - 1].quat, alpha);
    }

    return out;
}

} // namespace gyroflow
