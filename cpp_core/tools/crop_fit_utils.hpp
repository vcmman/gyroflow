#pragma once

// Shared glue for the crop-budget guard (--fit-crop, SMOOTHING_RND §8r) between the validate
// and stabilize CLIs: the per-frame required-zoom callback built from the same adaptive-zoom
// machinery the render uses. Header-only; OpenCV-free.

#include <cstddef>
#include <vector>

#include "gyroflow/lens_profile.hpp"
#include "gyroflow/smoothing/crop_guard.hpp"
#include "gyroflow/stabilization/frame_transform.hpp"
#include "gyroflow/types.hpp"
#include "gyroflow/zooming/adaptive_zoom.hpp"

namespace gyroflow_tools {

// Per-frame required zoom (1/raw_fov, instantaneous inscribed-crop demand before temporal
// smoothing / max-zoom clamp) of a candidate smoothed series, under the tool's exact lens /
// framing / RS setup. `raw` and `lens` are captured by pointer and must outlive the callback.
inline gyroflow::CropDemandFn makeCropDemandFn(std::vector<double> ts_all,
                                               const std::vector<gyroflow::TimeQuat>* raw,
                                               const gyroflow::LensProfile* lens, int width,
                                               int height, double fps,
                                               gyroflow::TransformParams tp,
                                               gyroflow::AdaptiveZoomParams az) {
    tp.fov = 1.0;  // demand is measured at unit base FOV (validate convention)
    return [=](const std::vector<gyroflow::TimeQuat>& cand) {
        std::vector<double> rf;
        gyroflow::computeAdaptiveFovs(ts_all, *raw, cand, *lens, width, height, fps, tp, az, &rf);
        std::vector<double> rz(rf.size());
        for (std::size_t i = 0; i < rf.size(); ++i)
            rz[i] = rf[i] > 1e-9 ? 1.0 / rf[i] : 1e9;
        return rz;
    };
}

} // namespace gyroflow_tools
