#pragma once

#include <string>

#include "gyroflow/telemetry.hpp"

namespace gyroflow {

// Phase 1 telemetry bridge.
//
// Instead of parsing DJI djmd/DVTM protobuf in C++ (deferred to a later phase), Phase 1
// consumes a JSON sidecar exported by the Rust Gyroflow CLI / the Python tools in tools/.
// This decouples the stabilization-algorithm port from the metadata-parser port.
//
// Expected JSON schema (all fields optional unless noted; unknown fields ignored):
//
//   {
//     "detected_source": "DJI Avata 2",
//     "fps": 59.94,                       // required for timestamp -> frame mapping
//     "width": 3840,                      // required
//     "height": 2160,                     // required
//     "frame_readout_time_ms": 25.0,      // 0 disables rolling-shutter correction
//     "frame_readout_direction": "TopToBottom",  // or BottomToTop/LeftToRight/RightToLeft
//
//     // Fused (org) attitude, one entry per sample. Timestamp is microseconds.
//     // Quaternion is scalar-first (w, x, y, z), matching gyroflow-core.
//     "quaternions": [ [t_us, w, x, y, z], ... ],   // required, non-empty
//
//     "lens_profile": {
//       "camera_brand": "DJI",
//       "camera_model": "Avata 2",
//       "lens_model": "",
//       "calib_width": 3840,  "calib_height": 2160,
//       "output_width": 3840, "output_height": 2160,
//       // Intrinsics: either a 3x3 "camera_matrix" (Gyroflow fisheye_params layout)
//       // or flat fx/fy/cx/cy.
//       "camera_matrix": [[fx,0,cx],[0,fy,cy],[0,0,1]],
//       "distortion_model": "opencv_fisheye",
//       "distortion_coeffs": [k0, k1, k2, k3],
//       "frame_readout_time_ms": 25.0
//     }
//   }
//
// Throws std::runtime_error with a descriptive message on parse error or missing
// required fields.
FileMetadata loadTelemetryFromJsonString(const std::string& json);
FileMetadata loadTelemetryFromJsonFile(const std::string& path);

} // namespace gyroflow
