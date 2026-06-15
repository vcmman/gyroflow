# Gyroflow C++ Core Port

UI-free C++ port of Gyroflow's stabilization core. The target shape is a small
dependency-light library plus CLI tools, validated against Rust Gyroflow output. See
[`DEVELOPMENT_PLAN.md`](DEVELOPMENT_PLAN.md) for the full phased plan and the porting
references into `../src/core/`.

## Status

Phase 1 (headless DJI stabilizer) and Phase 2 (adaptive zoom) are implemented and run
end-to-end on a real DJI clip, producing a stabilized MP4 with dynamic cropping.

Implemented:

- **Quaternion math** (`types.hpp`, `quaternion.cpp`) — slerp, inverse, time-series sampling.
- **OpenCV fisheye model** (`distortion/opencv_fisheye.*`) — forward `distort_point` and the
  10-iteration Newton `undistort_point`, ported from `opencv_fisheye.rs`.
- **Default smoothing** (`smoothing/default_algo.*`) — velocity-adaptive forward/backward
  slerp with the distance-modulated second pass (`default_algo.rs`, non-per-axis path).
- **Frame transform** (`stabilization/frame_transform.*`, `mat3.hpp`) — per-scanline
  `i_r = inverse(new_K · R)` with rolling-shutter row timing.
- **CPU undistort kernel** (`stabilization/undistort.*`) — `rotate_and_distort` + bilinear
  remap over packed 8-bit buffers, threaded over rows. Core stays OpenCV-free.
- **Adaptive zoom** (`zooming/adaptive_zoom.*`) — forward `undistortPoints`, per-frame
  inscribed-rectangle FOV search, EnvelopeFollower temporal smoothing (DJI default), and
  `max_zoom` clamp.
- **Telemetry bridge** (`telemetry_io.*`, `../tools/export_bridge_json.py`) — Phase 1 reads
  a JSON sidecar exported from the Rust Gyroflow CLI instead of parsing DJI protobuf in C++.

Not yet done: native DJI metadata parser, native FFmpeg I/O (Phase 1 uses OpenCV video I/O,
no audio), cropped `output_dimension` (renders the full sensor; see the note in the plan),
horizon lock, additional lens models, GPU.

## Build & test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
(cd build && ctest --output-on-failure)   # ctest 3.16 has no --test-dir; run from build/
```

The `gyroflow_cpp_stabilize` CLI is built when OpenCV is found; the core library and unit
tests have no external dependencies (JSON is vendored under `third_party/`).

## Run the headless stabilizer

```sh
# 1. Export the telemetry bridge JSON (needs a Gyroflow binary on PATH, or pass a
#    pre-exported .gyroflow project + camera-data CSV):
python3 ../tools/export_bridge_json.py INPUT.MP4 -o bridge.json

# 2. Stabilize (adaptive zoom on by default):
./build/gyroflow_cpp_stabilize INPUT.MP4 --telemetry bridge.json -o out.mp4
#   --max-zoom 130        zoom ceiling (percent)
#   --no-adaptive-zoom    disable dynamic crop
#   --fov 0.8             static zoom instead of adaptive
#   --max-frames N        process only the first N frames
```

## Implementation notes

- **Bridge-first.** The hard DJI `djmd`/DVTM protobuf parse is deferred; the algorithm port
  is validated against a JSON sidecar (org quaternions + readout time + matched lens
  profile) exported from Rust Gyroflow. This decoupled the math port from the parser port.
- **Faithful to the Rust math.** Quaternions are scalar-first unit quaternions; the per-row
  transform is `smoothed(ts)^-1 · raw(row_ts)` folded into `inverse(new_K·R)` with the
  `frame_transform.rs` axis sign flips; the forward FOV map is its inverse with a `W>0`
  behind-camera guard. Parameters match the Gyroflow project export (smoothing 0.5 /
  max_smoothness 1.0 / alpha_0_1s 0.1, adaptive window 4 s, max_zoom 130%, method 1).
- **Validation on `data/DJI_..._0032_D.MP4`** (3840×2880, OsmoAction4, 21.82 ms readout):
  center-crop inter-frame motion reduced ~21% (subject is walking); adaptive zoom cut
  worst-frame black border from 7.4% (Phase 1 static fov) to **0.07%**.
- **Dimension-consistency fix.** The bridge lens profile carries Gyroflow's 16:9
  `output_dimension`; using it for `new_K`'s principal point while rendering the 4:3 sensor
  produced inconsistent coordinate systems and a large border. Phase 2 forces output ==
  input (full sensor); honoring the 16:9 crop is a tracked follow-up.
