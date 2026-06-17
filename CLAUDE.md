# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Gyroflow is a cross-platform video stabilization app that uses gyroscope/accelerometer
telemetry embedded in video files (GoPro, Sony, DJI, Insta360, etc.) to stabilize footage.
The app is **Rust + QML**; the stabilization engine is a dependency-free Rust library.

This checkout (`claude/new-cpp-impl` branch) additionally contains in-progress work that is
**not** part of upstream Gyroflow:
- `cpp_core/` — a UI-free **C++ port** of the core algorithms (separate CMake project), built
  and validated against the Rust engine's golden output.
- `tools/` + `data/` — Python tools for analyzing **DJI MP4 quaternion/telemetry** streams,
  used to drive and verify the C++ port.

When asked to work on "the core port", "DJI quaternions", or "the C++ stuff", that means
`cpp_core/` and `tools/` — not the main Rust app.

## Build & run

Builds go through `just` (a `Justfile` wrapper that dispatches to per-OS scripts in
`_scripts/<os>.just`). Run all commands from the repo root. On Linux the relevant recipes are:

```sh
just install-deps   # one-time: fetches Qt, ffmpeg, OpenCV (vcpkg) into ext/, installs apt deps
just run [args]     # cargo run --release -- <args>
just debug [args]   # cargo run -- <args>   (debug build)
just test [args]    # cargo test -- <args>
just clippy         # lint
just deploy         # release build + bundle Qt/ffmpeg/mdk libs into a distributable
```

`just <cmd>` always forwards to `_scripts/{linux,macos,windows,android,ios}.just`. The
heavy native deps (Qt, ffmpeg, OpenCV, mdk-sdk) are external and pulled by `install-deps`;
a bare `cargo build` will not work without them.

Direct cargo also works once deps exist:
```sh
cargo build --release                 # build the gyroflow binary
cargo test -p gyroflow-core           # test only the core engine
cargo test --test <name> -- <filter>  # run a single test / filter by name
```

The default feature set enables `opencv` (used only for lens calibration and optical-flow
sync). `opencl` is an extra feature. Mobile targets build core with `opencv` only.

## Workspace layout

Two crates:
- **`gyroflow`** (root `Cargo.toml`, entry `src/gyroflow.rs`) — the desktop/mobile app:
  QML UI, ffmpeg rendering, GPU preview, CLI. Edition 2024.
- **`gyroflow-core`** (`src/core/`, crate name `gyroflow-core`) — the stabilization engine.
  No Qt, no ffmpeg; OpenCV optional. This is the part being ported to C++.

### App (`src/`)
- `src/gyroflow.rs` — main entry point; `src/cli.rs` — headless CLI path.
- `src/controller.rs` — the bridge between QML and core; QML calls land here and dispatch
  into `gyroflow-core`. This is the main place to look when wiring UI ↔ engine.
- `src/ui/` — all QML (entry `App.qml` / `main_window.qml`). `src/ui/*.rs` expose Rust to QML.
- `src/rendering/` — all ffmpeg code (final render + processing for sync).
- `src/qt_gpu/` — zero-copy GPU undistortion via Qt RHI + GLSL compute shaders.

### Engine (`src/core/`, see `lib.rs` for the module list)
The processing pipeline, roughly in order:
1. `gyro_source/` — parse telemetry (via the `telemetry-parser` crate) into a quaternion
   time series; per-vendor quirks in `sony.rs`, `canon.rs`, `imu_transforms.rs`.
2. `imu_integration/` — integrate raw IMU into orientation when no fused attitude exists
   (`complementary`, `complementary_v2`, `vqf`).
3. `smoothing/` — orientation smoothing algorithms (`default_algo`, `plain`, `fixed`,
   `horizon`, …) selected at runtime.
4. `stabilization/` — undistortion + per-row rolling-shutter compensation; has CPU and
   `gpu/` (OpenCL / wgpu) backends.
5. `zooming/` — adaptive zoom / dynamic cropping. `lens_profile*` — lens distortion models.
   `synchronization/` — gyro-to-video auto-sync (uses OpenCV optical flow when available).

`mod.rs`/`lib.rs` in each directory is that module's entry point.

## C++ core port (`cpp_core/`)

Self-contained CMake project (C++17), independent of the Rust build:
```sh
cmake -S cpp_core -B cpp_core/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp_core/build -j
(cd cpp_core/build && ctest --output-on-failure)   # ctest 3.16 has no --test-dir; run from build/
```
The core library and unit tests have no external deps (JSON vendored in `third_party/`); the
`gyroflow_cpp_stabilize` CLI is built only when OpenCV is found.

Implemented (Phase 1 headless DJI stabilizer + Phase 2 adaptive zoom + Gyroflow
`output_dimension` framing): `distortion/` (OpenCV fisheye), `smoothing/default_algo`,
`stabilization/{frame_transform,undistort}`, `mat3.hpp`, `zooming/adaptive_zoom`,
`telemetry_io` (JSON bridge), and `tools/gyroflow_cpp_stabilize.cpp`. Each `*.cpp` is a
faithful port of the matching `src/core/` Rust file — check changes against Gyroflow golden
output. The math is validated against golden Gyroflow metadata (smoothing ≤0.0147°, adaptive
fov ≤0.0012% over 973 frames) via `tools/gyroflow_cpp_validate` (OpenCV-free) +
`tools/compare_gyroflow_metadata.py`; stabilization quality (shake removed, before/after) via
`tools/stabilization_quality.py` (ITF + residual inter-frame motion). Scope/migration order
and validation numbers live in `cpp_core/README.md` and `cpp_core/DEVELOPMENT_PLAN.md`; the
end-to-end algorithm flow + exact parameters behind the result video are in
`cpp_core/PIPELINE.md`; the quantitative head-to-head vs Gyroflow under identical parameters
(math, frame PSNR, stabilization quality) is in `cpp_core/COMPARISON.md`. Coordinate
conventions (and why a gyro in the OpenCV image frame needs Gyroflow `imu_orientation = yxz`,
Gyroflow's expected IMU input frame being X=up/Y=left/Z=back) are in `cpp_core/PIPELINE.md`
("Coordinate conventions"), verified by `cpp_core/tests/test_imu_orientation.cpp`.

Run the stabilizer (DJI fused-quaternion path, OpenCV-fisheye lens, adaptive zoom on):
```sh
python3 tools/export_bridge_json.py INPUT.MP4 -o bridge.json   # needs a Gyroflow binary
./cpp_core/build/gyroflow_cpp_stabilize INPUT.MP4 --telemetry bridge.json -o out.mp4
```
Output is encoded via `ffmpeg` (H.264/H.265 + audio copy) when on PATH, else OpenCV `mp4v`.
The telemetry parser and video *decode* are still bridged (JSON sidecar + OpenCV decode);
native DJI protobuf + libav decode are later phases.

## DJI telemetry tools (`tools/`)

Python 3.10+ scripts (need `numpy`, `matplotlib`) that extract and plot DJI MP4 quaternion
streams. Several scripts shell out to a built Gyroflow binary to export the same camera-data
CSV Gyroflow uses internally; they locate it via `--gyroflow-bin`, `$GYROFLOW_BIN`, or
`gyroflow`/`Gyroflow` on `PATH`. On headless machines set `MPLBACKEND=Agg`. See
`tools/README_DJI_quaternion.md` for the full command reference; shared helpers live in
`tools/gyro_analysis/`.

## Conventions

- The core library must stay free of Qt/ffmpeg; keep OpenCV usage behind the `opencv`
  feature and confined to calibration + optical-flow sync.
- All timing is timestamp-based (variable/high frame rate support), not frame-index based.
- UI live reload: set `live_reload = true` in `gyroflow.rs` to hot-reload QML on save.
