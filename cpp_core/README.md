# Gyroflow C++ Core Port

This directory is the UI-free C++ porting surface for Gyroflow core algorithms.

Initial scope:

- Quaternion time series utilities.
- Quaternion-only stabilization path, matching the DJI-style fused attitude input first.
- Rolling shutter row timestamp and row compensation calculation.
- Lens profile data structures with `opencv_fisheye` as the default distortion model.

Near-term migration order:

1. DJI MP4 metadata reader: `djmd`/DVTM protobuf parsing into quaternion, lens profile, and readout metadata.
2. Lens distortion models: start with OpenCV fisheye, then add OpenCV standard and camera-specific models.
3. Default smoothing parity against Rust Gyroflow golden data.
4. CPU remap renderer for debug output.
5. Raw IMU integration paths: Complementary, VQF, SimpleGyro, Mahony, and Madgwick.

This is intentionally separate from QML/UI code. The target shape is a small core library plus CLI tools that can be validated against Rust Gyroflow outputs.

## Autosync-time (gyro↔video time-offset finder)

`timesync.{hpp,cpp}` ports Gyroflow's default time-offset search
(`essential_matrix::find_offsets` + `filtering::Lowpass`, forward-backward 20 Hz Butterworth).
Given two angular-velocity series it finds the constant timestamp delay (ms) that aligns them
(1 ms coarse sweep → 0.01 ms refine, weighted least-squares cost). See
[`AUTOSYNC_TIME_PLAN.md`](AUTOSYNC_TIME_PLAN.md) for scope and the evaluation methodology.

Because we don't decode video, the video-side motion signal is synthesised from the DJI fused
quaternions (`quaternionsToAngularVelocity`, validated to match `tools/`'s `quat_omega_*` output).

CLI `gyroflow_autosync`:
```sh
# Inject known offsets and measure recovery accuracy (timestamp-sync precision):
./build/gyroflow_autosync selftest --quat ../data/dji_quaternions_full.csv --fps 30 --search 100
# Baseline bias (true offset 0) between full-rate and video-rate signals:
./build/gyroflow_autosync compare  --quat ../data/dji_quaternions_full.csv --fps 30
# Dump quaternion-derived angular velocity as CSV:
./build/gyroflow_autosync omega    --quat ../data/dji_quaternions_full.csv > omega.csv
```
On the bundled DJI clip, offset recovery is **~0.4 ms RMS (max 0.7 ms)** at 30 fps — the residual
is the ~0.3 ms bias of the nearest-IMU-sample (1 kHz) lookup, faithful to the Rust algorithm.

NOTE: `ctest --test-dir build` needs CMake ≥ 3.20; on older CMake run `ctest` from inside `build/`.
