# Autosync-Time — Summary Report

UI-free C++ port of Gyroflow's gyro↔video time-offset finder ("autosync time"), doubling as a
timestamp-synchronization accuracy evaluator. Companion docs:
[plan](AUTOSYNC_TIME_PLAN.md) · [testing method](AUTOSYNC_TIME_TESTING.md).

## 1. What was built

| File | Role |
|------|------|
| `include/gyroflow/timesync.hpp`, `src/timesync.cpp` | Core: `Lowpass`, `quaternionsToAngularVelocity`, `resampleAngularVelocity`, `findOffset`, `maxAngle` |
| `tools/gyroflow_autosync.cpp` | CLI: `selftest` / `compare` / `omega` |
| `tools/run_autosync_eval.sh` | One-shot build + test + evaluation |
| `tests/test_timesync.cpp` | Unit tests (lowpass, ω, recovery, noise) |

Ported faithfully from the Rust engine:
- `src/core/synchronization/find_offset/essential_matrix.rs` → `findOffset` (the default
  `offset_method = 0`): 1 ms coarse sweep over `[initial±search]`, then 0.01 ms refine over ±2 ms;
  cost = mean weighted squared angular-velocity difference (x,y ×70, z ×100) over matched
  samples; nearest-upper IMU lookup; 90 % acceptance window; `max_angle < 3` gate.
- `src/core/filtering.rs` → `Lowpass`: RBJ Butterworth biquad (`Q = 1/√2`),
  Direct-Form-II-Transposed, forward-backward (zero-phase), incl. the `2·f0 > fs` Nyquist skip.

The "video-side" motion is synthesised from the DJI fused quaternions instead of decoding video
(`如果需要 gyro 的旋转数据，考虑用 dji 的四元数生成一份来用`), which also enables ground-truthed accuracy
measurement. Out of scope (left in Rust): video decode, OpenCV optical flow, pose estimation,
and the `rs_sync` / `visual_features` offset methods.

## 2. Test data

`data/`, extracted from `DJI_20260605174353_0032_D.MP4`: **32 465** fused-quaternion samples
(~989 Hz) over **32.45 s**; 973 video frames ≈ 30 fps. Both `dji_quaternions_full.csv` and
`dji_camera_data.csv` are auto-detected and give identical results.

## 3. Results

### 3.1 Unit tests — PASS
`gyroflow_cpp_core_tests` and `gyroflow_cpp_timesync_tests` both pass (`ctest`: 2/2, 100 %).
The timesync suite covers four groups: lowpass (DC/attenuation/Nyquist), ω-from-quaternions,
offset recovery, and noise robustness.
- Recovery on the random-walk fixture: max error **0.34 ms** across injected offsets {−20, −7.3,
  0, 8.5, 18} ms at 30 fps.
- Noise robustness: adding 1/3/5 °/s Gaussian noise drifts the recovered offset by **0.000 ms**.

### 3.2 ω derivation cross-check — PASS
C++ ω matches the Python tools' `quat_omega_*` exactly:

| t (ms) | C++ ωx,ωy,ωz (deg/s) | analysis CSV (rad/s) → deg/s |
|--------|----------------------|------------------------------|
| first  | 14.1351, −31.1522, −12.6851 | 0.246703 → 14.1351 ; −0.543708 → −31.1522 ; −0.221396 → −12.6851 |

### 3.3 Zero-offset baseline (`compare`, true offset = 0) — PASS

| fps | recovered offset | matched |
|-----|------------------|---------|
| 30  | −0.01 ms | 974/974 |
| 60  | −0.02 ms | 1948/1948 |
| 120 | −0.01 ms | 3896/3896 |

Intrinsic bias floor is ≈ 0.

### 3.4 Offset-recovery precision (`selftest`, 10 injected offsets −50…+80 ms)

| fps | noise (°/s) | mean err | RMS err | max err | recovered |
|-----|-------------|----------|---------|---------|-----------|
| 24  | 0 | 0.236 | 0.367 | 0.69 | 10/10 |
| 24  | 1 | 0.236 | 0.367 | 0.69 | 10/10 |
| 24  | 3 | 0.232 | 0.353 | 0.56 | 10/10 |
| 30  | 0 | 0.304 | 0.384 | 0.71 | 10/10 |
| 30  | 1 | 0.309 | 0.389 | 0.71 | 10/10 |
| 30  | 3 | 0.321 | 0.403 | 0.71 | 10/10 |
| 60  | 0 | 0.204 | 0.338 | 0.58 | 10/10 |
| 60  | 1 | 0.204 | 0.338 | 0.58 | 10/10 |
| 60  | 3 | 0.179 | 0.302 | 0.57 | 10/10 |
| 120 | 0 | 0.102 | 0.246 | 0.56 | 10/10 |
| 120 | 1 | 0.045 | 0.171 | 0.54 | 10/10 |
| 120 | 3 | 0.045 | 0.171 | 0.54 | 10/10 |

(All values in ms.) Numbers are reproducible via `tools/run_autosync_eval.sh`.

## 4. Conclusions

- The port recovers timestamp offsets to **sub-millisecond accuracy** on real DJI motion:
  **~0.4 ms RMS, ≤ 0.71 ms max** at 30 fps, improving to **~0.17 ms RMS** at 120 fps.
- Accuracy is **noise-tolerant** (negligible change up to 3 °/s added noise) thanks to the 20 Hz
  forward-backward low-pass and least-squares averaging over hundreds of frames.
- Residual error is a small **positive bias (~0.3 ms)** from the nearest-upper IMU-sample lookup
  on the ~1 kHz grid — inherent to the algorithm and shrinking with higher rates; it is faithful
  to the Rust implementation (kept for parity).

## 5. Known limitation

On **smooth / periodic** synthetic signals (pure sinusoids), the cost function is multi-modal
(autocorrelation sidelobes), so the coarse search can settle ~1–2 ms off-peak. Real broadband
motion — and the random-walk test fixture — produce a sharp, unique minimum and the sub-ms
accuracy above. This mirrors real autosync, which relies on rich scene/handheld motion.

## 6. Possible next steps

- Optional interpolated (vs nearest-upper) IMU lookup to remove the ~0.3 ms bias for the
  evaluator use-case (kept off by default for Rust parity).
- Port `find_offset::visual_features` (method 1) and `rs_sync` (method 2) for completeness.
- Wire real optical-flow `estimated_gyro` in, to drive the same `findOffset` from decoded video.
