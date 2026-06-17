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
  evaluator use-case (kept off by default for Rust parity). **Done in `py_autosync`
  (`interp=True`); see §8.** Port the same switch to C++ `findOffset`.
- Port `find_offset::visual_features` (method 1) and `rs_sync` (method 2) for completeness.
- Wire real optical-flow `estimated_gyro` in, to drive the same `findOffset` from decoded video.

## 7. Development log (brief)

Chronological record of how this was built and the decisions/findings along the way.

1. **Survey the Rust source.** Read `synchronization/{mod,autosync}.rs`,
   `find_offset/essential_matrix.rs`, and `filtering.rs` to pin down the exact algorithm:
   per-range 20 Hz forward-backward low-pass, 1 ms coarse sweep → 0.01 ms refine, weighted
   least-squares cost with a nearest-upper IMU lookup. Chose `offset_method = 0` (default,
   most testable) as the port target.
2. **Inspect the data.** `data/` holds ~32 k DJI fused-quaternion samples (~989 Hz) + 973
   video frames (~30 fps); `dji_quat_analysis.csv` already carries quaternion-derived
   `quat_omega_*`, confirming the "generate gyro from quaternions" approach.
3. **Plan doc** (`AUTOSYNC_TIME_PLAN.md`) written before coding: scope, components, evaluation
   method (inject a known offset, measure recovery).
4. **Core implementation** (`timesync.{hpp,cpp}`): ported the RBJ Butterworth biquad
   (Direct-Form-II-Transposed) incl. the `2·f0 > fs` Nyquist skip; `quaternionsToAngularVelocity`;
   `findOffset`. Used signed-microsecond keys for the IMU lookup instead of Rust's
   `as usize` (which would clamp the DJI clip's small negative start timestamps to 0).
5. **CLI + tests + CMake**, then first build/run. Two issues surfaced and were fixed:
   - Release defines `NDEBUG`, so `assert()`-based tests were silently no-ops → replaced with a
     `CHECK` macro that always runs.
   - The ω cross-check matched the Python tools exactly (14.1351 deg/s = 0.246703 rad/s, …).
6. **Debugged a recovery bias.** On a smooth multi-tone sinusoid the recovered offset was ~1–2 ms
   off. Dumped the cost curve and bisected the cause with variants: `nolpf+upper` → +0.33 ms,
   `lpf+upper` → −1.0 ms. Conclusion: at low video fps the 20 Hz low-pass interacts with the
   quasi-periodic signal's autocorrelation sidelobes (multi-modal cost) — an inherent property
   of the method, not a port bug.
7. **Validated on real data:** 30 fps recovery = **0.4 ms RMS**, and the bias is the expected
   ~0.3 ms nearest-upper-lookup floor. Switched the unit-test fixture from sinusoids to a
   band-limited random walk (AR(1) body rates), which mimics real motion and gives a sharp,
   reliable minimum (max err 0.34 ms).
8. **Evaluation + docs:** added `tools/run_autosync_eval.sh` (build → ctest → ω check → baseline
   → fps×noise table), `AUTOSYNC_TIME_TESTING.md`, and this report; added a noise-robustness
   unit test (drift 0.000 ms up to 5 deg/s). `ctest` 2/2 green.

Faithfulness note: where Rust behaviour and "more correct" behaviour diverged (nearest-upper
lookup bias, low-fps low-pass interaction), parity was kept and the effect documented rather than
silently "fixed", so results stay comparable to the Rust engine.

## 8. Precision characterisation & higher-precision approaches

### 8.1 Measured precision (variance study)

Monte-Carlo on the bundled DJI clip (Python `py_autosync/variance_experiment.py`, 30 fps).
Two error components are distinct in nature:

- **Random jitter (variance):** σ ≈ **0.05 ms** at realistic SNR (peak |ω| ≈ 530 °/s). Re-running
  on identical input is deterministic (variance 0). Only rises past ~0.15 ms when injected noise
  reaches 20–40 °/s. A constant gyro **zero-bias is ≲0.1 ms even at 5 °/s** — alignment is driven
  by the AC shape, not the DC level.
- **Systematic bias:** **+0.2…0.5 ms**, deterministic, bounded by one IMU sample interval. This is
  the dominant spread across different offsets/segments, *not* random variance.

**Accuracy ceiling = one IMU sample interval (≈1 ms @ 1 kHz)** because the nearest-upper lookup
snaps each video sample to a discrete IMU sample. ⇒ repeatability ~0.05 ms, absolute accuracy
~1 IMU sample. A ~1 ms spread across segmented estimates is therefore *not* noise; it is the
per-segment ceiling plus possible real **clock drift** (diagnose by trend vs time) or low-motion
conditioning.

### 8.2 Interpolated lookup result

Implemented `interp=True` in `py_autosync` (linear-interpolated IMU lookup + symmetric refine
window). Measured at 30 fps: bias **+0.166 → −0.005 ms**, repeat-error **+0.446 → +0.009 ms**
(~50× better). Default stays nearest-upper for Rust/C++ parity. Also fixed a latent refine-window
bug (original stepped only `[center-2, center]`, one-sided).

### 8.3 Higher-precision calibration approaches (survey)

Ordered by change cost → achievable precision:

1. **Sub-grid refinement (cheap, µs-level):** interpolated lookup (done); **parabolic peak
   interpolation** of the cost curve (analytic vertex of 3 points around the min) to remove the
   0.01 ms refine grid; up-sample before correlating.
2. **Frequency-domain:** **GCC-PHAT** (Knapp & Carter 1976) for a sharp sub-sample peak;
   cross-spectrum **phase-slope** (delay = −dφ/dω) for fractional delay directly.
3. **Continuous-time / model-based (gold standard, tens of µs):** B-spline continuous-time batch
   estimation (Furgale, Barfoot, Sibley, ICRA 2012) — time offset is a continuous, differentiable
   parameter jointly optimised with extrinsics/intrinsics. Furgale, Rehder, Siegwart, *Unified
   Temporal and Spatial Calibration* (IROS 2013); **Kalibr** toolbox (Rehder et al., T-RO 2016).
4. **Online / filter-based (time-varying td):** Li & Mourikis (IJRR 2014); Qin & Shen,
   VINS-Mono online temporal calibration (IROS 2018).
5. **Clock drift:** segment-wise offset + linear regression → slope = skew (ppm), resample one
   stream; i.e. an offset+skew clock model (cf. NTP/PTP).
6. **Estimation quality:** higher IMU sample rate (raises the ceiling directly); robust loss
   (Huber); inverse-covariance weighting from Allan-variance gyro noise; motion gate.
7. **Hardware sync (<<1 µs):** common clock, hardware timestamping, trigger/GenLock, PTP (IEEE 1588).

**How Kalibr substantiates its (tens-of-µs) precision:** there is no ground-truth offset on real
data, so it relies on (a) the estimator's **posterior covariance** on td (inverse Hessian),
(b) **simulation** recovery of injected offsets, (c) **NEES consistency** checks that the reported
covariance is honest, and (d) **cross-dataset repeatability** — the continuous-time spline makes td
a grid-free differentiable parameter, which is why it beats the discrete correlation ceiling.
