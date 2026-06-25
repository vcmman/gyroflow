# Autosync-Time — Design & Findings

The design rationale, algorithm, parity contract, measured precision, and the conclusions of the
precision experiments. For task-oriented usage see [`README.md`](README.md).

---

## 1. Goal

Recover the single constant time delay between a camera's rotational motion and its IMU/gyro stream
so a stabilizer can share one clock for both. Both streams measure the *same physical rotation*, so
the correct delay produces a sharp minimum in a motion-matching cost. This tool is the **time-offset
half** of Gyroflow's sync; it does not decode pixels — the camera-side motion is supplied as angular
velocity (optical flow / `estimated_gyro`, a second gyro, or — for evaluation — synthesized from DJI
fused quaternions).

## 2. Algorithm

Matches the Rust engine step for step:

1. **Angular velocity.** Each signal is `(N,3)` body-frame ω (deg/s) with per-sample timestamps
   (ms). From a quaternion attitude series, ω over each interval is `rotvec(q[i]⁻¹·q[i+1]) / Δt`
   (`quaternions_to_angular_velocity`); from a raw gyro, ω is the samples themselves.
2. **Zero-phase low-pass.** 20 Hz forward-backward Butterworth biquad (`Q = 1/√2`). Filtering twice
   (forward then backward) cancels phase, so it does not move the cost minimum. A Nyquist guard
   skips the filter when `2·f_cutoff > sample_rate` (the video side at 24/30 fps is left untouched —
   exactly what Gyroflow does).
3. **Cost.** For a candidate offset `o`, every video sample at `t` is compared to the IMU value at
   `t − o`; the cost is the mean weighted squared difference over matched samples, axis weights
   **x:70, y:70, z:100**. Valid only if > half the video samples find an IMU match.
4. **Coarse → refine.** Sweep `[initial ± search]` at **1 ms**, then refine **±2 ms** around the
   coarse optimum at **0.01 ms**.
5. **Sub-grid (optional).** `parabolic=True` fits a parabola to the three cost samples around the
   refine minimum and jumps to its analytic vertex (`δ = ½·(c₋−c₊)/(c₋−2c₀+c₊)`, guarded to a convex
   bracket with `|δ| ≤ 1`), removing the residual 0.01 ms grid step.
6. **IMU lookup — the accuracy knob.**
   - `nearest` (default): snap the query up to the next IMU sample — bit-for-bit faithful to
     Rust/C++, but with a quantization bias up to one IMU sample interval.
   - `interp`: linearly interpolate the IMU value at the exact query time — removes that bias.
7. **Acceptance.** Accept only if the optimum lies within 90% of the search window.

### Code shape
`_prepare_cost(...)` does the windowing + low-pass + lookup setup and returns a `cost_fn(offset)`;
both `find_offset` (the search) and `cost_curve` (visualization) call it, so they evaluate the
*identical* cost. Loaders (`load_gcsv`, `load_angular_velocity_csv`, `load_motion`) normalize any
input to deg/s with ms timestamps re-based to 0.

## 2b. Camera angular velocity from video (`video_omega.py`)

The camera-side ω can be derived from an MP4 instead of supplied as a CSV, porting Gyroflow's
autosync motion path (`src/core/synchronization/`). Per consecutive (sampled) frame pair:

1. **Features** — `goodFeaturesToTrack` (maxCorners 200, quality 0.01, minDist 10, blockSize 3,
   Harris off, k 0.04) on the previous frame.
2. **Track** — `calcOpticalFlowPyrLK` (win 21×21, 3 levels, criteria COUNT+EPS/30/0.01,
   minEigThreshold 1e-4); keep `status==1` points inside both frames; need ≥10.
3. **Pose** — undistort to normalized camera coords (pinhole `K⁻¹`; no distortion model), then
   `findEssentialMat(identity K, LMEDS, prob 0.999, threshold 1e-5, maxIters 4000)` and
   `recoverPose(..., distanceThresh=100000)` (matches Gyroflow's `recover_pose_triangulated`; the
   large threshold is essential — the default rejects low-parallax points on cheirality) → R, ≥10
   inliers.
4. **ω** — `rotvec = axisAngle(R) / dt`, then **swap X/Y** and convert to deg/s (the exact
   convention of Gyroflow's `estimated_gyro`), timestamped at the **frame-pair midpoint** (the motion
   happened *between* the frames). `dt` is the real timestamp gap (VFR-safe), from
   `CAP_PROP_POS_MSEC` with an fps fallback.

**Intrinsics.** `K` comes from `--video-focal-px`, else a horizontal `--video-fov-deg`, else a ~50°
default. For autosync only the *timing/shape* of the motion matters; a focal-length error mostly
rescales the amplitude, not the recovered offset.

**Accuracy.** The essential-matrix rotation is well-determined in *direction/timing* but its
*amplitude* is approximate when rotation mixes with low translation (validated: the recovered ω
series correlates strongly — r≈0.8–0.97 — with a known time-varying camera rotation, while the
absolute scale carries a mild bias). That is exactly what `find_offset` needs, since it correlates
the time-shape, not the amplitude. The pose geometry itself is exact (validated on synthetic 3D
point correspondences). OpenCV is imported lazily, so the rest of the toolkit runs without it.

## 3. Parity contract

The default `nearest` path is line-for-line faithful: 1 ms coarse sweep → 0.01 ms refine over ±2 ms,
weighted least-squares cost (x,y ×70, z ×100), nearest-upper IMU lookup (microsecond keys, truncated
toward zero), 90% acceptance, and the `2·f0 > fs` Nyquist skip. On `data/dji_quaternions_full.csv`
recovered offsets and costs match `cpp_core/build/gyroflow_autosync` to 4 decimals wherever the
minimum is well-defined; ties can differ by at most one 0.01 ms refine step from float accumulation.
`scipy.signal.lfilter` (zero initial state) reproduces the Rust biquad `DirectForm2Transposed` to
~4e-13. `interp`/`parabolic` intentionally break bit-exact parity in exchange for accuracy.

## 4. Measured precision & experiment conclusions

Reproduce with `variance_experiment.py`, `segment_experiment.py`, `segment_stats.py`.

### Interpolation removes a systematic bias (DJI clip, 30 fps)

| mode | bias | std | repeat-error @7.3 ms |
|------|------|-----|----------------------|
| `nearest` (default) | +0.166 ms | 0.210 ms | +0.446 ms |
| `interp` | **−0.005 ms** | **0.020 ms** | **+0.009 ms** |

≈ 50× better absolute accuracy. **Why:** `nearest` snaps every query up to the next IMU sample, a
deterministic same-sign rounding. That bias (+0.2…0.5 ms, bounded by one IMU sample) does **not**
average out; only `interp` removes it. The random jitter floor is ≈ 0.05 ms.

The interp path also fixed a latent refine-loop bug: the original stepped 200×0.01 ms from
`center−2`, searching the one-sided `[center−2, center]` and never climbing above the coarse pick;
the interp path sweeps the full symmetric `[center−2, +2]`. `parabolic` then removes the last
0.01 ms grid step (a few µs).

### Segmented estimates that "differ by ~1 ms" are not drift

Inject one fixed offset over the whole clip, estimate per segment (`segment_experiment.py`):

- Per-segment estimates scatter ~1 ms **even when the truth is constant** — this is per-segment
  conditioning + lookup quantization, not a time-varying offset. Shorter segments scatter more
  (per-seg std 0.16 ms @16 s → 0.57 ms @1 s).
- The **clip-mean is far more accurate than any single segment**, but for two distinct reasons:
  *random* scatter averages out (std of the mean ~0.02–0.04 ms, shrinking ≈ 1/√N), while the
  *systematic* `nearest` bias does **not** (clip-mean stays at +0.3…0.4 ms for any segment count).
  Only `interp` removes it.
- **Diagnosis by pattern:** a monotonic trend vs segment time ⇒ real camera/IMU **clock drift**
  (offset is time-varying; needs offset+skew, not a single constant); random scatter correlated with
  low-motion / shallow-cost segments ⇒ poorly conditioned segments; otherwise it is lookup
  quantization (use `interp`).

**Net:** `interp` (kills the bias) + averaging well-excited segments (kills the variance) reaches
~0.02 ms; averaging alone leaves the `nearest` bias intact.

### Worked example (200 Hz IMU, 60 fps, true offset 8.5 ms)

```
nearest          -> 10.0000 ms   (+1.5 ms quantization bias; visible as a staircase cost curve)
interp+parabolic ->  8.4940 ms   (−0.006 ms error; smooth cost valley)
```

## 5. Variable frame timing (VFR)

The pipeline is timestamp-based, so non-uniform spacing is handled in most places:

1. **Offset search** matches on each sample's real timestamp (`gyro at of.t[k] − offs`), never on
   `k·dt`.
2. **ω magnitude** divides each interval's rotation by its actual `Δt`, not a constant fps.
3. **Low-pass** is the only part assuming a uniform rate (the biquad takes one scalar
   `sample_rate`). Rarely bites: at 20 Hz cutoff the video side is skipped by the Nyquist guard for
   24/30 fps, and the IMU is a steady ~1 kHz stream. For heavy VFR on a filtered side, use the
   median frame interval as the nominal rate, resample onto a uniform grid before filtering (match
   on real timestamps afterward), or a time-aware filter (deviates from parity).

## 6. Limitations & future work

**Limitations**
- Does not decode video — camera-side motion must be supplied as angular velocity.
- Estimates a single constant offset, not clock drift (offset + skew).
- Low-pass assumes a uniform sample rate (§5).
- Motion-gated: unreliable below ~3 deg/s of excitation.

**Future work** (ordered cheap → research-grade)
- Accept a quaternion CSV directly for `--video` (derive ω internally).
- Normalized 0–1 confidence score (from cost + curvature) and JSON output for pipelines.
- **Clock drift:** segmented offset + linear fit → report skew (ppm); offer offset+skew correction
  (resample one stream onto the other's clock) with a "trend (drift)" vs "scatter (conditioning)"
  diagnostic.
- **GCC-PHAT** / cross-spectrum phase-slope estimator as a sub-sample alternative to the time-domain
  sweep; Huber loss + inverse-covariance weighting + per-segment motion gating.
- Port `interp`/`parabolic` to the C++ `findOffset` for cross-port parity; validate against a
  continuous-time (Kalibr-style) reference.
