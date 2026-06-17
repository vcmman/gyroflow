# C++ Stabilizer — Algorithm Pipeline & Parameters

This documents the exact algorithm flow and parameters that produced the result video
`data/DJI_20260605174353_0032_D_cpp_stabilized.mp4` (DJI Osmo Action 4 clip, fused-quaternion
path). It is a record of the **current** `cpp_core` stabilizer; the per-stage Rust references
are noted so the math can be checked against Gyroflow.

## Command used

```sh
./cpp_core/build/gyroflow_cpp_stabilize \
    data/DJI_20260605174353_0032_D.MP4 \
    --telemetry data/dji_bridge.json \
    -o data/DJI_20260605174353_0032_D_cpp_stabilized.mp4 \
    --crf 18
```

All other knobs were left at their defaults (adaptive zoom on, 16:9 output, ffmpeg H.264 +
audio copy). The telemetry bridge JSON was exported beforehand from the `.gyroflow` project
via `tools/export_bridge_json.py`.

## Input (this clip)

| Property | Value |
|----------|-------|
| Source | DJI Osmo Action 4, fused attitude (no IMU integration needed) |
| Input video | 3840×2880 (4:3), 29.97 fps, 973 frames, 10-bit HEVC (decoded 8-bit via OpenCV) |
| Lens / distortion model | `opencv_fisheye` |
| Camera matrix (calib=input) | fx = fy = 1457.0737 px, cx = 1920, cy = 1440 |
| Distortion coeffs k0..k3 | 0.155131, 0.137141, −0.093861, 0.004170 |
| Calibration dimension | 3840×2880 (== input ⇒ no calib→input rescale) |
| Output dimension | 3840×2160 (16:9) |
| Frame readout time | 21.8172 ms, direction TopToBottom (vertical rolling shutter) |
| Attitude samples | 32 465 fused quaternions (scalar-first w,x,y,z, µs timestamps) |

## Pipeline (in order)

```
bridge JSON ─► smoothing ─► adaptive zoom (per-frame FOV) ─► per-frame/-row transform ─► undistort kernel ─► ffmpeg encode
              default_algo  zooming/adaptive_zoom            stabilization/frame_transform stabilization/undistort
```

### 1. Telemetry ingestion — `telemetry_io`
Loads the bridge JSON into `FileMetadata` + `LensProfile`: org (raw fused) quaternion series
(µs→ms), camera matrix, distortion coeffs, readout time/direction, fps, dimensions. No DJI
protobuf parsing in C++ yet (bridge-first; Phase 3 will replace this).

### 2. Smoothing — `smoothing/default_algo` (ref `src/core/smoothing/default_algo.rs`)
Velocity-adaptive forward/backward SLERP filter producing the smoothed **orientation** series
P(t) at the input quaternion timestamps.
- Per-sample angular velocity → time constant `τ` interpolated between `max_smoothness`
  (zero velocity) and `alpha_0_1s` (max velocity), scaled by `smoothness`.
- Smoothing alpha per step: `α = 1 − exp(−(Δt)/τ)`.
- Forward pass, then a distance-modulated **second pass** (`second_pass = true`).
- `camera_diagonal_fov` for this clip ≈ **117.48°** (= `2·atan(diag/(2·fy))`, diag of
  3840×2880); `fov_ratio = camera_diagonal_fov / 120`.

| Param | Value |
|-------|-------|
| smoothness | 0.5 |
| max_smoothness | 1.0 s |
| alpha_0_1s | 0.1 s |
| per_axis | false |
| second_pass | true |
| camera_diagonal_fov | ≈117.48° (computed) |

Validated vs Gyroflow `stab_quat`: max 0.0147°, mean 0.0037° over 973 frames.

### 3. Adaptive zoom — `zooming/adaptive_zoom` (ref `fov_iterative.rs` + `zoom_dynamic.rs`)
Per frame, forward-maps a dense border ring of source pixels into stabilized output space
(inverse fisheye + `new_K·R` reproject, `W>0` behind-camera guard), inscribes the largest
centred rectangle of the **output** aspect (16:9), and converts to a per-frame FOV. Then
temporal smoothing + clamp:
- Border ring, `new_K` centre, search centre and FOV denominator are in **input** dims;
  only the inscribed-rectangle aspect uses the **output** dims (mirrors Gyroflow temporarily
  setting output=input during the FOV calc, `zooming/mod.rs:48`).
- Per-frame timestamp = `frame·1000/fps`.
- Temporal smoothing = **EnvelopeFollower** (method 1, DJI default), two passes:
  `α₁ = 1 − exp(−(1/fps)/window)`, `α₂ = 1 − exp(−(1/fps)/0.2)`.
- `max_zoom` clamps FOV to `≥ 100/max_zoom_percent`.

| Param | Value |
|-------|-------|
| adaptive zoom | ON (default) |
| window_s | 4.0 s |
| method | 1 (EnvelopeFollower) |
| max_zoom_percent | 130 (FOV ≥ 0.769) |
| fov_algorithm_margin | 2.0 px |
| center_offset | (0, 0) |

Resulting per-frame FOV on this clip: range ≈ [0.974, 1.245]. Validated vs Gyroflow
`fov_scale`: max 0.0012% rel error.

### 4. Per-frame / per-row transform — `stabilization/frame_transform` (ref `frame_transform.rs`)
For each output frame (center timestamp `frame·1000/fps`) builds one 3×3 matrix **per source
scanline** for rolling-shutter correction.
- Virtual camera `new_K`: `fx/fov`, `fy/fov` on the diagonal, principal point at the
  **output** centre `(out_w/2, out_h/2)`; `fov = fov_frame · width/output_width` (here
  width==output_width ⇒ factor 1).
- Rolling shutter: `row_readout = readout / input_height`, `start_ts = center − readout/2`,
  `row_ts(y) = start_ts + row_readout·y`.
- Per row: `quat = P(center)⁻¹ · org(row_ts)` (algebraically equal to Gyroflow's
  `smoothed(center)·org(center)⁻¹·org(row_ts)`), `R = R(quat)` with the non-inverted-
  framebuffer axis sign flips, `i_r = inverse(new_K·R)` stored as 9 floats.
- Matrix count = **input** readout-axis length (one per source row), not output.

### 5. Undistort kernel — `stabilization/undistort` (ref `cpu_undistort.rs` + `stabilize.rs`)
For each **output** pixel (0..3840 × 0..2160):
1. Two-pass rolling-shutter index: map the pixel with the middle matrix to a source
   coordinate, take its source-row → select `i_r` for that row (clamped to input dims).
2. `(_x,_y,_w) = i_r·(x,y,1)`; reject if `_w ≤ 0` (background).
3. OpenCV-fisheye `distort_point`: `θ=atan(r)`, `θ_d = θ(1+k0θ²+k1θ⁴+k2θ⁶+k3θ⁸)`, scale.
4. `(u,v) = distorted·f + c` → bilinear sample of the source frame; out-of-bounds = black.
- Threaded over output rows; core stays OpenCV-free (the CLI owns the buffers).

| Param | Value |
|-------|-------|
| interpolation | bilinear (Gyroflow default is Lanczos4; ~0.6 dB factor — see `COMPARISON.md`) |
| background | black (0,0,0) |
| threads | auto (`hardware_concurrency`) |
| output size | 3840×2160 (16:9, default = lens `output_dimension`) |

### 6. Encode — `tools/gyroflow_cpp_stabilize`
Raw BGR frames piped to ffmpeg.

| Param | Value |
|-------|-------|
| encoder | libx264 (H.264) |
| CRF | 18 (visually lossless) |
| preset | medium |
| pixel format | yuv420p |
| audio | copied from source (AAC) |

## Output

`data/DJI_20260605174353_0032_D_cpp_stabilized.mp4` — 3840×2160 H.264, 973 frames @ 29.97 fps,
AAC audio, ~745 MB.

## Result quality (sample clip)

- **Stabilization** (`tools/stabilization_quality.py`, before→after): global camera shake
  (phase-correlation) **−89%** (10.5→1.2 px), optical-flow motion −40% (residual ≈7.6 px is
  the walking subject — real scene motion), ITF +1.56 dB.
- **Fidelity vs Gyroflow** (same params; full head-to-head in `COMPARISON.md`): smoothing
  ≤0.0147°, adaptive fov ≤0.0012% (golden metadata); frame PSNR ~33.5 dB both-bilinear
  (residual ~0.11 px, quaternion-sampling FP + resampling — not mainly interpolation); both
  remove the same camera shake (89.0% vs 88.9%).
- **Black border**: worst-frame ≈0.16% (16:9 crop) with adaptive zoom.

## Not applied (this path)

IMU integration (DJI gives fused attitude), horizon lock, IBIS/OIS, digital lens / mesh
correction, light refraction, focal-plane distortion, `lens_correction_amount < 1`,
per-axis smoothing, keyframes, GPU. See `DEVELOPMENT_PLAN.md` / `TODO.md` for the roadmap.
