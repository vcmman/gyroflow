# cpp_core — status & next steps (resume here)

Headless C++ port of Gyroflow's stabilization core. Self-contained CMake project; math
validated against Rust Gyroflow's golden metadata. Last updated 2026-07-08.

## Current status (one line)

Phase 1 (headless DJI stabilizer) + Phase 2 (dynamic zoom + `output_dimension` framing) are
**complete and golden-validated**, including **both** adaptive-zoom methods. End-to-end on
real DJI footage (Osmo Action 4 sample + Osmo Action 6 dji6 6.6-min clip); on par with
Gyroflow to a sub-pixel margin. Input decode is still 8-bit (OpenCV) and telemetry is still
bridged from the Rust binary — those are the two biggest remaining gaps.

---

## ✅ Implemented

- **Headless stabilizer** (`tools/gyroflow_cpp_stabilize.cpp`): bridge-JSON telemetry →
  smoothing → per-frame/-row transform → CPU undistort → ffmpeg encode (H.264/H.265 + audio).
  OpenCV used only for video I/O.
- **Smoothing — Default algo** (`smoothing/default_algo.*`): velocity-adaptive slerp (time
  constants `max_smoothness`↔`alpha_0_1s`, velocity-normalized), two-pass with the distance
  term. Matches golden `stab_quat` ≤0.0166° (dji6 full 11 934 frames). Includes the **per-axis**
  branch (`--per-axis`, `smoothness_pitch/yaw/roll`; euler decompose/recompose) — golden ≤0.016°.
- **Rolling-shutter correction** (`stabilization/frame_transform.*`): one 3×3 matrix per
  readout-axis row, each sampling attitude at that row's exposure time; kernel indexes by
  source row. Active whenever `frame_readout_time_ms > 0` (dji6: 12.98 ms, TopToBottom).
- **Distortion** (`distortion/opencv_fisheye.*`): OpenCV fisheye distort/undistort.
- **Dynamic zoom — BOTH Gyroflow methods** (`zooming/adaptive_zoom.*`): per-frame inscribed-
  rectangle FOV (dense-scan port of `fov_iterative`) + temporal smoothing via
  **EnvelopeFollower** (default) **and GaussianFilter**, selectable with `--zoom-method`;
  `max_zoom` clamp. Both match golden `fov_scale` ≤0.0012% (sample) / ≤0.0047% (dji6 full).
- **Framing**: renders lens `output_dimension` (16:9) by default; `--keep-sensor` (4:3 full
  sensor) / `--output-size WxH`.
- **Telemetry bridge** (`telemetry_io.*`, `tools/export_bridge_json.py`): consumes fused
  attitude quaternions + lens profile + readout from the Rust Gyroflow CLI.
- **DCR gate / `--enhanced` preset** (`--dcr`, SMOOTHING_RND §1/§8e): direction-consistency
  gating on the velocity dampening — the validated Tier-1 enhancement (−31…35 % rendered
  vertical shake, black border unchanged). `--enhanced` ≡ `--dcr`; golden default untouched.
- **Finite look-ahead** (`--look-ahead S`, §7): in-camera-realizable smoothing — backward pass
  limited to S seconds of buffered future (0 = offline/golden).
- **Zoom FOV look-ahead** (`--zoom-look-ahead S`, §8h): real-time dynamic-zoom envelope with S
  seconds of future; fixes causal zoom pops (~5–10× smaller jumps), borders already 0 (<0 =
  offline/golden).
- **Portable reference** (`examples/dynamic_zoom_reference.cpp`): self-contained, dependency-free
  dynamic-zoom algorithm for board-side porting (see `examples/README.md`).
- **Validation tooling**: `gyroflow_cpp_validate` (+ `--zoom-method`, `raw_fov` column = required
  FOV pre-smoothing/clamp), `compare_gyroflow_metadata.py` (math vs golden),
  `stabilization_quality.py` (ITF + residual motion), image/telemetry eval scripts
  (`tools/README.md` §3).
- **Tests**: ctest **7/7** (core, fisheye, telemetry_io, transform, adaptive_zoom [both
  methods, constant + varying signal], imu_orientation, smoothing [scalar + per-axis]).

> Field result (dji6 0005, full 11 934-frame renders, 16:9 + 4:3): global camera shake
> −62…63%, ITF +1.83 dB, on par with Gyroflow's own render (steadier only by ~0.05 px). See
> `COMPARISON.md`. Coordinate conventions in `PIPELINE.md` ("Coordinate conventions").

---

## TODO (priority order)

### 0. Algorithm quality — from the evaluation campaign (`EVALUATION_SUMMARY.md`)
- **0a. Velocity-adaptive Gaussian** (best value, ~1 day): drive the Gaussian kernel's per-frame σ
  from the velocity ratio (± DCR gate) that `default_algo` already computes — chase DCR's `dy`
  (amplitude) *and* the Gaussian's low acceleration (smoothness) at once. **Acceptance gate**
  (tools ready): `dy` ≤ DCR AND accel < DCR on run+bike, black border not increased
  (`vertical_flow_compare.py` + `angular_derivatives_compare.py` + `zoom_vs_maxzoom.py`).
  Passing → becomes the new `--enhanced`. Base kernel: `claude/gaussian-smoothing`.
- **0b. Frame-periphery residual**: the only place DJI leads (smooth biking, 4:3 edges,
  EVALUATION_SUMMARY §3/§6). Investigate per-row rolling-shutter interpolation and fisheye-model
  accuracy at high image radii; band analysis already localizes it.
- **0c. Branch consolidation**: merge `claude/gaussian-smoothing` (verified low-risk — only 2
  Markdown conflicts, CLI/CMake auto-merge); **rebase** `claude/speed-bump-jolt-rnd` (L1) — do NOT
  naive-merge (real conflicts in both CLI tools; it predates DCR/`--enhanced`/`raw_fov`). On
  rebase, unify L1's API to `(quats, duration_ms, params)` + same-timestamp output, share
  default_algo's euler↔quat helpers, replace fixed 2000 ADMM iterations with a convergence test.
- **0d. Translation-domain stabilization** (research, biggest headroom): the visible "running
  float" is translational parallax no rotational smoother reaches (`SMOOTHING_RND.md` §3);
  needs optical-flow translation smoothing + crop budget. Start with a design doc.

### 1. Native libav 10-bit decode (input)  ← highest-value platform gap
Input is still `cv::VideoCapture` (8-bit BGR), truncating 10-bit/HDR DJI footage. Output
already pipes to ffmpeg. Replace decode with libav / `ffmpeg -f rawvideo` and carry bit depth
into the kernel (the undistort kernel would need a 16-bit sample path). Ref
`src/rendering/ffmpeg_video.rs`. Pixel-fidelity only — does not affect the validated geometry.

### 2. Horizon lock (`src/core/smoothing/horizon.rs`)
Parity gap; also the cheapest win for the jolt case (locks roll so the frame doesn't tilt on
a jolt). Needs the gravity reference — pure-quaternion horizon works without accel; full
`use_gravity_vectors` needs accel (see item 4).

### 3. Higher-order interpolation in `undistort.cpp`
Currently bilinear; Gyroflow defaults to Lanczos4. The `COEFFS` tables in `cpu_undistort.rs`
port directly. Add `--interpolation bilinear|bicubic|lanczos4`. Output-quality only (~0.6 dB
in the same-interp PSNR test — not the path to closing the Gyroflow gap).

### 4. Raw-IMU integration + accelerometer input (`src/core/imu_integration/`)
cpp_core currently consumes **only fused quaternions** — no `complementary`/`complementary_v2`
/`vqf`, and **accelerometer is not used at all**. Needed for sources that expose only raw
gyro+accel (no fused attitude), and a prerequisite for accel-driven features (horizon gravity
vectors, jolt detection). Requires carrying gyro+accel streams through the bridge/parser.
Mind the hardcoded integrator swap `omega=(-g[1],g[0],g[2])` + `imu_orientation` (PIPELINE.md).

### 5. Native DJI telemetry parse (drop the bridge)
Port `djmd`/DVTM protobuf parsing + readout extraction + lens-profile DB (CBOR+gzip) so
`export_bridge_json.py` / the Gyroflow binary are no longer needed. Refs
`src/core/gyro_source/mod.rs`, `lens_profile_database.rs`, the `telemetry-parser` crate. (Also
where DJI's native quaternion axis convention is normalized into the orientation frame.)

### 6. Algorithmic R&D beyond Gyroflow — severe jolts ("大坑") + running float
Gyroflow's velocity-adaptive low-pass *loosens* smoothing at high velocity, so it can't
distinguish an intentional fast pan from an unintentional jolt/bob and passes it through;
adaptive zoom then "pumps" or hits `max_zoom` (black borders). The speed-bump jolt research
(scenario, severity sweep, gate prototype) is recorded in **`cpp_core/JOLT_RND.md`**;
the L1 jerk-limiting comparison is in **`cpp_core/SMOOTHING_RND.md`** §5 (L1 code on this
branch: `--smoothing l1`).

**Finding so far (JOLT_RND E8) — smoothing-only jolt rejection is NOT a clear win, decision
deferred.** On a *synthetic* pure-oscillation bump a moderate gate cuts output-path jerk
~35–40% at zero zoom cost; but on *real* dji6 jolts it only helps ~12% at jr≈0.3 and regresses
past that (jerk rises, zoom eaten toward the `max_zoom` clamp). Real jolts = oscillation + a
small *sustained* attitude shift: the oscillation is cheap to reject, the sustained part costs
crop, so a smoothing-only gate's real-world gain is marginal and strength-fragile. Deferred
next steps: (a) real-footage validation on an actual speed-bump clip; (b) crop-constrained
joint smoothing↔zoom (the root fix; couples smoothing to the zoom margin).

The related **running low-frequency vertical float** case has its own record in
**`cpp_core/SMOOTHING_RND.md`**: the **DCR** (Direction Consistency Ratio) gate — `--dcr`,
**merged** — cuts the rotational vertical bob −45…75% by only loosening when motion is fast AND
directionally consistent. But an image-domain cross-check shows the *visible* running float is
dominated by **translational parallax** (unremovable by any rotational smoother — the real
ceiling), and a Gaussian base kernel beats EMA+DCR on jerk at equal crop. Candidate upgrades:
- **Translation-domain residual stabilization** (optical-flow 2-D / depth-aware) — the actual
  fix for the visible running float; rotation-only methods cannot reach it.
- **Gaussian / linear-phase base kernel** as `--smoothing gaussian` (drops DCR; far lower jerk).
- **Jerk/transient detection + non-causal variable-window smoothing** (widen the window /
  reduce follow around detected impacts) — most targeted, medium cost.
- **L1-optimal camera path** (Grundmann 2011) as an alternative smoothing mode: crop-bounded
  constant/linear/parabolic path; absorbs transients without breathing.
- **Spike/outlier rejection** (Hampel/median) on the attitude before smoothing.
- ✅ Done: **jerk-RMS / P95 / ITF-P05** metrics added to `stabilization_quality.py`; IMU-layer
  `tools/jolt_analysis.py`; `tools/make_synthetic_jolt.py` (bump + gaussian profiles).

### 7. Remaining parity (lower priority)
- ✅ **Per-axis smoothing** — ported (`DefaultAlgoParams::per_axis` + `smoothness_pitch/yaw/roll`,
  `--per-axis`/`--smoothness-*` on validate + stabilize). Euler decompose/recompose matching
  nalgebra's intrinsic X-Y-Z convention; scalar path untouched (bit-identical). Golden-validated
  vs Gyroflow with distinct sliders: smoothed ≤0.016°, fov ≤0.0015% (ctest `smoothing`).
- `lens_correction_amount < 1`; color-range (full/limited).
- Other smoothing algos: `plain`, `fixed`, `none`, `focal_length` (only `default_algo` ported).
- More distortion models (`opencv_standard`, poly3/5, …).
- Optional GPU backend (OpenCL/wgpu).
- Keyframed dynamic-zoom window (zooming-speed keyframes / video-speed) — intentionally not
  ported; the headless bridge carries no keyframes.

---

## Quick resume commands
```sh
cmake --build cpp_core/build -j && (cd cpp_core/build && ctest --output-on-failure)   # 7/7

# stabilize (default: 16:9 crop, EnvelopeFollower zoom; --keep-sensor for 4:3,
#            --zoom-method gaussian for the other method)
python3 tools/export_bridge_json.py INPUT.MP4 -o bridge.json        # needs a Gyroflow binary
./cpp_core/build/gyroflow_cpp_stabilize INPUT.MP4 --telemetry bridge.json -o out.mp4 --codec h265

# validate math vs golden (encoder-independent), per method:
gyroflow PROJECT.gyroflow --export-metadata "3:/tmp/gf_meta.json"
./cpp_core/build/gyroflow_cpp_validate bridge.json --frames N --zoom-method envelope > /tmp/cpp.csv
python3 tools/compare_gyroflow_metadata.py /tmp/gf_meta.json /tmp/cpp.csv             # expect PASS

# stabilization quality (before vs after):
python3 tools/stabilization_quality.py --compare INPUT.MP4 out.mp4 --max-frames 300
```
Sample clip + project + bridge live under `data/` (gitignored): `DJI_20260605174353_0032_D`.
