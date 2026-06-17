# cpp_core — Next steps (resume here)

Status as of last session: Phase 1 (headless DJI stabilizer) and Phase 2 (adaptive zoom +
**Gyroflow output_dimension framing**) work end-to-end on `data/DJI_..._0032_D.MP4`; output
is encoded via ffmpeg (H.264/H.265 + audio). 5/5 unit tests pass. **Validated against golden
Gyroflow metadata**: smoothing ≤0.0147°, adaptive fov ≤0.0012% over 973 frames; frame PSNR
~33.5 dB (residual ~0.11 px, sub-pixel — quaternion-sampling FP + resampling, not mainly
interpolation). **Stabilization quality**: global camera shake −89% (Gyroflow −88.9%, i.e.
equal), ITF +1.58 dB. Full head-to-head in `COMPARISON.md`; see also `README.md` /
`DEVELOPMENT_PLAN.md`.

Pick up from the top. Each item lists the concrete files to touch.

---

## 1. ✅ DONE — match Gyroflow framing (input vs output dimensions)

The C++ now renders Gyroflow's lens `output_dimension` by default (this clip: **3840×2160,
16:9** from the 3840×2880 4:3 sensor); `--keep-sensor` and `--output-size WxH` switch modes.
Black-border fraction on the 16:9 crop stays ≈0.16% worst (clip-boundary frame), i.e. the
crop is self-consistent — no large bottom border. What was done:

- `stabilization/frame_transform.cpp` — `TransformParams` gained `output_width/height`;
  `new_K` principal point = `output_w/2, output_h/2`, `fov` scaled by `width/output_width`
  (`get_fov`); source `c` stays at input `cx,cy`; per-row matrix count stays in input dims.
- `stabilization/undistort.cpp` — kernel iterates **output** pixels and now picks the
  rolling-shutter matrix by the **source** row (two-pass: map output→source with the middle
  matrix, then index), mirroring `stabilize.rs::undistort`'s `sy`. Needed because the matrix
  count = input readout-axis length, not output.
- `zooming/adaptive_zoom.cpp` — inscribed-rect aspect now uses **output** dims
  (`output_inv_aspect`); border ring, `new_K` centre, search centre and fov denominator stay
  in **input** dims (Gyroflow sets output=input during the fov calc, so `output_dim.0`
  collapses to the input width — see `zooming/mod.rs:48`).
- CLI: `--keep-sensor`, `--output-size WxH`; output buffer + encoder sized to output dims.
- `tests/test_transform.cpp` — round-trip test for output≠input: matrix count stays in input
  dims, source intrinsics unchanged, and the fov-scaling identity
  `out_fov = centre + (out_fov1 − centre)/fov` holds with output centre ≠ input centre.

## 2. ✅ MOSTLY DONE — PSNR / golden comparison vs Rust Gyroflow

**Math cross-check (encoder-independent) — DONE, matches golden to ~0.015°/0.001%.**
`tools/gyroflow_cpp_validate` dumps smoothed quats + adaptive fovs; diff against Gyroflow's
`--export-metadata 3:...` with `tools/compare_gyroflow_metadata.py`. Over all 973 frames:
- org_quat sampling: max 0.014°, mean 0.0037° (sanity — raw quaternion interpolation)
- smoothed orientation vs `stab_quat`: max **0.0147°**, mean 0.0037° (smoothing port)
- adaptive fov vs `fov_scale`: max **0.0012%** rel, mean 0.0004% (zoom + 16:9 framing)
Conventions nailed: render samples at `frame*1000/fps`; the metadata export samples at
`frame*1000/fps + readout/2` (gyro_export middle_ts); fovs at `frame*1000/fps`. Gyroflow's
exported `stab_quat = org * smoothed^-1`, so the C++ `smoothed` series compares directly.

**Frame PSNR — DONE (with caveats).** Gyroflow's default render is blocked in this env (no
working GPU encoder; the 10-bit `YUV420P10LE` software path is rejected). Workaround:
transcode the source to 8-bit (`ffmpeg -crf 12 -pix_fmt yuv420p`) and point a copy of the
`.gyroflow` project at it (keep `gyro_source.filepath` = original for telemetry), then both
tools read identical 8-bit frames. Result (16:9, **both bilinear**, RS on): PSNR ~33.5 dB
mean, no global shift (0.005 px), matched brightness, no DC/colorspace offset. Dual-encoder
noise floor is ~38 dB, so the real geometry residual is mae ≈ 3.3 ≈ **~0.11 px** local
misalignment. Matching interpolation gained only ~0.6 dB vs Gyroflow's default Lanczos4, so
the residual is **not** mainly interpolation — it's quaternion-sampling FP/µs-rounding
(~0.09 px) + minor resampling differences. No systematic framing bug. Both outputs also
stabilize equally (camera-shake removal 89.0% vs Gyroflow 88.9%). Full head-to-head:
`COMPARISON.md`.

- [ ] (optional) The ~0.11 px residual is near the floor for two codebases + two encoders;
      not worth chasing. STMap diff is NOT useful here — Gyroflow's `--export-stmap` sets
      `suppress_rotation=true` (lens-only map).

## 3. Phase 4 — native libav decode (input)  ← do this next

Parity with Gyroflow is essentially achieved on the algorithm side (see items 1–2), so the
highest-value remaining work is removing the real quality limitation: input is still decoded
8-bit via OpenCV, truncating the clip's 10-bit/HDR.


Output already pipes to ffmpeg; **input is still OpenCV `VideoCapture` (8-bit BGR only)**, so
10-bit/HDR DJI footage is truncated.

- [ ] Replace `cv::VideoCapture` with libav decode (or pipe `ffmpeg -f rawvideo` out), keep
      bit depth, feed the kernel higher-precision pixels. See `src/rendering/ffmpeg_video.rs`
      for the decode→process→encode ordering (PreConversion/PostConversion).

## 4. Phase 3 — native DJI telemetry parser (drop the bridge)

- [ ] Port `djmd`/DVTM protobuf parsing + readout-time extraction + lens-profile DB
      (CBOR+gzip) loading/matching so `export_bridge_json.py` / the Gyroflow binary are no
      longer needed. Refs: `src/core/gyro_source/mod.rs`, `lens_profile_database.rs`, and the
      `telemetry-parser` crate.

## 5. Phase 2 quality polish

- [ ] Higher-order interpolation in `undistort.cpp` (bicubic / Lanczos4 — currently bilinear).
      Mainly an output-quality feature (Gyroflow defaults to Lanczos4); the `COEFFS` tables in
      `cpu_undistort.rs` port directly. Note: in the same-interpolation PSNR test this was only
      a ~0.6 dB factor, so it is **not** the path to closing the C++↔Gyroflow gap (see item 2).
      Add a `--interpolation bilinear|bicubic|lanczos4` CLI flag.
- [ ] Horizon lock (`src/core/smoothing/horizon.rs`).
- [ ] Color-range (full/limited) handling; per-axis smoothing; `lens_correction_amount < 1`.

> Tooling now in place for regression-checking the above: `tools/compare_gyroflow_metadata.py`
> (math vs golden) and `tools/stabilization_quality.py` (ITF + residual motion, before/after).
> Re-run both after any kernel/smoothing change.

## 6. Phase 5 — breadth

- [ ] More distortion models (opencv_standard, poly3/5, etc.).
- [ ] Raw-IMU integration (complementary / VQF) for non-fused sources.
- [ ] Other cameras; optional GPU (OpenCL/wgpu) backend.

---

### Quick resume commands
```sh
cmake --build cpp_core/build -j && (cd cpp_core/build && ctest --output-on-failure)
python3 tools/export_bridge_json.py --project data/DJI_20260605174353_0032_D.gyroflow \
    --camera-csv data/dji_camera_data.csv -o data/dji_bridge.json
./cpp_core/build/gyroflow_cpp_stabilize data/DJI_20260605174353_0032_D.MP4 \
    --telemetry data/dji_bridge.json -o /tmp/out.mp4 --max-frames 120   # default: 16:9 crop
```
Validate the math against golden Gyroflow metadata (encoder-independent):
```sh
gyroflow data/DJI_20260605174353_0032_D.gyroflow --export-metadata "3:/tmp/gf_meta.json"
./cpp_core/build/gyroflow_cpp_validate data/dji_bridge.json --frames 973 > /tmp/cpp.csv
python3 tools/compare_gyroflow_metadata.py /tmp/gf_meta.json /tmp/cpp.csv   # expect PASS
```
Measure stabilization quality (how much shake was removed, before vs after):
```sh
python3 tools/stabilization_quality.py --compare \
    data/DJI_20260605174353_0032_D.MP4 /tmp/out.mp4 --max-frames 300
```
Validation metrics (sample clip): global camera shake (phase-correlation) **−89%**
(10.5→1.2 px), optical-flow motion −40% (residual ≈7.6 px is the walking subject, not
shake), ITF +1.56 dB; worst-frame black-border fraction ≈0.16% (16:9 crop) / ≈0.07% (full
sensor) with adaptive zoom. The output is now 3840×2160 16:9 by default (use `--keep-sensor`
for the 4:3 sensor). `data/` is gitignored.
