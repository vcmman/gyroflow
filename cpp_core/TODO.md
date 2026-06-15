# cpp_core — Next steps (resume here)

Status as of last session: Phase 1 (headless DJI stabilizer) and Phase 2 (adaptive zoom)
work end-to-end on `data/DJI_..._0032_D.MP4`; output is encoded via ffmpeg (H.264/H.265 +
audio). 5/5 unit tests pass. See `README.md` and `DEVELOPMENT_PLAN.md` for the full picture.

Pick up from the top. Each item lists the concrete files to touch.

---

## 1. Finish Phase 2 — match Gyroflow framing + PSNR parity  ← do this first

The C++ currently renders the **full 4:3 sensor** (output == input). Gyroflow crops to its
lens `output_dimension` (this clip: **3840×2160, 16:9**). Until that matches we can't do a
pixel-level PSNR comparison.

- [ ] Thread distinct **input vs output** dimensions through the pipeline (today they're
      forced equal in `tools/gyroflow_cpp_stabilize.cpp` — `lens.output_* = width/height`):
  - `stabilization/frame_transform.cpp` — `new_K` principal point = `output_w/2, output_h/2`;
    source camera matrix `c` stays at input `cx,cy`. Decide rolling-shutter row indexing
    (kernel indexes output rows; matrix count currently = source rows).
  - `zooming/adaptive_zoom.cpp` — `computeAdaptiveFovs` must use **output** dims for `new_K`
    center, inscribed-rect aspect, and the fov denominator, while the source border ring and
    `c` use **input** dims. (Mirror Gyroflow `FovIterative::new`: ring in input dims,
    `output_inv_aspect` from output dims.)
  - `stabilization/undistort.cpp` / the CLI — output buffer + VideoWriter size = output dims.
- [ ] Verify the fov-scaling identity still holds (`out = center + (out_fov1-center)/fov`)
      when output center ≠ input center; add a round-trip unit test for output≠input.
- [ ] Add a `--output-size WxH` (or `--keep-sensor`) flag so both modes are testable.

## 2. PSNR / golden comparison vs Rust Gyroflow

- [ ] Render the reference with the Gyroflow binary. **Known issue:** the default render
      failed with `Pixel format YUV420P10LE is not supported` (the clip is 10-bit). Pass
      `-p '{...}'` out-params to force a supported pixfmt/codec, or transcode the source to
      8-bit first.
- [ ] Frame-by-frame PSNR / max-pixel-error between C++ output and Gyroflow output; also
      compare exported STMaps (`--export-stmap`) against the C++ remap coords.
- [ ] Cross-check smoothed quaternions: export Gyroflow stabilized quaternions and diff
      against `smoothDefault` output (validates the smoothing port independently of render).

## 3. Phase 4 — native libav decode (input)

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
- [ ] Horizon lock (`src/core/smoothing/horizon.rs`).
- [ ] Color-range (full/limited) handling; per-axis smoothing; `lens_correction_amount < 1`.

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
    --telemetry data/dji_bridge.json -o /tmp/out.mp4 --max-frames 120
```
Validation metric used last time: center-crop inter-frame motion (≈21% reduction) and
worst-frame black-border fraction (0.07% with adaptive zoom). `data/` is gitignored.
