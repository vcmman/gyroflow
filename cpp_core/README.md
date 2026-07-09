# Gyroflow C++ Core Port

UI-free C++ port of Gyroflow's stabilization core. The target shape is a small
dependency-light library plus CLI tools, validated against Rust Gyroflow output. See
[`DEVELOPMENT_PLAN.md`](DEVELOPMENT_PLAN.md) for the full phased plan and the porting
references into `../src/core/`, [`PIPELINE.md`](PIPELINE.md) for the end-to-end algorithm
flow + exact parameters behind the current result video, and [`COMPARISON.md`](COMPARISON.md)
for the quantitative head-to-head vs Rust Gyroflow under identical parameters.

## Quick start (set up a machine and stabilize a DJI clip)

End-to-end on a fresh Linux box, from zero to a stabilized MP4. Tested on Ubuntu 20.04
(cmake 3.16, g++ 9, OpenCV 4.2).

**1. Install build + runtime dependencies**

```sh
# Debian/Ubuntu. The core lib needs only a C++17 compiler + cmake; the CLI also needs
# OpenCV (video decode/encode fallback) and ffmpeg (preferred encoder, on PATH).
sudo apt-get update
sudo apt-get install -y build-essential cmake git libopencv-dev ffmpeg python3 python3-numpy
```

No sudo? Install a static ffmpeg into a PATH dir instead (see "Encoding" below), and use a
conda/vcpkg OpenCV. macOS: `brew install cmake opencv ffmpeg`.

**2. Get a Gyroflow binary** (only needed to export the telemetry bridge — see step 4; the
native DJI parser is a later phase). Download the official AppImage/app from
<https://gyroflow.xyz> and put it on `PATH` as `gyroflow` (or pass `--gyroflow-bin`):

```sh
# Linux AppImage example:
chmod +x Gyroflow-*.AppImage && sudo mv Gyroflow-*.AppImage /usr/local/bin/gyroflow
gyroflow --version    # sanity check
```

**3. Build the C++ stabilizer**

```sh
git clone <this-repo> && cd <repo>           # or use your existing checkout
cmake -S cpp_core -B cpp_core/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp_core/build -j
(cd cpp_core/build && ctest --output-on-failure)    # optional: 7/7 should pass
```

This produces `cpp_core/build/gyroflow_cpp_stabilize` (needs OpenCV; the build prints
`gyroflow_cpp_stabilize: enabled` when found).

**4. Export the telemetry bridge JSON for your clip** (reuses Gyroflow to get the matched
lens profile + readout time + attitude quaternions; the C++ doesn't parse DJI protobuf yet):

```sh
python3 tools/export_bridge_json.py MY_DJI_CLIP.MP4 -o my_bridge.json
# already have a .gyroflow project / camera CSV? pass --project / --camera-csv to skip re-export
```

**5. Stabilize** (adaptive zoom + 16:9 output + ffmpeg H.264 + audio, all on by default):

```sh
./cpp_core/build/gyroflow_cpp_stabilize MY_DJI_CLIP.MP4 --telemetry my_bridge.json -o out.mp4
# quick preview of the first 120 frames:  add  --max-frames 120
```

`out.mp4` is the stabilized result. See the flag tables below to change zoom, framing, codec,
etc., and [`PIPELINE.md`](PIPELINE.md) for what each stage does.

> Caveat: input is currently decoded 8-bit via OpenCV, so 10-bit/HDR DJI footage is truncated
> (native libav decode is a later phase). Stabilization quality is unaffected; only bit depth.

## Status

Phase 1 (headless DJI stabilizer) and Phase 2 (adaptive zoom + Gyroflow `output_dimension`
framing) are implemented and run end-to-end on a real DJI clip, producing a stabilized MP4
with dynamic cropping at Gyroflow's 16:9 output size (by default).

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
  inscribed-rectangle FOV search, both temporal-smoothing methods (EnvelopeFollower default +
  GaussianFilter, via `--zoom-method`; both validated against golden `fov_scale`), and
  `max_zoom` clamp.
- **Telemetry bridge** (`telemetry_io.*`, `../tools/export_bridge_json.py`) — Phase 1 reads
  a JSON sidecar exported from the Rust Gyroflow CLI instead of parsing DJI protobuf in C++.

Not yet done: native DJI metadata parser, native libav **decode** (input is still OpenCV
`VideoCapture`, 8-bit BGR), horizon lock, bicubic/lanczos interpolation, additional lens
models, GPU. (Output framing now matches Gyroflow's cropped `output_dimension`.)

## Build & test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
(cd build && ctest --output-on-failure)   # ctest 3.16 has no --test-dir; run from build/
```

The `gyroflow_cpp_stabilize` CLI is built when OpenCV is found; the core library, unit
tests, and the `gyroflow_cpp_validate` golden-comparison dumper have no external
dependencies (JSON is vendored under `third_party/`).

## Run the headless stabilizer

```sh
# 1. Export the telemetry bridge JSON (needs a Gyroflow binary on PATH, or pass a
#    pre-exported .gyroflow project + camera-data CSV):
python3 ../tools/export_bridge_json.py INPUT.MP4 -o bridge.json

# 2. Stabilize (adaptive zoom + ffmpeg H.264 + audio, all on by default):
./build/gyroflow_cpp_stabilize INPUT.MP4 --telemetry bridge.json -o out.mp4
```

Stabilization options:

| Flag | Default | Meaning |
|------|---------|---------|
| `--enhanced` | off | **Recommended preset** = DCR on (−31…35 % vertical shake, black border unchanged; `EVALUATION_SUMMARY.md`). |
| `--dcr [--dcr-window 0.5] [--dcr-power 1.0]` | off | Direction-consistency gate: keep full smoothing on reciprocating shake, follow intentional pans (`SMOOTHING_RND.md` §1). |
| `--per-axis --smoothness-pitch/-yaw/-roll <0..1>` | off | Per-euler-axis smoothing (evaluated §8d/§8e — not in the preset). |
| `--look-ahead <s>` | 0 (offline) | In-camera finite look-ahead for the smoothing backward pass (§7). |
| `--smoothness <0..1>` | 0.5 | Master smoothing slider (Gyroflow's); lower = follow more. |
| `--deviation-clamp <deg>` | 0 (off) | Bound the smoothed path to ≤ B° from raw — DJI-like bounded-deviation mode, hard-bounded crop demand (§8j). |
| `--deviation-clamp-soft <deg> [--deviation-clamp-ref-tau 0.02]` | 0 (off) | Soft (burr-free) variant: smooth box center + tanh saturation — DJI's amplitude with 37 % less roughness than DJI (§8j-4). |
| `--zoom-method envelope\|gaussian` | envelope | Dynamic-zoom temporal smoother (both golden-validated). |
| `--zoom-look-ahead <s>` | −1 (offline) | Real-time dynamic-zoom envelope with this much future; fixes causal zoom pops (§8h). |
| `--max-zoom <pct>` | 130 | Dynamic-zoom ceiling (percent). |
| `--no-adaptive-zoom` | off | Disable dynamic crop (renders with `--fov`). |
| `--fov <f>` | 1.0 | Static zoom (`<1` zooms in); disables adaptive zoom. |
| `--keep-sensor` | off | Render the full input sensor (e.g. 4:3) instead of the lens crop. |
| `--output-size <WxH>` | lens | Override the output framing (default: lens `output_dimension`). |
| `--max-frames <n>` | all | Process only the first N frames (quick preview). |
| `--threads <n>` | auto | CPU threads for the undistort kernel. |

By default the rendered size matches Gyroflow's lens `output_dimension` (e.g. **3840×2160
16:9** cropped from a 3840×2880 4:3 sensor). `--keep-sensor` renders the full sensor and
`--output-size WxH` sets an explicit size.

## Encoding (ffmpeg)

Output is encoded by piping raw BGR frames to **ffmpeg** when an ffmpeg binary is found
(proper H.264/H.265 compression + audio passthrough). Without ffmpeg it falls back to
OpenCV's `mp4v` writer (much larger, no audio).

```sh
ffmpeg -version            # needed on PATH; install below if missing
```

Install ffmpeg (no sudo required — static build into a PATH dir):

```sh
curl -L https://johnvansickle.com/ffmpeg/releases/ffmpeg-release-amd64-static.tar.xz \
  | tar xJ -C /tmp
cp /tmp/ffmpeg-*-static/ffmpeg /tmp/ffmpeg-*-static/ffprobe ~/.local/bin/   # ensure ~/.local/bin is on PATH
# or, with a package manager: sudo apt-get install -y ffmpeg   /   conda install -c conda-forge ffmpeg
```

Encoding options:

| Flag | Default | Meaning |
|------|---------|---------|
| `--codec h264\|h265` | h264 | `libx264` or `libx265` (smaller, slower). |
| `--crf <n>` | 18 | Quality/size knob. Lower = bigger/better; 18 ≈ visually lossless. |
| `--no-audio` | off | Don't copy the source audio track. |
| `--no-ffmpeg` | off | Force the OpenCV `mp4v` writer instead of ffmpeg. |
| `--ffmpeg-bin <path>` | `ffmpeg` | Path to the ffmpeg binary. |

Size reference (3840×2880, 120 frames of the sample DJI clip):

| Encoder | Size |
|---------|------|
| OpenCV `mp4v` | 147 MB |
| ffmpeg H.264 CRF 18 (default, visually lossless) | 102 MB |
| ffmpeg H.264 CRF 23 (high quality) | 60 MB |
| ffmpeg H.264 CRF 28 (smaller) | 33 MB |

Example — smaller file, H.265, no audio:

```sh
./build/gyroflow_cpp_stabilize INPUT.MP4 --telemetry bridge.json -o out.mp4 \
  --codec h265 --crf 26 --no-audio
```

## IMU orientation (raw-gyro sources)

The DJI path here uses fused attitude, so no IMU axis config is needed. But if you feed
**raw gyro** into Gyroflow (gcsv / a camera Gyroflow integrates) to produce the bridge, the
UI **"IMU orientation"** must map your sensor axes into Gyroflow's expected input frame,
which — relative to the OpenCV image frame (X=right/width, Y=down/height, Z=forward) — is:

> **X = up, Y = left, Z = back** (`imu_orientation` = `XYZ` means the data is already this).

So a gyro aligned to the OpenCV image frame needs **`imu_orientation = yxz`** (not `Xyz`/`XYZ`):
the raw gyro goes through both `orient()` and a fixed `(-g[1],g[0],g[2])` swap baked into every
integrator, which compose with the render's `diag(1,−1,−1)` flip. Verified in
`tests/test_imu_orientation.cpp`; full derivation + design rationale in
[`PIPELINE.md`](PIPELINE.md) ("Coordinate conventions").

## Validation against Rust Gyroflow

The math is cross-checked against Gyroflow's golden per-frame metadata, independent of any
video encoder (so it isolates the algorithms from rendering noise). `gyroflow_cpp_validate`
dumps the smoothed quaternions + adaptive FOVs; `../tools/compare_gyroflow_metadata.py`
diffs them against `gyroflow ... --export-metadata "3:meta.json"`. Over all 973 frames of
the sample clip:

| Quantity | vs Gyroflow | max | mean |
|----------|-------------|-----|------|
| smoothed orientation (`stab_quat`) | smoothing port | **0.0147°** | 0.0037° |
| adaptive fov (`fov_scale`) | zoom + 16:9 framing | **0.0012%** | 0.0004% |
| org_quat (sampling sanity) | raw interpolation | 0.0144° | 0.0037° |

```sh
gyroflow PROJECT.gyroflow --export-metadata "3:/tmp/gf_meta.json"
./build/gyroflow_cpp_validate bridge.json --frames 973 > /tmp/cpp.csv
python3 ../tools/compare_gyroflow_metadata.py /tmp/gf_meta.json /tmp/cpp.csv   # -> PASS
```

**Frame PSNR.** Rendering both tools from an identical 8-bit source (the 10-bit clip must be
transcoded first — Gyroflow's encoder rejects `YUV420P10LE` here), both on **bilinear**, gives
~33.5 dB mean at the 16:9 output with no global shift and matched brightness. The dual-encoder
noise floor is ~38 dB, so the real geometry residual is ~0.11 px. Matching interpolation only
gains ~0.6 dB vs Gyroflow's default Lanczos4, so the residual is **not** mainly interpolation —
it's dominated by quaternion-sampling FP (Gyroflow rounds the lookup to integer µs; ~0.09 px)
plus minor resampling differences. No systematic framing error; ~0.11 px is near the floor for
two independent codebases through two lossy encoders. See [`COMPARISON.md`](COMPARISON.md) for
the full head-to-head.

## Stabilization quality (how steady is the result)

The checks above answer *"does the C++ match Gyroflow"*. A separate question is *"how much
shake did we actually remove"* — measured on the output itself, not against a reference.
`../tools/stabilization_quality.py` reports intra-video metrics on a centre-cropped,
downscaled grayscale sequence (so the 4:3 input and the 16:9 output are comparable):

- **ITF** — mean PSNR between *consecutive* frames (steadier ⇒ higher).
- **phase-correlation shift** — global translation between consecutive frames (the dominant
  pan/tilt shake; lower ⇒ steadier).
- **optical-flow magnitude** — Farneback flow (captures rotation / complex residual motion).

```sh
python3 ../tools/stabilization_quality.py --compare INPUT.MP4 stabilized.mp4 --max-frames 300
```

On the sample clip (first 300 frames, original → stabilized):

| Metric | original | stabilized | change |
|--------|----------|------------|--------|
| ITF (consecutive-frame PSNR) | 17.57 dB | 19.13 dB | **+1.56 dB** |
| phase-corr shift (camera shake) | 10.47 px | 1.15 px | **−89%** |
| optical-flow magnitude | 12.59 px | 7.59 px | −40% |

The global camera shake is almost entirely removed (−89%); the residual optical flow (~7.6
px) is the walking subject + parallax — real scene motion the stabilizer should *not* remove,
which is why ITF/flow don't go to zero.

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
- **Output framing matches Gyroflow.** Distinct input-vs-output dimensions are threaded
  through the pipeline: `new_K`'s principal point uses the **output** centre and `fov` is
  scaled by `width/output_width` (`get_fov`), while the source intrinsics `f,c` and the
  per-row matrix count stay in **input** dims. Adaptive zoom uses the output aspect for the
  inscribed rectangle but keeps the border ring / `new_K` centre / fov denominator in input
  dims (Gyroflow temporarily sets output=input during the fov calc, `zooming/mod.rs:48`).
  The undistort kernel iterates output pixels and selects the rolling-shutter matrix by the
  **source** row (two-pass), mirroring `stabilize.rs`. Default output is the 16:9
  `output_dimension`; `--keep-sensor` renders the full 4:3 sensor (worst black-border
  ≈0.16% for the crop, ≈0.07% for the full sensor).
