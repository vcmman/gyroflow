# Gyroflow C++ Core — Development Plan

Goal of this document: a grounded, phased plan to bring the C++ port (`cpp_core/`) from
its current scaffold to a **headless, UI-free C++ tool that takes a DJI MP4 + its
telemetry and writes a stabilized MP4**, matching Rust Gyroflow's output.

It is written against the real Rust pipeline (`src/core/`), with concrete file/function
references so the math can be ported faithfully and validated against golden data.

---

## 1. Review of the current `cpp_core/` scaffold

What exists today (493 LOC total):

| File | Status |
|------|--------|
| `types.hpp` / `quaternion.cpp` | Quaternion math: `*`, `dot`, `inverse`, `normalized`, `fromAxisAngle`, `slerp`, time-series `sampleQuaternion` (binary-search + slerp). **Solid, keep.** |
| `smoothing.cpp` | `smoothQuaternionsPlain` — a forward/backward exponential slerp filter. Reasonable scaffold but **not** Gyroflow's default algorithm. |
| `stabilization.cpp` | `computeStabilizationSamples` / `computeRollingShutterRows`. Computes `compensation = smooth.inverse() * raw` and per-row readout timestamps. |
| `lens_profile.cpp` | `LensProfile`/`CameraMatrix` structs + a guessed default (`fx = width*0.8`). Data only. |
| `telemetry.hpp` | `FileMetadata` / `TimeIMU` structs. **No parser** behind them. |
| `tests/`, `tools/gyroflow_cpp_probe.cpp` | Smoke tests + a probe binary. |

### Correctness gaps to be aware of (the scaffold is a placeholder, not a port)

1. **The stabilization model is over-simplified.** The scaffold's `compensation =
   smooth.inverse() * raw` is *not* what Gyroflow applies. The real per-row transform
   (`src/core/stabilization/frame_transform.rs:235`) is:
   ```
   quat   = smoothed_quat(ts) * org_quat(ts).inverse() * org_quat(row_ts)
   R      = image_rotation * quat.to_rotation_matrix()   (with axis sign flips)
   i_r    = (new_K * R).pseudo_inverse()                 // 3×3, this is what the kernel uses
   ```
   and that `i_r` matrix — *not* a bare quaternion — is what the pixel kernel consumes.
2. **No undistortion / pixel remap exists.** This is the heart of stabilization and is
   entirely missing. See §3.
3. **No real lens model.** `makeDefaultLensProfile` guesses intrinsics; the real path
   needs the actual camera matrix + distortion coeffs from the lens profile, and the
   OpenCV-fisheye distort/undistort math (`distortion_models/opencv_fisheye.rs`).
4. **No telemetry parser and no video I/O.** Both are needed end-to-end.
5. **No adaptive zoom / FOV (`new_K`).** Output currently has no defined framing.

**Conclusion:** keep the quaternion math; treat smoothing/stabilization/lens as
scaffolding to be replaced with faithful ports validated against golden CSV/STMap data.

---

## 2. The real DJI stabilization pipeline (what we are porting)

Data flow, with source references:

```
DJI MP4
  └─ telemetry-parser (Rust git dep)  → fused quaternions (GroupId::Quaternion, w,x,y,z),
       src/core/gyro_source/mod.rs:174   frame_readout_time (+sign = direction),
                                         detected camera model, lens hints
  └─ lens profile DB (profiles.cbor.gz) → camera_matrix (fx,fy,cx,cy),
       src/core/lens_profile_database.rs   distortion_coeffs k[0..4], calib/output dims,
                                           distortion_model = "opencv_fisheye"
        │
        ▼
  smoothing  (src/core/smoothing/default_algo.rs, id=1)
        │  velocity-adaptive forward/backward slerp → smoothed quaternion series
        ▼
  per-frame, per-row transform  (frame_transform.rs)
        │  build new_K (FOV/zoom) and 3×3 i_r matrix per scanline
        ▼
  per-pixel undistort  (cpu_undistort.rs: rotate_and_distort + bilinear remap)
        │  out(x,y) → i_r·(x,y,1) → (x',y',w') → fisheye distort → ·f → +c → src(u,v)
        ▼
  ffmpeg decode → remap → encode  (src/rendering/)
```

### Key formulas to port (quote-accurate)

**Rolling shutter row timing** (`frame_transform.rs:212`):
```
row_readout = frame_readout_time / (horizontal? width : height)
start_ts    = frame_center_ts - frame_readout_time/2
row_ts(y)   = start_ts + row_readout * y
```

**Per-row rotation matrix** (`frame_transform.rs:235`, axis flips at :238):
```
quat = smoothed_quat(ts) * org_quat(ts).inverse() * org_quat(row_ts)
r    = image_rotation * R(quat)
# non-inverted framebuffer sign flips:
r[0][1]*=-1; r[0][2]*=-1; r[1][0]*=-1; r[2][0]*=-1
i_r  = pseudo_inverse(new_K * r)          # store 9 floats (+ IBIS/OIS: sx,sy,ra,ox,oy)
```

**Per-pixel remap** (`cpu_undistort.rs:135 rotate_and_distort`):
```
_x = x*i_r[0] + y*i_r[1] + i_r[2]
_y = x*i_r[3] + y*i_r[4] + i_r[5]
_w = x*i_r[6] + y*i_r[7] + i_r[8]
if _w <= 0: pixel is invalid (background)
(u,v) = fisheye.distort_point(_x,_y,_w)   # iterative model below
(u,v) = (u*f[0] + c[0],  v*f[1] + c[1])   # back to source pixels
sample source frame at (u,v) with bilinear interpolation
```

**OpenCV-fisheye distort** (`distortion_models/opencv_fisheye.rs:72`):
```
x=_x/_w; y=_y/_w; r=hypot(x,y); theta=atan(r)
theta_d = theta*(1 + k0·θ² + k1·θ⁴ + k2·θ⁶ + k3·θ⁸)
scale = r==0 ? 1 : theta_d/r
return (x*scale, y*scale)
```
(`undistort_point` is the 10-iteration Newton inverse — needed only for adaptive zoom /
STMap, not for the forward remap.)

**Quaternion convention:** scalar-first `(w,x,y,z)`, unit quaternions; timestamps stored
in microseconds (`TimeQuat = BTreeMap<i64,Quat>`). DJI provides fused attitude directly,
so **no IMU integration is required for the DJI path** — a major simplification.

---

## 3. Phase 1 — Headless DJI stabilizer (the target)

**Definition of done:** `gyroflow_cpp_stabilize input.MP4 telemetry.json [profile.json] -o out.MP4`
decodes the DJI clip, applies smoothed quaternion + rolling-shutter undistortion per
frame, and writes a stabilized MP4 whose frames match Rust Gyroflow within a small
tolerance on a reference DJI clip.

### Strategic decisions (confirmed)

- **Telemetry: JSON/CSV bridge.** Reuse the Rust Gyroflow CLI / existing `tools/` Python
  scripts to export, for the target clip: the org (raw) quaternion series, frame readout
  time + direction, fps, dimensions, and the matched lens profile (camera matrix +
  distortion coeffs). C++ reads these. This **decouples the algorithm port from the DJI
  protobuf parser** (deferred to Phase 3). The `tools/dji_export_quaternions_full.py`
  path already produces org_quat CSV — extend it to also emit readout time + lens JSON.
- **Video I/O: OpenCV `VideoCapture`/`VideoWriter`.** Simplest decode→remap→encode loop;
  `cpp_core` already centers on the OpenCV-fisheye model and OpenCV is already a build
  dependency. Accepted limitations for Phase 1: transcode (some quality loss), no audio
  passthrough, BGR 8-bit only. Native libav with audio is Phase 4.

### Work breakdown

1. **Telemetry ingestion (`telemetry_io`)**
   - Define the JSON schema (org quaternions `[t_us, w,x,y,z]`, `frame_readout_time_ms`,
     `frame_readout_direction`, `fps`, `width`, `height`, lens profile block).
   - C++ JSON loader (header-only nlohmann/json) → fills `FileMetadata` + `LensProfile`.
   - Extend the Python exporter to emit this schema; add a golden fixture for one clip.
2. **Lens model (`distortion/opencv_fisheye`)**
   - Port `distort_point` (forward) and the 10-iteration `undistort_point` (inverse).
   - Replace `makeDefaultLensProfile` with real intrinsics loaded from the profile JSON;
     handle calib-dimension → output-dimension scaling of `fx,fy,cx,cy`.
3. **Smoothing — faithful default algorithm (`smoothing/default_algo`)**
   - Port `DefaultAlgo` (`src/core/smoothing/default_algo.rs`): velocity computation,
     velocity-adaptive alpha `α = 1 − exp(−(1/rate)/τ)`, forward+backward passes,
     distance-modulated second pass. Defaults: `smoothness=0.5`, `max_smoothness=1.0`,
     `alpha_0_1s=0.1`, `per_axis=false`, `second_pass=true`.
   - Keep the existing plain smoother as a fallback / cross-check.
4. **Transform builder (`stabilization/frame_transform`)** — replace the scaffold:
   - Implement `new_K` from FOV (Phase 1: `fov` fixed, no adaptive zoom; output = input
     dims). Port `get_new_k`.
   - Per frame: compute `quat1`, `smoothed_quat1`; per row build `i_r = inverse(new_K·r)`
     with the axis sign flips. Output `std::vector<Mat3>` (one per scanline).
   - 3×3 inverse: small dependency (Eigen) or a hand-written adjugate inverse.
5. **CPU undistort kernel (`stabilization/cpu_undistort`)**
   - Port `rotate_and_distort` + bilinear sampling. Mark `_w<=0` and out-of-bounds as
     background color. Parallelize rows (std::thread / OpenMP / TBB; rayon → OpenMP).
   - Skip for Phase 1: digital lens, mesh correction, IBIS/OIS, light refraction,
     focal-plane distortion, color-range remap. (DJI fused path doesn't need them.)
6. **Video driver (`tools/gyroflow_cpp_stabilize`)**
   - argh-style CLI; OpenCV `VideoCapture` → per-frame timestamp = `frame/fps` →
     build transform → remap → `VideoWriter`. Map BGR frame to the kernel's pixel type.
7. **Validation harness**
   - Export Rust Gyroflow STMaps / a few stabilized PNG frames for a reference DJI clip;
     compare C++ remap coordinates and output pixels (PSNR / max-pixel-error threshold).
   - Unit tests: fisheye distort/undistort round-trip; matrix build vs a captured golden
     `i_r`; smoothing vs exported smoothed-quaternion CSV.

### Phase 1 explicitly out of scope
Adaptive zoom / dynamic cropping, audio, GPU, lens models other than fisheye, gyro→video
sync, raw-IMU integration, non-DJI cameras, 10-bit/HDR, keyframes.

---

## 4. Later phases

- **Phase 2 — Parity & quality:** ✅ **adaptive zoom implemented**
  (`zooming/adaptive_zoom.{hpp,cpp}`): forward `undistortPoints` (inverse fisheye +
  `new_K·R` reproject, with a `W>0` behind-camera guard), per-frame inscribed-rect FOV
  search, EnvelopeFollower temporal smoothing (method 1, the DJI default), and `max_zoom`
  clamp. On `data/DJI_..._0032_D.MP4` black borders dropped from 7.4% worst (Phase 1
  static fov) to **0.07% worst** at the default 130% max-zoom. Remaining Phase 2 items:
  match Gyroflow's cropped `output_dimension` (it outputs 3840×2160 16:9 from the 3840×2880
  4:3 sensor; the C++ currently renders the full 4:3 sensor — see note below), horizon
  lock, bicubic/lanczos interpolation, color-range handling. Goal: visually
  indistinguishable from Rust Gyroflow on DJI clips.

  > **Output-dimension note / bug fixed:** the bridge lens profile carries Gyroflow's
  > 16:9 `output_dimension` (3840×2160). Letting that drive `new_K`'s principal point while
  > the kernel rendered the 2880-tall sensor created three inconsistent coordinate systems
  > and left a large bottom border. Phase 2 forces output == input (full sensor) for
  > self-consistency; honoring the 16:9 crop requires threading distinct input vs output
  > dims through `computeFrameTransform` + `computeAdaptiveFovs` (output center & aspect)
  > and the rolling-shutter row indexing.
- **Phase 3 — Native telemetry parser:** port DJI `djmd`/DVTM protobuf parsing (replaces
  the JSON bridge), readout-time extraction, lens-profile DB (CBOR+gzip) loading and
  matching. Removes the Rust/Python dependency.
- **Phase 4 — Native FFmpeg I/O:** libav decode/encode, audio passthrough, bit-depth and
  codec/bitrate control, the PreConversion/PostConversion ordering from `rendering/`.
- **Phase 5 — More sources & GPU:** additional distortion models, raw-IMU integration
  (complementary/VQF), other cameras, optional GPU (OpenCL/wgpu) backend.

---

## 5. Suggested module/build layout for Phase 1

```
cpp_core/
  include/gyroflow/
    distortion/opencv_fisheye.hpp
    smoothing/default_algo.hpp
    stabilization/frame_transform.hpp   # Mat3, KernelParams, build per-row i_r
    stabilization/undistort.hpp         # rotate_and_distort + remap
    telemetry_io.hpp                    # JSON → FileMetadata/LensProfile
  src/  (matching .cpp)
  tools/gyroflow_cpp_stabilize.cpp      # OpenCV decode→remap→encode CLI
  tests/  (fisheye round-trip, transform vs golden, smoothing vs CSV)
```
CMake: add OpenCV (`find_package(OpenCV)`) and a JSON lib for the new tool/targets while
keeping `gyroflow_cpp_core` dependency-light. A 3×3 inverse needs either Eigen or a small
hand-rolled routine.

### First implementation order (smallest steps to a moving picture)
1. ✅ fisheye `distort_point` + `undistort_point` + round-trip unit test
   (`distortion/opencv_fisheye.{hpp,cpp}`, `tests/test_fisheye.cpp`).
2. ✅ JSON telemetry loader → `FileMetadata`/`LensProfile`
   (`telemetry_io.{hpp,cpp}`, `tests/test_telemetry_io.cpp`). Schema documented in the
   header; nlohmann/json vendored under `third_party/`. **TODO:** extend the Python
   exporter in `tools/` to emit this schema + a real DJI fixture.
3. ✅ `frame_transform` (fov fixed) → per-row `i_r` (`stabilization/frame_transform.{hpp,cpp}`,
   `mat3.hpp`); identity-map integration test in `tests/test_transform.cpp`.
4. ✅ `undistort` kernel + OpenCV driver → first stabilized MP4, rolling shutter on
   (`stabilization/undistort.{hpp,cpp}`, `tools/gyroflow_cpp_stabilize.cpp`,
   `tools/export_bridge_json.py`).
5. ✅ `DefaultAlgo` smoothing port (`smoothing/default_algo.{hpp,cpp}`); params verified to
   match the project export (smoothness 0.5, max_smoothness 1.0, alpha_0_1s 0.1, per_axis 0).
6. ⬜ PSNR comparison vs Rust Gyroflow frames; tune until within tolerance. (Blocked on
   matching framing: Gyroflow applies adaptive zoom (window 4.0) by default — see Phase 2.)

**End-to-end status:** runs on `data/DJI_20260605174353_0032_D.MP4` (3840×2880, OsmoAction4,
21.82 ms readout) producing a stabilized MP4. Confirmed faithful: no sync offset
(`offsets: {}`), `integration_method 0` with no axis remap, frame timestamp = `frame*1000/fps`
matches `timestamp_at_frame`. Center-crop inter-frame motion reduced ~21% (subject is walking,
so scene motion remains). Remaining gap to Gyroflow = adaptive zoom (Phase 2).
