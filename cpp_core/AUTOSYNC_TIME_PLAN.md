# C++ Autosync-Time Port — Development Plan

UI-free C++ port of Gyroflow's **gyro↔video time-offset finder** ("autosync time"),
usable both as a sync engine and as a **timestamp-synchronization accuracy evaluator**.
Lives in `cpp_core/`, independent of the Rust build, validated against the Rust algorithm
and against the DJI data in `../data/`.

## 1. Goal & scope

Gyroflow's auto-sync solves: *given camera motion observed two ways (from the video, and
from the IMU), what constant time offset between the two clocks best aligns them?* The
answer is the **timestamp delay** (ms) between the gyro stream and the video stream.

This port reproduces the **time-offset search** only — the part that, given two angular-velocity
time series, finds the offset that minimises their difference. It is the `essential_matrix`
offset method (`offset_method = 0`), which is the default and the most directly testable.

**Out of scope** (kept in Rust / future work): video decoding, OpenCV optical flow, pose
estimation (essential-matrix / homography / Almeida), and `rs_sync`/`visual_features` offset
methods. We do **not** read pixels — instead we obtain the two motion signals from telemetry,
as the task allows ("如果需要 gyro 的旋转数据，考虑用 dji 的四元数生成一份来用").

### Why this is enough to be useful
The offset finder is the numerically interesting, parity-critical core. Everything upstream
(optical flow → per-frame rotation → per-frame angular velocity `estimated_gyro`) just
produces *one* of the two angular-velocity signals. We synthesise that signal from the DJI
fused quaternions instead of decoding video, which lets us:
- run the exact same offset search the Rust app runs, and
- **inject a known offset and measure how accurately it is recovered** — a direct, ground-truthed
  measure of timestamp-sync precision that the real pipeline cannot give us (it has no ground truth).

## 2. Source algorithm (Rust references)

- `src/core/synchronization/find_offset/essential_matrix.rs` — `find_offsets` / `calculate_cost`.
  Core search: per sync-range, 20 Hz zero-phase low-pass both signals, sweep offset over
  `[initial_offset - search_size, initial_offset + search_size]` at 1 ms, then refine ±2 ms at
  0.01 ms; cost = mean weighted squared angular-velocity difference (x,y ×70, z ×100), looking
  up the gyro sample at `of_ts - offset` (nearest upper neighbour). Accept if within 90% of
  search size and ≥ half the OF samples matched; reject ranges whose max angle < 3.
- `src/core/filtering.rs` — `Lowpass` = RBJ Butterworth biquad (`Q = 1/√2`),
  `DirectForm2Transposed`, applied forward then backward (zero phase). Ported exactly.
- `src/core/synchronization/mod.rs` — `recalculate_gyro_data` shows how per-frame rotation
  becomes `estimated_gyro` (deg/s, **x/y swapped**, timestamp placed *between* frames). We
  reproduce the units/axis convention when generating the video-side signal.

## 3. Data inputs (`../data/`, from `DJI_20260605174353_0032_D.MP4`)

| file | content | use |
|------|---------|-----|
| `dji_quaternions_full.csv` | `imu_index,frame,frame_timestamp_ms,quat_timestamp_ms,quat_{w,x,y,z}` (~32 465 rows, ~989 Hz) | primary: fused attitude → angular velocity |
| `dji_camera_data.csv` | `frame,timestamp_ms,org_{pitch,yaw,roll},org_quat_{w,x,y,z}` | alt. quaternion source (Gyroflow camera-data format) |
| `dji_quat_analysis.csv` | adds `quat_omega_{x,y,z}_rad_s` (already-differentiated) | cross-check our ω derivation |

Stream is ~989 Hz over 32.45 s; video is 973 frames ≈ 30 fps. So the full-rate ω mimics the
**IMU gyro**, and ω subsampled to ~30 fps mimics the **video `estimated_gyro`**.

## 4. Components to build

```
cpp_core/
  include/gyroflow/timesync.hpp   # new: TimeImu, Lowpass, ω-from-quaternions, findOffset
  src/timesync.cpp                # new: implementations (ports of filtering.rs + essential_matrix.rs)
  tools/gyroflow_autosync.cpp     # new: CLI
  tests/test_timesync.cpp         # new: lowpass + offset-recovery unit tests
  CMakeLists.txt                  # +timesync.cpp, +CLI, +test
```

### 4.1 Core (`timesync.hpp/.cpp`)
- `struct TimeImu { double timestamp_ms; bool has_gyro; double gyro[3]; }` (deg/s).
- `class Lowpass` — biquad coeffs + `DirectForm2Transposed`; `filterGyroForwardBackward(freq, sample_rate, data)`. Exact port.
- `quaternionsToAngularVelocity(samples, swap_xy, degrees)` — for consecutive `(t0,q0),(t1,q1)`:
  `dq = q0⁻¹·q1`; rotation vector = axis·angle of `dq`; `ω = rotvec/Δt`. Body frame, matches
  Gyroflow's per-frame derivation; optional x/y swap + rad→deg to mirror `estimated_gyro`.
- `resampleAngularVelocity(series, target_timestamps)` — linear interpolation, to build the
  video-rate signal from full-rate ω.
- `findOffset(of, gyro, initial_offset, search_size, of_rate, gyro_rate)` → `{offset_ms, cost, matched}`.
  Faithful port of `find_offsets`’ single-range search (1 ms sweep → 0.01 ms refine, weighted
  cost, nearest-upper gyro lookup, 90% acceptance, max-angle gate).

### 4.2 CLI (`gyroflow_autosync`)
Modes:
- `selftest` — generate ω from quaternions; build a video-rate copy shifted by a **known
  injected offset** (+ optional Gaussian noise); recover it; print recovered vs injected +
  error. Sweeps a set of injected offsets → reports mean / RMS / max error = **precision**.
- `compare` — find the offset between two telemetry-derived signals (e.g. full-rate vs
  video-rate, true offset 0) → baseline bias check.
- `omega` — dump ω CSV (for cross-checking against `dji_quat_analysis.csv`).

Output is the **timestamp delay in ms** (+ cost). Flags: `--quat <csv>`, `--fps`, `--search`,
`--initial`, `--inject`, `--noise`, `--lpf` (default 20), `--swap-xy`.

### 4.3 Tests
- Lowpass: DC gain ≈ 1, attenuates a tone above cutoff, zero-phase symmetry.
- Offset recovery: synthetic chirp, inject known offset, assert recovered within 0.5 ms.

## 5. Evaluation plan (`../data/`)
1. Build, `ctest`.
2. `omega` vs `dji_quat_analysis.csv` — confirm ω derivation matches (within filtering diffs).
3. `selftest` sweeping injected offsets (e.g. −30..+30 ms) at 30 fps with realistic noise —
   report recovery error distribution → timestamp-sync precision of the algorithm.
4. `compare` full-rate vs 30 fps (true offset 0) — confirm near-zero recovered offset (bias floor).

## 6. Parity / correctness checks
- Lowpass coefficients & state recurrence identical to `biquad` `DirectForm2Transposed`.
- Cost function (weights, mean-over-matches, ≥half-matched rule, MAX otherwise) identical.
- Search grid (`search_size*2` steps @1 ms, then 200 steps @0.01 ms over ±2 ms) identical.
- Acceptance window (`|off-initial| < search_size*0.9`) and max-angle<3 gate identical.

## 7. Milestones
1. ✅ Plan (this doc).
2. Core lib + ω generation + Lowpass.
3. findOffset port.
4. CLI + tests.
5. Build, ctest, run on `../data/`, report.
</content>
</invoke>
