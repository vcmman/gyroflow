# Autosync-Time — Complete Tool Documentation

A standalone, UI-free Python tool that finds the constant **timestamp offset** (in milliseconds)
between a camera's motion and its IMU/gyroscope stream. It is a faithful port of Gyroflow's default
time-offset finder (`essential_matrix::find_offsets` + `filtering::Lowpass`) with an added
higher-precision interpolated mode and a ground-truthed accuracy evaluator.

- **New users:** read [`QUICKSTART.md`](QUICKSTART.md) first, then come back here for depth.
- **This document** is the full reference: background, theory, every mode and flag, input/output
  formats, the Python API, recipes, troubleshooting, precision data, and limitations.
- 中文版见 [`DOCUMENTATION.zh-CN.md`](DOCUMENTATION.zh-CN.md)（内容一致）。

---

## Table of contents

1. [What problem this solves](#1-what-problem-this-solves)
2. [How it works (algorithm)](#2-how-it-works-algorithm)
3. [Installation](#3-installation)
4. [Concepts & conventions](#4-concepts--conventions)
5. [Command-line reference](#5-command-line-reference)
   - [`sync`](#51-sync--align-two-real-signals-production) ·
     [`selftest`](#52-selftest--ground-truthed-accuracy) ·
     [`compare`](#53-compare--bias-floor) ·
     [`omega`](#54-omega--dump-angular-velocity)
   - [Global flags](#55-global-flags)
6. [Input / output formats](#6-input--output-formats)
7. [Precision modes](#7-precision-modes-nearest-vs-interp-vs-parabolic)
8. [Python API reference](#8-python-api-reference)
9. [Recipes / end-to-end workflows](#9-recipes--end-to-end-workflows)
10. [Troubleshooting & FAQ](#10-troubleshooting--faq)
11. [Variable frame timing (VFR)](#11-variable-frame-timing-vfr)
12. [Parity with the Rust/C++ engine](#12-parity-with-the-rustc-engine)
13. [Measured precision](#13-measured-precision)
14. [Limitations & future work](#14-limitations--future-work)
15. [Repository map](#15-repository-map)

---

## 1. What problem this solves

Video stabilization needs the gyro samples and the video frames to share one clock. In practice the
two streams start at slightly different, unknown times — the IMU log and the camera frames are offset
by some constant delay (typically a few to tens of milliseconds). Feed the stabilizer the wrong
offset and the correction lags or leads the real motion, leaving visible jitter.

**Autosync-time recovers that delay.** Given the camera's rotational motion as an angular-velocity
signal and the IMU's angular-velocity signal, it finds the single time shift that makes the two
overlap best. Because both signals measure the *same physical rotation*, the correct shift produces a
sharp minimum in a motion-matching cost.

This tool is the **time-offset half** of Gyroflow's sync. It does **not** decode pixels: you supply
the camera-side motion already as angular velocity (from optical flow / `estimated_gyro`, from a
second gyro, or — for evaluation — synthesized from DJI fused quaternions).

---

## 2. How it works (algorithm)

The pipeline matches the Rust engine step for step:

1. **Angular velocity.** Each signal is an `(N,3)` array of body-frame angular velocity (deg/s) with
   per-sample timestamps (ms). For a quaternion attitude series, ω over each interval is
   `rotvec(q[i]⁻¹·q[i+1]) / Δt` (see `quaternions_to_angular_velocity`); for a raw gyro it is the
   samples themselves.
2. **Zero-phase low-pass.** Both signals are filtered with a 20 Hz forward-backward Butterworth
   biquad (`Q = 1/√2`). Forward-then-backward filtering cancels phase, so the filter does not move
   the cost minimum. A Nyquist guard skips the filter when `2·f_cutoff > sample_rate` (e.g. the video
   side at 24/30 fps is left untouched — exactly what Gyroflow does).
3. **Cost.** For a candidate offset `o`, every video sample at time `t` is compared against the IMU
   value at `t − o`. The cost is the mean weighted squared difference over matched samples, with axis
   weights **x:70, y:70, z:100** (z = yaw is weighted higher). A candidate is valid only if more than
   half the video samples find an IMU match.
4. **Coarse sweep → refine.** Sweep `[initial ± search]` at **1 ms**, then refine **±2 ms** around the
   coarse optimum at **0.01 ms**.
5. **Sub-grid (optional).** `parabolic=True` fits a parabola to the three cost samples around the
   refine minimum and jumps to its analytic vertex, removing the residual 0.01 ms grid step.
6. **IMU lookup (the accuracy knob).**
   - `nearest` (default): snap the query time up to the next IMU sample — bit-for-bit faithful to the
     Rust/C++ port, but with a quantization bias of up to one IMU sample interval.
   - `interp`: linearly interpolate the IMU value at the exact query time — removes that bias.
7. **Acceptance.** The result is accepted only if the optimum lies within 90% of the search window
   (a weak optimum at the edge is rejected as "not found").

---

## 3. Installation

```sh
cd py_autosync
pip install -r requirements.txt        # numpy, scipy
python3 test_autosync_time.py          # 8 tests — all should print OK
```

No build step; pure Python. `scipy.signal.lfilter` (zero initial state) reproduces the Rust biquad
`DirectForm2Transposed` output to ~4e-13.

---

## 4. Concepts & conventions

- **Units.** The cost weights and the 3 deg/s motion gate are tuned for **deg/s**. Every loader
  returns deg/s internally; rad/s inputs are converted.
- **Timestamps.** Milliseconds, re-based so each signal starts at 0. The pipeline is
  *timestamp-based, never frame-index based*, so non-uniform spacing is handled naturally.
- **Offset sign.** `offset_ms` is the delay such that **the IMU is sampled at `video_time − offset_ms`**.
  Equivalently, the IMU stream aligns to video timestamps shifted by `−offset_ms`. This is the same
  convention Gyroflow reports.
- **Axes must be comparable.** The cost is per-axis. If the two streams use different axis
  conventions (permuted/flipped), align them first with `--gyro-orientation` / `--video-orientation`
  (a 3-char map; upper-case keeps an axis, lower-case negates: `xzY` → `(−x, −z, +y)`), or the sync
  may not lock.
- **Motion gate.** Below ~3 deg/s of excitation the minimum is shallow and the offset is unreliable;
  the tool warns.

---

## 5. Command-line reference

```
python3 gyroflow_autosync.py <mode> [flags]
  modes: sync | selftest | compare | omega
```

### 5.1 `sync` — align two real signals (production)

Find the offset between two independently-measured angular-velocity streams.

```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video camera_motion.csv --interp-parabolic
```

Required: `--gyro` (IMU log: GCSV or angular-velocity CSV) and `--video` (camera-motion CSV).
Optional: `--units`, `--gyro-orientation`, `--video-orientation`, `--search`, `--initial`, `--lpf`,
`--interp` / `--interp-parabolic`.

Sample rates are estimated automatically from each signal's median timestamp spacing.

**Output** (stderr = diagnostics, stdout = result):

```
gyro : 1200 samples, ~200.0 Hz, span [0.0,5995.0] ms, max|omega| 73.16 deg/s
video: 360 samples,  ~60.0 Hz, span [0.0,5983.3] ms, max|omega| 70.08 deg/s
offset_ms=8.4940 cost=2453.5929 matched=360/360 mode=interp+parabolic
```

Read it as: lower `cost` and `matched == video samples` ⇒ clean fit. Exit code `0` on success, `1`
if no acceptable offset was found, `2` on a usage error (e.g. missing `--video`).

### 5.2 `selftest` — ground-truthed accuracy

Synthesizes the camera side from a DJI quaternion CSV, injects known offsets, and measures recovery
— this is how you quantify the tool's own precision.

```sh
python3 gyroflow_autosync.py selftest --quat ../data/dji_quaternions_full.csv \
    --fps 30 --search 120 --inject="-30,-10,0,10,30" --noise 1.5
```

Prints a `injected,recovered,error,cost,matched,frames` table plus a mean/RMS/max-error summary.

> **Gotcha:** pass negative offset lists with `=`, e.g. `--inject="-30,..."`, otherwise argparse
> reads the leading `-` as a flag.

### 5.3 `compare` — bias floor

Finds the offset between the full-rate signal and a video-rate resample whose true offset is 0, so
the recovered value *is* the algorithm's bias floor.

```sh
python3 gyroflow_autosync.py compare --quat ../data/dji_quaternions_full.csv --fps 30 --interp
```

### 5.4 `omega` — dump angular velocity

Derives ω from the quaternions and writes it as CSV. The output is itself a valid `--video`/`--gyro`
input, so this doubles as a converter.

```sh
python3 gyroflow_autosync.py omega --quat ../data/dji_quaternions_full.csv --range="1000,9000" > omega.csv
# -> timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s
```

### 5.5 Global flags

| Flag | Default | Applies to | Meaning |
|------|---------|-----------|---------|
| `--quat PATH` | — | selftest/compare/omega | DJI quaternion CSV (full or camera_data format) |
| `--gyro PATH` | — | sync | IMU log (GCSV or angular-velocity CSV) |
| `--video PATH` | — | sync | camera-motion angular-velocity CSV |
| `--units deg\|rad` | `deg` | sync | units when not inferable from column names |
| `--gyro-orientation STR` | none | sync | 3-char axis remap for the IMU signal, e.g. `xzY` |
| `--video-orientation STR` | none | sync | 3-char axis remap for the camera signal |
| `--fps FLOAT` | `30` | selftest/compare | synthesized video frame rate |
| `--search FLOAT` | `200` | all offset modes | offset search half-width (ms); must exceed the true offset |
| `--initial FLOAT` | `0` | all offset modes | rough starting offset (ms) if you have a prior |
| `--lpf FLOAT` | `20` | all offset modes | low-pass cutoff (Hz); 20 matches Gyroflow |
| `--interp` | off | all offset modes | interpolated IMU lookup (removes quantization bias) |
| `--interp-parabolic` | off | all offset modes | `--interp` + sub-grid vertex (implies `--interp`) |
| `--swap-xy` | off | quat-derived ω | swap x/y when deriving ω from quaternions |
| `--inject LIST` | `-30,-15,-5,0,5,15,30` | selftest | comma-separated injected offsets (ms) |
| `--noise FLOAT` | `0` | selftest | Gaussian noise stddev added to the video signal (deg/s) |
| `--range A,B` | full | all | analyse only timestamps in `[A,B]` ms |
| `--seed INT` | `12345` | selftest | noise RNG seed |

---

## 6. Input / output formats

### Generic angular-velocity CSV (`--gyro` / `--video`)

A header row naming a timestamp column and a 3-axis set, then data rows:

```
timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s
0.0,1.23,-0.45,0.10
16.6667,1.40,-0.30,0.22
...
```

- **Timestamp column** (first match): `timestamp_ms` / `time_ms` / `t_ms` / `ts_ms` (ms);
  `timestamp_s` / `t_s` / `seconds` (s); or bare `timestamp` / `time` / `t` / `ts` (assumed ms).
- **Axis columns** (first match): `wx_deg_s,wy_deg_s,wz_deg_s` · `wx_rad_s,...` · `wx,wy,wz` ·
  `gx,gy,gz` · `gyro_x,gyro_y,gyro_z` · `omega_x,omega_y,omega_z` · `x,y,z`.
- **Units** inferred from a `_deg`/`_rad` axis suffix, else from `--units`.
- Timestamps are re-based to 0. The `omega` sub-command's output round-trips directly.

### GCSV IMU log (`--gyro`)

The format telemetry-parser / Gyroflow emit — a metadata block, then a `t,gx,gy,gz` data header:

```
GYROFLOW IMU LOG
version,1.3
tscale,0.001
gscale,1.0
orientation,xzY
t,gx,gy,gz
0,0.0123,-0.0045,0.0010
1,0.0140,-0.0030,0.0022
...
```

Honoured headers: `tscale` (raw → seconds, default 0.001), `gscale` (raw → rad/s, default 1.0),
`orientation` (axis map; overridden by `--gyro-orientation`). Auto-detected vs the generic CSV by the
metadata block preceding the `t`/`time` header.

### DJI quaternion CSV (`--quat`)

Accepts `dji_quaternions_full.csv` or `dji_camera_data.csv`. Columns auto-detected:
`quat_timestamp_ms` / `timestamp_ms` plus `quat_w,quat_x,quat_y,quat_z` (or `org_quat_*`).
Quaternions are `(w,x,y,z)`, normalized on load.

### Output

`sync`/`compare` print a single `offset_ms=… cost=… matched=…/… mode=…` line on stdout.
`selftest` prints a CSV table + a summary. `omega` prints a `timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s`
CSV. Diagnostics (signal stats, warnings) go to stderr so stdout stays pipe-clean.

---

## 7. Precision modes (`nearest` vs `interp` vs `parabolic`)

| Mode | Flag | Parity | Accuracy | Use when |
|------|------|--------|----------|----------|
| nearest | *(default)* | bit-for-bit with Rust/C++ | ceiling ≈ one IMU sample | you need byte-exact parity |
| interp | `--interp` | breaks parity | removes the systematic bias | coarse IMUs (≤ ~500 Hz), accuracy matters |
| interp+parabolic | `--interp-parabolic` | breaks parity | + removes the 0.01 ms grid step | **default recommendation** |

**Why it matters.** `nearest` snaps every query up to the next IMU sample, so on a coarse IMU it
carries a deterministic, same-sign bias that does *not* average out. `interp` removes it; `parabolic`
removes the remaining refine-grid quantization.

**Worked example** (200 Hz IMU, 60 fps camera, true offset 8.5 ms):

```
nearest          -> offset_ms = 10.0000   (≈ +1.5 ms quantization bias)
interp+parabolic -> offset_ms =  8.4940   (≈ −0.006 ms error)
```

Note that the *systematic bias* is what `interp` targets. Residual per-estimate **scatter** (~0.1 ms
on short or low-motion clips) is conditioning-limited, not a quantization effect — average several
well-excited segments to beat it down (≈ 1/√N).

---

## 8. Python API reference

Import as a library: `import autosync_time as at`.

### Quaternion helpers
- `quat_normalize(q) -> (4,)` — normalize `(w,x,y,z)`.
- `quat_inverse_unit(q) -> (4,)` — conjugate of a unit quaternion.
- `quat_mul(a, b) -> (4,)` — Hamilton product.
- `quat_from_axis_angle(axis, radians) -> (4,)`.

### `Lowpass`
- `Lowpass.filter_forward_backward(freq_hz, sample_rate_hz, x) -> np.ndarray | None` — zero-phase
  20 Hz-style Butterworth on a 1-D array; `None` if `2·freq > sample_rate` (Nyquist guard).
- `Lowpass.filter_gyro_forward_backward(freq_hz, sample_rate_hz, gyro) -> bool` — in-place on an
  `(N,3)` array.

### Signals
- `GyroSeries(t, w)` — dataclass; `t` is `(N,)` ms, `w` is `(N,3)` deg/s. `len()` = N.
- `quaternions_to_angular_velocity(t_ms, quats, swap_xy=False, degrees=True) -> GyroSeries`.
- `resample_angular_velocity(series, target_t_ms) -> GyroSeries` — per-axis linear interp,
  end-clamped.
- `max_angle(series) -> float` — largest `|ω|` component (the motion gate).
- `estimate_sample_rate_hz(series) -> float` — robust rate from the median Δt.
- `orient_vec(w, orientation) -> (N,3)` — apply a 3-char axis map.

### Offset search
- `find_offset(of, gyro, initial_offset_ms, search_size_ms, of_sample_rate_hz, gyro_sample_rate_hz,
  lpf_hz=20.0, interp=False, parabolic=False) -> OffsetResult`
  where `of` is the video/camera signal, `gyro` is the IMU signal.
- `OffsetResult(found: bool, offset_ms: float, cost: float, matched: int)`.

### Loaders
- `load_quaternions(path) -> (t_ms (N,), quats (N,4))`.
- `load_gcsv(path, orientation=None) -> GyroSeries`.
- `load_angular_velocity_csv(path, units="deg", orientation=None) -> GyroSeries`.
- `load_motion(path, units="deg", orientation=None) -> GyroSeries` — auto-routes GCSV vs generic.

**Minimal library example:**

```python
import autosync_time as at

gyro  = at.load_motion("imu.gcsv")
video = at.load_motion("camera_motion.csv")
r = at.find_offset(video, gyro, 0.0, 200.0,
                   at.estimate_sample_rate_hz(video),
                   at.estimate_sample_rate_hz(gyro),
                   lpf_hz=20.0, interp=True, parabolic=True)
print(r.offset_ms if r.found else "no lock")
```

---

## 9. Recipes / end-to-end workflows

**A. Sync a real IMU log against an optical-flow motion CSV**
```sh
python3 gyroflow_autosync.py sync --gyro flight.gcsv --video flow_omega.csv --interp-parabolic
```

**B. Convert a DJI quaternion clip to ω, then sync two such clips**
```sh
python3 gyroflow_autosync.py omega --quat camA.csv > a.csv
python3 gyroflow_autosync.py omega --quat camB.csv > b.csv
python3 gyroflow_autosync.py sync --gyro a.csv --video b.csv --interp-parabolic
```

**C. Fix mismatched axes before syncing**
```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video cam.csv --gyro-orientation xzY --interp
```

**D. Measure the tool's own precision on your data**
```sh
python3 gyroflow_autosync.py selftest --quat my_clip.csv --fps 30 \
    --inject="-20,-7.3,0,8.5,18" --noise 1.0 --interp-parabolic
```

**E. Restrict to a well-excited time window**
```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video cam.csv --interp \
    --search 100      # narrower window once you know the offset is small
```

---

## 10. Troubleshooting & FAQ

| Symptom | Likely cause / fix |
|---------|--------------------|
| `no acceptable offset found` | True offset is outside `±search` → raise `--search`. Or signals barely overlap / wrong units → check the stderr `span` and `max|omega|` lines. |
| `--inject` errors with "expected one argument" | Leading `-` parsed as a flag → use `--inject="-30,..."` (with `=`). |
| `warning: motion below the 3 deg/s gate` | Not enough rotation to lock; use a more dynamic window via `--range`, or accept low confidence. |
| Offset looks ~1–2 ms off on a coarse IMU | That's the `nearest` quantization bias → add `--interp-parabolic`. |
| Sync won't lock at all but motion is strong | Axis conventions differ → set `--gyro-orientation` / `--video-orientation`. |
| `could not estimate a sample rate` | A signal has < 2 samples / identical timestamps. |
| Result differs from the C++/Rust tool by one 0.01 ms step | Expected float-accumulation tie in `nearest`; `interp` avoids it. |
| `unrecognised CSV header` / `no recognised angular-velocity columns` | Rename columns to a supported set (see §6) or use the `omega` dump format. |

**Which mode do I want?** Use `sync` for real data; `selftest` to benchmark precision; `compare`
for the zero-offset bias floor; `omega` to convert quaternions to ω.

**Does it handle dropped/variable frames?** Yes for alignment and ω magnitude — see §11.

---

## 11. Variable frame timing (VFR)

The pipeline is timestamp-based, so non-uniform frame intervals are handled in most places:

1. **Offset search** matches on each sample's *real* timestamp (`gyro at of.t[k] − offs`), never on
   `k·dt`. Feed real per-frame timestamps and uneven spacing is fine.
2. **ω magnitude** divides each interval's rotation by its actual `Δt`, not a constant fps.
3. **Low-pass** is the only part that assumes a uniform rate (the biquad takes one scalar
   `sample_rate`). In practice this rarely bites: at 20 Hz cutoff the video side is skipped by the
   Nyquist guard when `2·20 > fps` (24/30 fps), and the IMU is a steady ~1 kHz stream.

For heavy VFR on a side that *is* filtered (60/120 fps video), mitigate with: the median frame
interval as the nominal rate, resampling both onto a uniform grid before filtering and matching on
real timestamps afterward, or a time-aware filter (deviates from biquad parity).

---

## 12. Parity with the Rust/C++ engine

The default (`nearest`) path is line-for-line faithful: 1 ms coarse sweep → 0.01 ms refine over
±2 ms, weighted least-squares cost (x,y ×70, z ×100), nearest-upper IMU lookup (microsecond keys,
truncated toward zero), 90% acceptance window, and the `2·f0 > fs` Nyquist skip. On
`data/dji_quaternions_full.csv`, recovered offsets and costs match `cpp_core/build/gyroflow_autosync`
to 4 decimals wherever the cost minimum is well-defined; ties can differ by at most one 0.01 ms
refine step from float accumulation. `interp`/`parabolic` intentionally break bit-exact parity in
exchange for accuracy.

---

## 13. Measured precision

On the bundled DJI clip at 30 fps (see `README.md` → "Precision & accuracy" and
`variance_experiment.py` / `segment_*` studies):

| mode | bias | std | repeat-error @7.3 ms |
|------|------|-----|----------------------|
| `nearest` (default) | +0.166 ms | 0.210 ms | +0.446 ms |
| `interp` | **−0.005 ms** | **0.020 ms** | **+0.009 ms** |

- Random jitter floor ≈ 0.05 ms; the `nearest` systematic bias (+0.2…0.5 ms, bounded by one IMU
  sample) does **not** average out — only `interp` removes it.
- Averaging N well-excited segments shrinks the random component ≈ 1/√N. Net: `interp` (kills bias)
  + segment averaging (kills variance) reaches ~0.02 ms.

---

## 14. Limitations & future work

**Limitations**
- Does not decode video — you supply the camera-side motion as angular velocity.
- Estimates a single constant offset, not clock drift (offset + skew).
- The low-pass assumes a uniform sample rate (see §11).
- Motion-gated: unreliable below ~3 deg/s of excitation.

**Future work** (see [`QUICKSTART.md`](QUICKSTART.md) → "Future improvements" and [`TODO.md`](TODO.md)
for the prioritized roadmap)
- Accept a quaternion CSV directly for `--video` (derive ω internally).
- Normalized 0–1 confidence score + JSON output for pipeline integration.
- Clock-drift model: segmented offsets + linear fit → skew (ppm) and an offset+skew correction.
- GCC-PHAT / cross-spectrum phase-slope estimator; Huber loss + per-segment motion gating.
- Port `interp`/`parabolic` to the C++ `findOffset` for cross-port parity.

---

## 15. Repository map

| File | Role |
|------|------|
| `autosync_time.py` | Core library: quaternion helpers, `Lowpass`, ω derivation, `find_offset`, loaders. |
| `gyroflow_autosync.py` | CLI: `sync` / `selftest` / `compare` / `omega`. |
| `test_autosync_time.py` | Unit tests (lowpass, ω, recovery, noise, interp bias, loaders, sync). |
| `variance_experiment.py` | Monte-Carlo precision study (bias / variance / range vs noise). |
| `segment_experiment.py` | Per-segment offset under one fixed truth (nearest vs interp) → PNG/CSV. |
| `segment_stats.py` | Segment-count sweep → bias / repeatability / 1/√N averaging → PNG/CSV. |
| `QUICKSTART.md` | Fast path: sync, formats, precision modes, deployment status. |
| `DOCUMENTATION.md` | This document — the complete reference. |
| `TODO.md` | Prioritized engineering roadmap. |
| `README.md` | Overview, parity details, VFR notes, precision studies. |

Related ports: `../cpp_core/` (C++ `gyroflow_autosync` + `AUTOSYNC_TIME_REPORT.md`),
`../tools/gcsv_simple_gyro_compare.py` (GCSV gyro comparison).
