# Python Autosync-Time

UI-free Python port of Gyroflow's gyro↔video time-offset finder ("autosync time"), doubling as a
timestamp-synchronization accuracy evaluator. It is a sibling of the C++ port in `cpp_core/`
(`gyroflow_autosync`) and implements the **same algorithm** ported from the Rust engine
(`essential_matrix::find_offsets` + `filtering::Lowpass`).

Given two angular-velocity time series (one from the video, one from the IMU) it finds the
constant timestamp delay (ms) that best aligns them. The `sync` mode syncs two **real**
independently-measured signals (a GCSV IMU log + a camera-motion CSV); the `selftest`/`compare`
modes synthesise the video side from DJI fused quaternions for ground-truthed accuracy evaluation
(inject a known offset, measure recovery).

**New here? Start with [`QUICKSTART.md`](QUICKSTART.md)** — it covers the production `sync` path,
input formats, precision modes, deployment limits, and future improvements.

## Requirements

```sh
pip install -r requirements.txt   # numpy, scipy
```
`scipy.signal.lfilter` (with zero initial state) reproduces the Rust biquad
`DirectForm2Transposed` output exactly — verified to ~4e-13.

## Usage

```sh
# Production: sync two real signals (IMU GCSV + camera-motion CSV) -> offset in ms:
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video camera_motion.csv --interp-parabolic

# Inject known offsets and measure recovery accuracy (timestamp-sync precision):
python3 gyroflow_autosync.py selftest --quat ../data/dji_quaternions_full.csv \
    --fps 30 --search 120 --inject="-30,-10,0,10,30" --noise 1.5

# Baseline bias (true offset 0) between full-rate and video-rate signals:
python3 gyroflow_autosync.py compare --quat ../data/dji_quaternions_full.csv --fps 30

# Dump quaternion-derived angular velocity as CSV:
python3 gyroflow_autosync.py omega --quat ../data/dji_quaternions_full.csv > omega.csv
```

NOTE: pass negative offset lists with `=`, e.g. `--inject="-30,..."`, otherwise argparse treats
the leading `-` as an option flag.

Both `dji_quaternions_full.csv` and `dji_camera_data.csv` are auto-detected.

## Modules

| File | Role |
|------|------|
| `autosync_time.py` | Core: quaternion helpers, `Lowpass`, `quaternions_to_angular_velocity`, `resample_angular_velocity`, `find_offset`, `load_quaternions`, real-signal loaders (`load_gcsv`, `load_angular_velocity_csv`, `load_motion`) |
| `gyroflow_autosync.py` | CLI: `sync` (two real signals) / `selftest` / `compare` / `omega` |
| `test_autosync_time.py` | Unit tests (lowpass, ω, recovery, noise robustness, interp bias, loaders, sync) |
| `variance_experiment.py` | Monte-Carlo precision study: bias / variance / range vs noise, repeatability, gyro-bias robustness |
| `segment_experiment.py` | Inject one fixed offset, estimate per segment, plot recovered offset vs segment (nearest vs interp) → `segment_offsets.png/.csv` |
| `segment_stats.py` | Sweep segment count over many noise realizations → bias / repeatability / 1/√N variance averaging → `segment_stats.png/.csv` |

## Tests

```sh
python3 test_autosync_time.py     # standalone
pytest test_autosync_time.py      # or via pytest
```

## Variable frame timing (VFR / jitter / dropped frames)

The whole pipeline is **timestamp-based, not frame-index based** (a core Gyroflow design
principle), so non-uniform frame intervals are handled naturally in most places:

1. **Offset search** — `find_offset` matches on each sample's *real* timestamp
   (`gyro at of.t[k] - offs`), never on `k * dt`. As long as you feed real per-frame
   timestamps, uneven spacing is fine. (The CLI `selftest`/`compare` modes generate *uniform*
   frame times via `_frame_timestamps` only because they synthesise video; real data should pass
   actual timestamps.)
2. **Angular-velocity magnitude** — `quaternions_to_angular_velocity` divides each interval's
   rotation by its **actual** `Δt = t[i+1]-t[i]`, not a constant fps, so ω is correct under VFR.
   (For real optical-flow `estimated_gyro`, divide each frame's rotation by its real interval the
   same way.)
3. **Low-pass filter** — the *only* place that assumes a uniform sample rate (the Butterworth
   biquad is designed for one scalar `sample_rate`). In practice this rarely bites:
   - At cutoff 20 Hz, the video side is **skipped by the Nyquist guard when `2·20 > fps`** (i.e.
     fps < 40, e.g. 24/30), so VFR of the video never reaches the filter; only at 60/120 fps is
     the video actually filtered.
   - The IMU side is a steady ~1 kHz stream, unaffected by video VFR.

   For heavy VFR where the video side *is* filtered, mitigate with one of: use the median frame
   interval as the nominal rate (`sample_rate = 1000 / median(diff(t))`); resample both signals
   onto a uniform time grid before filtering and match on real timestamps afterward; or use a
   time-aware filter (deviates from the biquad parity, use with care).

**Summary:** offset alignment and ω magnitude are already VFR-safe; the residual approximation is
the low-pass sample rate, which typical video frame rates dodge via the Nyquist skip.

## Parity with the C++/Rust port

The algorithm is line-for-line faithful: 1 ms coarse sweep → 0.01 ms refine over ±2 ms, weighted
least-squares cost (x,y ×70, z ×100) over matched samples, nearest-upper IMU lookup (microsecond
keys, truncated toward zero), 90 % acceptance window, and the `2·f0 > fs` Nyquist skip in the
low-pass. On `data/dji_quaternions_full.csv` the recovered offsets and costs match
`cpp_core/build/gyroflow_autosync` to 4 decimal places wherever the cost minimum is well-defined
(e.g. inject 3/10/50/80 ms → cost 15987.3352 / 19570.3128 / 17342.8915 / 19164.8214 in both);
ties differ by at most one 0.01 ms refine step due to float accumulation in the frame timestamps.

On the bundled DJI clip, offset recovery is **sub-millisecond** (~0.4 ms RMS at 30 fps), with a
small positive bias from the nearest-IMU-sample lookup on the ~1 kHz grid — inherent to the
algorithm and kept for parity.

## Precision & accuracy (measured)

Monte-Carlo study on the bundled DJI clip (`variance_experiment.py`, 30 fps). Two error sources
are fundamentally different and must not be conflated:

| Source | Magnitude | Nature |
|--------|-----------|--------|
| Random jitter (noise-driven) | **σ ≈ 0.05 ms** at realistic SNR (peak \|ω\|≈530 °/s) | random; averages out over repeats. Only grows past ~0.15 ms when noise reaches 20–40 °/s |
| Systematic lookup bias | **+0.2…0.5 ms**, bounded by one IMU sample interval | deterministic; same value for the same data — *not* variance. The dominant spread across offsets/segments |
| Constant gyro zero-bias | **≲ 0.1 ms** even at 5 °/s | negligible — alignment is driven by the AC shape, not DC level |
| Refine grid floor | 0.01 ms | search step |

**Accuracy ceiling = one IMU sample interval (≈1 ms @ 1 kHz).** The estimator snaps each video
sample to the next IMU sample, so offsets finer than one sample land on the same pair and cannot
be distinguished. Repeatability on identical input is exact (deterministic estimator).

### Interpolated lookup (`interp=True`) — breaks the ceiling

`find_offset(..., interp=True)` linearly interpolates the IMU value at the exact (continuous)
query time instead of snapping to the next sample. This removes the quantization bias (parity with
the Rust/C++ port is intentionally given up). Measured on the DJI clip at 30 fps:

| mode | bias | std | repeat-error @7.3 ms |
|------|------|-----|----------------------|
| `nearest` (default, C++-faithful) | +0.166 ms | 0.210 ms | +0.446 ms |
| `interp` | **−0.005 ms** | **0.020 ms** | **+0.009 ms** |

≈50× better absolute accuracy (~0.01 ms). The default stays bit-for-bit faithful to the port; pass
`interp=True` for the high-precision mode. The interp path also fixes a latent bug in the original
refine loop, which only stepped 200×0.01 ms from `center-2` — i.e. searched the one-sided window
`[center-2, center]` and could never climb above the coarse pick; the interp path sweeps the full
symmetric `[center-2, +2]`.

### Parabolic sub-grid vertex (`parabolic=True`)

`find_offset(..., interp=True, parabolic=True)` adds an analytic refinement on top of `interp`:
after the 0.01 ms refine grid it fits a parabola through the three cost samples at
`(best−step, best, best+step)` and moves to its vertex
`δ = ½·(c₋ − c₊)/(c₋ − 2c₀ + c₊)` (guarded to a convex bracket with `|δ| ≤ 1`). This removes the
residual 0.01 ms grid step for ~two extra cost evaluations. On a coarse-IMU fixture it tracks
`interp` to within a few µs — the remaining error is no longer the refine grid but the IMU/video
sampling itself, so the gain over plain `interp` is small once the grid is already 0.01 ms.

From the CLI: `--interp` selects interpolated lookup; `--interp-parabolic` adds the vertex step
(and implies `--interp`).

### Segmented estimates differ by ~1 ms — what it means

If 5 per-segment offsets spread by ~1 ms, that is **20× the random floor (0.05 ms)**, so it is
*not* noise or gyro bias. ~1 ms is exactly the per-segment scatter ceiling (one IMU sample +
motion-conditioning). Diagnose by **pattern**: a monotonic trend vs segment time ⇒ real
camera/IMU **clock drift** (offset is time-varying; needs offset+skew correction, not a single
constant); random scatter correlated with low-motion / shallow-cost segments ⇒ poorly conditioned
segments; otherwise the residual is the lookup quantization (use `interp=True`).

`segment_experiment.py` and `segment_stats.py` demonstrate this directly: inject **one fixed
offset** over the whole clip, then estimate per segment. Findings (DJI clip, 30 fps, σ=1 °/s):

- Per-segment estimates still scatter ~1 ms even though the truth is constant — the scatter is
  per-segment conditioning + lookup quantization, **not** a time-varying offset. Shorter segments
  scatter more (per-seg std 0.16 ms @16 s → 0.57 ms @1 s).
- **The mean over segments is much more accurate than any single segment**, but for two different
  reasons that must be separated:
  - *Random* scatter averages out: the std of the clip-mean estimate is ~0.02–0.04 ms and shrinks
    roughly as 1/√N — averaging segments buys precision.
  - *Systematic* `nearest` bias does **not** average out: the clip-mean stays at **+0.3…0.4 ms**
    for any segment count (it is deterministic rounding, same sign every segment). Only
    `interp=True` removes it (clip-mean bias → ±0.05 ms).
- Net: **`interp` (kills the bias) + averaging multiple well-excited segments (kills the variance)**
  is what gets you to ~0.02 ms; averaging alone leaves the `nearest` quantization bias intact.

See `../cpp_core/AUTOSYNC_TIME_REPORT.md` §8 for the remaining higher-precision calibration
approaches (GCC-PHAT, continuous-time/Kalibr, offset+skew drift models); parabolic peak
interpolation is now implemented (`parabolic=True`, above).
