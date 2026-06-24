# Autosync-Time (Python)

A clean, UI-free Python tool that finds the constant **timestamp offset** (ms) between a camera's
motion and its IMU/gyro stream — the time-alignment half of Gyroflow's sync. Faithful port of
`essential_matrix::find_offsets` + `filtering::Lowpass`, plus a higher-precision interpolated mode
and basic visualization.

For the algorithm, parity notes, measured precision and experiment conclusions, see
[`DESIGN.md`](DESIGN.md).

## Install

```sh
pip install -r requirements.txt    # numpy, scipy (+ matplotlib for visualize.py)
python3 test_autosync_time.py      # 8 tests, all should print OK
```

## Quick use

```sh
# Production: sync two real signals (IMU GCSV + camera-motion CSV) -> offset in ms
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video camera_motion.csv --interp-parabolic

# Visualize the estimate (cost curve + alignment overlay) -> PNG
python3 visualize.py --gyro imu.gcsv --video camera_motion.csv --interp-parabolic --out sync.png

# Ground-truthed accuracy: inject known offsets, measure recovery
python3 gyroflow_autosync.py selftest --quat ../data/dji_quaternions_full.csv \
    --fps 30 --inject="-30,-10,0,10,30" --noise 1.5

# Dump quaternion-derived angular velocity as CSV (re-ingestable by `sync`)
python3 gyroflow_autosync.py omega --quat ../data/dji_quaternions_full.csv > omega.csv
```

`offset_ms` is the delay such that the **IMU is sampled at `video_time − offset_ms`** (Gyroflow's
convention). Pass negative `--inject` lists with `=`, e.g. `--inject="-30,..."`.

## CLI modes (`gyroflow_autosync.py`)

| Mode | Inputs | Purpose |
|------|--------|---------|
| `sync` | `--gyro` + `--video` | Offset between two real angular-velocity signals (production). |
| `selftest` | `--quat` | Inject known offsets, measure recovery → precision. |
| `compare` | `--quat` | Offset vs a video-rate resample (true offset 0) → bias floor. |
| `omega` | `--quat` | Dump quaternion-derived ω as CSV. |

Common flags: `--search` (search half-width ms), `--initial`, `--lpf` (Hz), `--interp` /
`--interp-parabolic` (precision), `--units deg|rad`, `--gyro-orientation` / `--video-orientation`
(3-char axis remap, e.g. `xzY`).

## Precision modes

| Mode | Flag | Notes |
|------|------|-------|
| nearest | *(default)* | Bit-for-bit parity with the Rust/C++ engine; ceiling ≈ one IMU sample. |
| interp | `--interp` | Linear IMU lookup — removes the nearest-sample quantization bias. |
| interp+parabolic | `--interp-parabolic` | + analytic sub-grid vertex; **recommended** for accuracy. |

On a 200 Hz IMU the default carries a ~+1.5 ms quantization bias; `--interp-parabolic` removes it
(recovers 8.494 ms vs a true 8.5 ms). See `DESIGN.md` for the full measured numbers.

## Input formats

- **Generic angular-velocity CSV** (`--gyro`/`--video`): a header naming a timestamp column
  (`timestamp_ms` / `t_s` / `t` / …) and a 3-axis set (`wx_deg_s,…` / `wx,wy,wz` / `gx,gy,gz` /
  `x,y,z` / …). Units inferred from a `_deg`/`_rad` suffix, else `--units`. The `omega` output
  round-trips directly.
- **GCSV** (`--gyro`): the telemetry-parser / Gyroflow IMU log (honours `tscale`, `gscale`,
  `orientation`). Auto-detected vs generic CSV.
- **DJI quaternion CSV** (`--quat`): `dji_quaternions_full.csv` or `dji_camera_data.csv`.

## Visualization (`visualize.py`)

Renders a 3-panel diagnostic PNG: **(A)** cost vs offset over the full search window, **(B)** a
±3 ms zoom on the minimum, **(C)** an angular-velocity overlay in a peak-motion window with the gyro
aligned by the recovered offset (faint grey = before alignment). Works on real signals
(`--gyro`/`--video`) or synthetic (`--quat --fps --inject`, which also draws the injected truth).

```sh
python3 visualize.py --quat ../data/dji_quaternions_full.csv --fps 30 --inject 8.5 --out demo.png
```

## Files

| File | Role |
|------|------|
| `autosync_time.py` | Core library: quaternion helpers, `Lowpass`, ω derivation, `find_offset`, `cost_curve`, loaders. |
| `gyroflow_autosync.py` | CLI: `sync` / `selftest` / `compare` / `omega`. |
| `visualize.py` | Basic visualization (cost curve + alignment overlay). |
| `test_autosync_time.py` | Unit tests. |
| `variance_experiment.py`, `segment_experiment.py`, `segment_stats.py` | Reproduce the precision studies summarized in `DESIGN.md`. |
| `DESIGN.md` | Algorithm, parity, precision/experiment conclusions, VFR, future work. |
