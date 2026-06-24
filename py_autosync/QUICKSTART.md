# Autosync-Time — Quick Start

Find the constant timestamp offset (ms) between an IMU/gyro stream and a camera-motion stream, so
the two can be aligned for stabilization. Faithful port of Gyroflow's
`essential_matrix::find_offsets` + `filtering::Lowpass`, plus a higher-precision interpolated mode.

> Need the full reference (every mode/flag, formats, Python API, troubleshooting)? See
> [`DOCUMENTATION.md`](DOCUMENTATION.md).

## 1. Install

```sh
cd py_autosync
pip install -r requirements.txt        # numpy, scipy
python3 test_autosync_time.py          # sanity check — all tests should pass
```

## 2. Sync two real signals (the production path)

You need **two angular-velocity streams of the same motion**, each with its own real timestamps:

- `--gyro` — the IMU log. Accepts a **Gyroflow GCSV** log (auto-detected; honours `tscale`/`gscale`
  and an `orientation` header) or any angular-velocity CSV.
- `--video` — the camera-side motion (e.g. an optical-flow `estimated_gyro`, or another gyro). Any
  angular-velocity CSV.

```sh
# GCSV IMU log + a camera-motion CSV; high-precision mode:
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video camera_motion.csv --interp-parabolic
```

Output (offset and a confidence line):

```
gyro : 1200 samples, ~200.0 Hz, span [0.0,5995.0] ms, max|omega| 73.16 deg/s
video: 360 samples,  ~60.0 Hz, span [0.0,5983.3] ms, max|omega| 70.08 deg/s
offset_ms=8.4940 cost=2453.5929 matched=360/360 mode=interp+parabolic
```

`offset_ms` is the delay: **the gyro aligns to video timestamps shifted by `-offset_ms`**
(same convention as Gyroflow). Lower `cost` and `matched == video samples` indicate a clean fit.

### Accepted input formats

**Generic angular-velocity CSV** — a header row naming a timestamp column and a 3-axis set:

```
timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s
0.0,1.23,-0.45,0.10
...
```

- Timestamp column: `timestamp_ms` / `t_ms` (ms), `timestamp_s` / `t_s` / `seconds` (s), or bare
  `t` / `time` / `timestamp` (assumed ms). Timestamps are re-based to start at 0.
- Axis columns (first match wins): `wx_deg_s,wy_deg_s,wz_deg_s` · `wx_rad_s,...` · `wx,wy,wz` ·
  `gx,gy,gz` · `gyro_x,gyro_y,gyro_z` · `omega_x,...` · `x,y,z`.
- Units: inferred from a `_deg`/`_rad` suffix; otherwise set with `--units deg|rad`.
- The `omega` sub-command's own output is a valid input (round-trips directly).

**GCSV** — a metadata block, then a `t,gx,gy,gz` data header (gyro stored rad/s × `gscale`,
timestamps × `tscale`). This is what telemetry-parser / Gyroflow emit.

### Useful flags

| Flag | Meaning |
|------|---------|
| `--search 200` | offset search half-width in ms (must exceed the true offset) |
| `--initial 0` | rough starting offset (ms) if you have a prior |
| `--lpf 20` | low-pass cutoff (Hz); 20 matches Gyroflow |
| `--interp` | interpolated IMU lookup — removes the nearest-sample quantization bias |
| `--interp-parabolic` | `--interp` + analytic sub-grid vertex (µs-level; **recommended**) |
| `--units deg\|rad` | units when not inferable from column names |
| `--gyro-orientation xzY` | 3-char axis remap for the IMU signal (upper keeps, lower negates) |
| `--video-orientation ...` | same, for the camera signal |

> **Axes must be comparable.** The cost compares per-axis (x,y ×70, z ×100). If the two streams use
> different axis conventions, fix it with `--gyro-orientation` / `--video-orientation` first, or the
> sync may not lock. See the IMU-orientation notes in the repo memory.

## 3. Which precision mode?

| Mode | Flag | When |
|------|------|------|
| `nearest` | *(default)* | Bit-for-bit parity with the Rust/C++ engine. Accuracy ceiling ≈ one IMU sample. |
| `interp` | `--interp` | Removes the systematic nearest-sample bias (big win on coarse, e.g. ≤500 Hz, IMUs). |
| `interp+parabolic` | `--interp-parabolic` | As above + removes the 0.01 ms refine-grid step. Best accuracy. |

On a 200 Hz IMU the default carries a ~+1.5–2 ms quantization bias; `--interp-parabolic` removes it
(recovered 8.494 ms vs a true 8.5 ms in the bundled example). Use `--interp-parabolic` unless you
specifically need byte-exact parity with the Rust engine.

## 4. Ground-truthed evaluation (no real camera signal needed)

Synthesizes the camera side from DJI fused quaternions to measure recovery accuracy:

```sh
# Inject known offsets, measure recovery (timestamp-sync precision):
python3 gyroflow_autosync.py selftest --quat ../data/dji_quaternions_full.csv \
    --fps 30 --inject="-30,-10,0,10,30" --noise 1.5
# Baseline bias at true offset 0:
python3 gyroflow_autosync.py compare  --quat ../data/dji_quaternions_full.csv --fps 30
# Dump quaternion-derived angular velocity (re-ingestable by `sync`):
python3 gyroflow_autosync.py omega    --quat ../data/dji_quaternions_full.csv > omega.csv
```

Note: pass negative offset lists with `=`, e.g. `--inject="-30,..."`.

## 5. Deployment status & limits

**Ready** for offline batch syncing of two real angular-velocity streams: GCSV + camera-motion CSV
in, offset out, with a motion-gate confidence check and auto sample-rate estimation. Sub-millisecond
on well-excited motion (`--interp-parabolic`).

Know the boundaries:

- **It does not decode video.** You must supply the camera-side motion already as angular velocity
  (optical-flow `estimated_gyro`, another gyro, or — for evaluation — DJI quaternions).
- **Single constant offset.** It does not estimate clock *drift* (offset + skew). If per-segment
  offsets trend over the clip, that's drift — see `README.md` → "Segmented estimates differ by ~1 ms".
- **Low-pass assumes a uniform rate.** Fine for steady IMU/video rates; heavy VFR on the *filtered*
  side needs mitigation (`README.md` → "Variable frame timing").
- **Motion-gated.** Below ~3 deg/s of excitation the offset is unreliable (the tool warns).

See `TODO.md` for the prioritized roadmap.

## Future improvements

Short term (cheap, high value):

- **Quaternion / attitude input for `--video`** — accept a quaternion CSV directly and derive ω
  internally (reuse `quaternions_to_angular_velocity`), so DJI/camera attitude logs need no
  pre-conversion.
- **Confidence score** — turn the raw `cost` + cost-curve curvature into a normalized 0–1 quality
  metric and a clear pass/fail, instead of an unscaled cost number.
- **`--out` JSON** — emit a machine-readable result (offset, cost, rates, mode, quality) for
  pipeline integration.
- **Optional time/orientation auto-search** — try `--video-orientation` candidates and report the
  best-locking one when axis conventions are unknown.

Accuracy / robustness:

- **Clock-drift model** — segmented offsets + linear fit → report skew (ppm) and offer an
  offset+skew correction (resample one stream onto the other's clock). (`TODO.md` "Clock drift".)
- **GCC-PHAT / cross-spectrum phase-slope** estimator as a sub-sample alternative to the time-domain
  sweep, robust on band-limited motion.
- **Robust loss (Huber) + motion gating per segment** to drop poorly-excited spans before fitting.
- **Time-aware low-pass** for heavy-VFR footage (deviates from biquad parity; opt-in).

Parity / ecosystem:

- **Port `interp`/`parabolic` to the C++ `findOffset`** so both ports match (`TODO.md` "Parity").
- **Validate against a continuous-time (Kalibr-style) reference** for a ground-truth-grade baseline.
