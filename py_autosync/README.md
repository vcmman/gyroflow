# Python Autosync-Time

UI-free Python port of Gyroflow's gyro↔video time-offset finder ("autosync time"), doubling as a
timestamp-synchronization accuracy evaluator. It is a sibling of the C++ port in `cpp_core/`
(`gyroflow_autosync`) and implements the **same algorithm** ported from the Rust engine
(`essential_matrix::find_offsets` + `filtering::Lowpass`).

Given two angular-velocity time series (one from the video, one from the IMU) it finds the
constant timestamp delay (ms) that best aligns them. The video-side motion is synthesised from
DJI fused quaternions rather than decoded from pixels, which also enables ground-truthed accuracy
evaluation (inject a known offset, measure recovery).

## Requirements

```sh
pip install -r requirements.txt   # numpy, scipy
```
`scipy.signal.lfilter` (with zero initial state) reproduces the Rust biquad
`DirectForm2Transposed` output exactly — verified to ~4e-13.

## Usage

```sh
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
| `autosync_time.py` | Core: quaternion helpers, `Lowpass`, `quaternions_to_angular_velocity`, `resample_angular_velocity`, `find_offset`, `load_quaternions` |
| `gyroflow_autosync.py` | CLI: `selftest` / `compare` / `omega` |
| `test_autosync_time.py` | Unit tests (lowpass, ω, recovery, noise robustness) |

## Tests

```sh
python3 test_autosync_time.py     # standalone
pytest test_autosync_time.py      # or via pytest
```

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
