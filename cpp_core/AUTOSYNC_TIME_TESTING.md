# Autosync-Time — Testing & Evaluation Method

How to verify the C++ autosync-time port (`timesync.{hpp,cpp}`, `gyroflow_autosync`) and how to
measure timestamp-synchronization accuracy. For results see
[`AUTOSYNC_TIME_REPORT.md`](AUTOSYNC_TIME_REPORT.md); for design see
[`AUTOSYNC_TIME_PLAN.md`](AUTOSYNC_TIME_PLAN.md).

## TL;DR — one command

```sh
cpp_core/tools/run_autosync_eval.sh            # auto-locates ../data (or the main git worktree)
cpp_core/tools/run_autosync_eval.sh /path/to/data
```
It builds, runs the unit tests, cross-checks the ω derivation, prints the zero-offset baseline,
and prints the offset-recovery precision table (fps × noise). Requires `cmake`, a C++17 compiler,
and `data/dji_quaternions_full.csv`.

## What we test, and why each is convincing

The hard, parity-critical part of autosync is the **time-offset search**: given two
angular-velocity signals, find the constant delay that aligns them. We do **not** decode video;
instead the "video-side" motion is synthesised from the DJI fused quaternions. That choice is
what makes rigorous testing possible — we can inject a *known* offset and check recovery, which
the real pipeline (no ground truth) cannot do.

### 1. Unit tests (`tests/test_timesync.cpp`, `ctest`)
Self-contained, no data files. A `CHECK` macro is used instead of `assert` so the checks still
run in Release (where `NDEBUG` strips `assert`). Cases:
- **Lowpass** — DC gain ≈ 1 (unit pass-through); a 150 Hz tone is attenuated > 95 % at 20 Hz
  cutoff; the Nyquist guard (`2·f0 > fs`) leaves data untouched, matching the `biquad` crate.
- **ω from quaternions** — a constant 45 °/s yaw yields a constant 45 °/s on z (and the
  `swap_xy` flag maps a pure pitch rate onto x).
- **Offset recovery** — a realistic band-limited random-walk attitude (AR(1) body rates) is
  sampled at video fps with several injected offsets; all must be recovered within **0.6 ms**.
  (Pure sinusoids are deliberately *not* used: their autocorrelation sidelobes make the search
  multi-modal — see the report's "Known limitation".)
- **Noise robustness** — the recovery case is repeated with Gaussian sensor noise added to the
  video signal; recovery must stay within tolerance.

Run just these:
```sh
cmake -S cpp_core -B cpp_core/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp_core/build -j
( cd cpp_core/build && ctest --output-on-failure )   # --test-dir needs CMake >= 3.20
# or directly: cpp_core/build/gyroflow_cpp_timesync_tests
```

### 2. ω derivation cross-check (vs the Python tools)
`gyroflow_autosync omega` should match `data/dji_quat_analysis.csv`'s `quat_omega_*_rad_s`
(our output is deg/s; multiply the CSV by 57.2958). This confirms the quaternion→rate step that
feeds the search.
```sh
cpp_core/build/gyroflow_autosync omega --quat data/dji_quaternions_full.csv | head
```

### 3. Zero-offset baseline (`compare`)
Build a video-rate resample of the full-rate signal with **no** injected shift and confirm the
recovered offset is ≈ 0. This isolates the algorithm's intrinsic bias floor from any real delay.
```sh
cpp_core/build/gyroflow_autosync compare --quat data/dji_quaternions_full.csv --fps 30
```

### 4. Offset-recovery precision (`selftest`) — the headline metric
Inject a sweep of known offsets (optionally with noise), recover each, and report
mean / RMS / max error. This is the **timestamp-sync precision** number.
```sh
cpp_core/build/gyroflow_autosync selftest --quat data/dji_quaternions_full.csv \
    --fps 30 --search 120 --inject "-50,-25,-10,-3,0,3,10,25,50,80" --noise 1.5
```
Sweep `--fps` (24/30/60/120) and `--noise` to characterise accuracy vs frame rate.

## Acceptance criteria
- `ctest`: 100 % pass.
- ω cross-check: matches the analysis CSV to ~1e-3 rad/s.
- `compare`: |recovered| ≤ ~0.05 ms at all tested frame rates.
- `selftest` on the DJI clip: RMS error ≤ ~0.5 ms, max ≤ ~1 ms, 100 % of offsets recovered.

## Interpreting the numbers
A small **positive** bias (~0.3 ms) is expected: the cost looks up the nearest IMU sample at or
after the query time (1 kHz ⇒ ~1 ms grid), exactly as the Rust code does. It shrinks at higher
IMU/video rates. Errors that jump by whole milliseconds on smooth/periodic signals are the
multi-modal-cost limitation, not a regression — use rich (real or random-walk) motion.
