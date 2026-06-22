# Autosync-Time — TODO (higher-precision roadmap)

Ordered by cost → payoff. Context and measured numbers in `README.md` (Precision & accuracy) and
`../cpp_core/AUTOSYNC_TIME_REPORT.md` §8.

## Done
- [x] **Deployment: `sync` mode on real signals.** Real-signal loaders (`load_gcsv`,
      `load_angular_velocity_csv`, `load_motion`) + a `sync` CLI mode that takes a GCSV IMU log and a
      camera-motion CSV, auto-estimates sample rates, motion-gates, and reports the offset. Tests:
      `test_load_gcsv`, `test_load_angular_velocity_csv`, `test_sync_recovery_two_signals`. See
      `QUICKSTART.md`.
- [x] Monte-Carlo precision study (`variance_experiment.py`): random jitter σ≈0.05 ms, systematic
      bias +0.2…0.5 ms bounded by one IMU sample, gyro zero-bias ≲0.1 ms.
- [x] Interpolated IMU lookup `find_offset(..., interp=True)` — bias +0.166→−0.005 ms,
      repeat-error +0.446→+0.009 ms (~50×). Default stays nearest-upper (Rust/C++ parity).
- [x] Fixed one-sided refine window for the interp path ([center-2, center] → symmetric [-2,+2]).

## Next (cheap, µs-level)
- [x] **Parabolic peak interpolation** of the cost curve (analytic vertex of the 3 points around
      the minimum) to remove the residual 0.01 ms refine-grid step. `find_offset(..., parabolic=True)`;
      pairs with `interp`. On the coarse-IMU fixture it tracks `interp` to within a few µs.
- [x] Wire `--interp` (and `--interp-parabolic`, which implies `--interp`) through
      `gyroflow_autosync.py` CLI.
- [x] Added `test_interp_removes_quantization_bias`: a 200 Hz IMU / 120 fps video fixture where
      nearest carries a ~+1.9 ms systematic bias and `interp` removes it (|mean error| < 0.05 ms).
      Note: the *mean* (bias) is what `interp` targets — per-injection scatter (~0.1 ms) on this
      fixture is video-sampling/motion-conditioning limited, not an IMU-quantization effect.

## Clock drift (the "segments differ by ~1 ms" case)
- [ ] **Segmented offset + linear fit** → report slope as clock skew (ppm); offer offset+skew
      correction (resample one stream onto the other's clock). Add a `segment_offsets()` helper +
      a diagnostic that flags "trend (drift)" vs "scatter (low-motion conditioning)".

## Bigger (research-grade)
- [ ] **GCC-PHAT** / cross-spectrum phase-slope estimator as an alternative to the time-domain
      sweep (sub-sample, robust on band-limited motion).
- [ ] Robust loss (Huber) + inverse-covariance weighting; motion gate to drop low-excitation
      segments before fitting.
- [ ] Evaluate against / interoperate with continuous-time (Kalibr-style) calibration for a
      ground-truth-grade reference.

## Parity / cross-port
- [ ] Port the `interp` switch to C++ `findOffset` (see report §6, §8) so both ports match.
