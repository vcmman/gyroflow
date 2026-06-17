# Autosync-Time — TODO (higher-precision roadmap)

Ordered by cost → payoff. Context and measured numbers in `README.md` (Precision & accuracy) and
`../cpp_core/AUTOSYNC_TIME_REPORT.md` §8.

## Done
- [x] Monte-Carlo precision study (`variance_experiment.py`): random jitter σ≈0.05 ms, systematic
      bias +0.2…0.5 ms bounded by one IMU sample, gyro zero-bias ≲0.1 ms.
- [x] Interpolated IMU lookup `find_offset(..., interp=True)` — bias +0.166→−0.005 ms,
      repeat-error +0.446→+0.009 ms (~50×). Default stays nearest-upper (Rust/C++ parity).
- [x] Fixed one-sided refine window for the interp path ([center-2, center] → symmetric [-2,+2]).

## Next (cheap, µs-level)
- [ ] **Parabolic peak interpolation** of the cost curve (analytic vertex of the 3 points around
      the minimum) to remove the residual 0.01 ms refine-grid step. Pairs naturally with `interp`.
- [ ] Wire `--interp` (and an `--interp-parabolic`) flag through `gyroflow_autosync.py` CLI.
- [ ] Add an `interp`-mode unit test asserting recovery error < 0.05 ms on the random-walk fixture.

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
