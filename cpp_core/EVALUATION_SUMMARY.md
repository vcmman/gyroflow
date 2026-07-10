# Stabilization evaluation — summary report

Consolidated findings from the image-domain evaluation of the C++ core's stabilization
(smoothing configs, black borders / zoom, the DCR enhancement, and a head-to-head vs DJI's
in-camera RockSteady). This is the executive summary; the detailed R&D, per-frame numbers, and
reproduce commands live in [`SMOOTHING_RND.md`](SMOOTHING_RND.md) §8, [`COMPARISON.md`](COMPARISON.md),
and [`figures/README.md`](figures/README.md).

**Test data:** DJI Osmo Action 6, 3840×2880 10-bit, 29.97 fps. Matched same-scene pairs — each
activity recorded twice, once stabilization-off (`dji6_L`, we stabilize) and once DJI-in-camera
(`dji6_R`, RockSteady) — durations match to <1 s: **run** (77 s / 49 s clips, violent vertical
bob) and **bike** (398 s clip, smooth high-frequency vibration).

**Metric:** `phaseCorrelate dy` — per-frame global vertical shift (px) between consecutive frames
of the stabilized output; `RMS dy` = overall vertical-shake magnitude, lower = steadier. Frames
resized to 640 px wide (square pixels → aspect-independent). Defined in
[`figures/README.md`](figures/README.md#metric-what-phasecorrelate-dy-means); ±0.1 px is noise.

---

## 1. Headline conclusions

1. **The C++ port is at parity with Rust Gyroflow.** Under identical default params the rendered
   outputs are indistinguishable: `dy` RMS within 0.3 %, per-frame corr 0.998–1.000, sub-0.1 px
   residual (decode/encode noise, not stabilization). → `COMPARISON.md` §4.

2. **Tier-1 shipped: DCR as `--enhanced`.** Direction-Consistency-Ratio gating is the confirmed
   default enhancement — **−31…35 % rendered vertical shake on both run and bike**, black border
   unchanged from default (~1.2 %), +8…13 % crop. The golden default is untouched
   (`dcr=false`, `--enhanced`≡`--dcr`, ctest 7/7). → §5, `SMOOTHING_RND.md` §8e.

3. **Per-axis smoothing was evaluated and excluded.** It helps one clip only, gives no gain on the
   harder run clip, and stacking it with DCR forces 8.6–8.8 % black border (required zoom stacks
   past the 1.30 clamp). Kept as an available flag, not in the preset. → §5.

4. **vs DJI in-camera (matched scene, FOV-matched 4:3): activity-dependent.**
   - **Running (violent bob): DCR is ~4× steadier than DJI** (0.49 vs 1.94; 0.96 vs 3.84).
   - **Biking (smooth): DJI is ~1.3× steadier** (0.31 vs 0.39), and that gap is entirely at the
     frame **periphery** (distortion / rolling-shutter residual), not the smoothing. → §6.

5. **Gaussian / L1 kernels: opt-in, not DCR replacements.** They minimise angular *acceleration*
   (smoothest path — Gaussian σ0.5 beats DCR everywhere on accel) but never match DCR on `dy`
   (amplitude). Two objectives, two winners. → §3; kernels live on `claude/gaussian-smoothing` /
   `claude/speed-bump-jolt-rnd`.

6. **Dynamic zoom: borders solved; look-ahead fixes pops.** Black borders come only from the
   `max_zoom` clamp; dynamic zoom beats static for *every* smoothing config (~0 % vs 2–8 %
   black-border frames at equal crop). 1 s FOV look-ahead (`--zoom-look-ahead 1`) does **not**
   change borders (already 0 by min-tracking) — it removes the causal zoom *pops* (~5–10× smaller
   jumps, offline-level crop). → §4, `SMOOTHING_RND.md` §8c′/§8h.

7. **L1 black borders eliminated by construction (`--l1-fit-crop`).** The bounded L1 modes
   breached the 130 % clamp on 3–7 % of frames at 4:3 (per-axis angle boxes can't be tight in
   the zoom domain). Per-frame constraint generation fits the path to the actual crop budget:
   zero borders on all four clips, beats the best static zero-border box on 3/4 (run0002 1.10
   vs 1.96 dy), stays under DJI on the violent clip (5.80 vs 5.95). `--l1-auto-box` derives the
   per-axis budgets from the lens geometry (roll 11.7°/pitch 16.0°/yaw 32.2° — the equal box 12
   was over-budget on roll and 3× conservative on yaw). → `SMOOTHING_RND.md` §8p/§8q,
   `figures/l1_fitcrop_dy_vs_borders.png`.

![DCR vs DJI head-to-head](figures/dji_headtohead_summary.png)

---

## 2. Port parity — Rust vs C++ (default params)

| clip | Rust RMS `dy` | C++ RMS `dy` | per-frame RMS(rust−cpp) | corr |
|---|---:|---:|---:|---:|
| run 0001 | 0.675 | 0.675 | 0.017 px | 1.000 |
| run 0002 | 1.374 | 1.369 | 0.074 px | 0.998 |

The port reproduces Rust on the actual pixels; all downstream comparisons use the C++ renderer as
a faithful stand-in for Gyroflow. Detail + figure: `COMPARISON.md` §4.

## 3. Consolidated smoothing-config comparison (all experiments)

Every smoothing config evaluated this session, on two metrics: **`dy`** = rendered phaseCorrelate
vertical-shake RMS (px @640, ↓ = steadier / less bob *amplitude*) and **`accel`** = angular
acceleration RMS (°/s², telemetry, ↓ = smoother path; the clean smoothness discriminator, §8f).
All offline, smoothness 0.5, 16:9, adaptive zoom max 130 %. `dy` needs a render (`—` = not rendered
on that clip); `accel` is computed from the smoothed quaternions (available for all).

| config | run0001 `dy`·`accel` | run0002 `dy`·`accel` | bike0005 `dy`·`accel` | verdict |
|---|---|---|---|---|
| default (Gyroflow) | 0.675 · 49 | 1.369 · 93 | 0.320 · 17 | golden baseline |
| **DCR (`--enhanced`)** | **0.469** · 33 | **0.896** · 43 | 0.315 · 15 | **shipped default** — best `dy` |
| per-axis (yaw 0.9) | 0.508 · 31 | 1.152 · 42 | — · 9 | excluded (§8e) — clip-specific |
| DCR + per-axis @130 | 0.386 · — | 0.902 · — | — · — | unshippable — 8.6 % black border (§8e) |
| Gaussian σ0.4 | 0.712 · 29 | 1.050 · 32 | — · 20 | opt-in (§8g) |
| Gaussian σ0.5 | 0.596 · **20** | 0.988 · **24** | 0.325 · **13** | opt-in — best `accel` |
| L1 (match-default) | **0.356** · 18 | 0.825 · 32 | — · 9 | other branch; lowest `dy` on 0001, +crop |
| DCR-off + 1 s look-ahead | 0.675 · 49 | 1.372 · — | — · — | ≡ default (look-ahead alone ≠ shake) |
| DCR + 1 s look-ahead | 0.484 · — | 0.890 · — | — · — | ≡ DCR — real-time realizable (§7); 4:3 Δ ≤ 4 % on all 4 clips (§8o) |

**Two objectives, two winners:** **DCR minimises `dy` (bob amplitude)** — the "is it steady"
metric — and ships as the default. **Gaussian σ0.5 / L1 minimise `accel` (path smoothness)** but do
not match DCR on `dy` → they trade amplitude for smoothness (kept opt-in, §8g). On **bike** (smooth
footage) every config ties on `dy` (~0.31–0.33); only `accel` separates them. Once DCR is off,
1 s look-ahead does not change `dy` (bob rejection is the short time-constant, not the far future).
Sources: `SMOOTHING_RND.md` §8a (dy), §8f/§8g (accel), §8e (Tier-1 matrix); Gaussian/L1 from
branches `claude/gaussian-smoothing` / `claude/speed-bump-jolt-rnd`.

**Baseline — DJI in-camera (RockSteady).** DJI's own stabilized clip is the external reference. Two
caveats make it a *separate* comparison rather than a row above: (1) DJI outputs **4:3** while our
configs render **16:9** — comparing our 16:9 `dy` to DJI's 4:3 flatters us (the 16:9 crop discards
the high-residual sensor periphery, §6), so the fair `dy` comparison re-renders our DCR at 4:3
(`--keep-sensor`); (2) DJI bakes stabilization into the pixels and exports **no** smoothed camera
path, so the telemetry `accel` metric **cannot** be computed for it (`dy` from the video is all that
is measurable). Matched same-scene pairs, FOV-matched 4:3, `dy` RMS (px @640):

| framing-matched (4:3) | run0001 `dy` | run0002 `dy` | bike0005 `dy` | `accel` |
|---|---:|---:|---:|---:|
| our **DCR** (4:3) | 0.491 | 0.962 | 0.391 | (see 16:9 table) |
| **DJI in-camera** (4:3) | 1.940 | 3.842 | 0.311 | n/a (no exported path) |
| ratio (DJI / ours) | 3.9× | 4.0× | 0.79× | — |

→ **vs DJI, activity-dependent:** our DCR is **~4× steadier on running** (violent bob), DJI **~1.3×
steadier on smooth biking** — and that bike gap is entirely frame-**periphery** residual (distortion
/ rolling-shutter at the 4:3 edges), not the smoothing; in the center band we tie (§6). Note our 4:3
`dy` (0.49/0.96/0.39) is higher than the 16:9 `dy` in the table above (0.47/0.90/0.32) precisely
because 4:3 includes that periphery. Detail: §6, `figures/dji_headtohead_summary.png`.

## 4. Black borders & zoom

- **Black borders come only from the `max_zoom` (130 %) clamp.** The adaptive-zoom envelope tracks
  the per-frame minimum FOV, so a border is forced only when the instantaneous required zoom
  exceeds 1.30. `default`/`DCR` never breach meaningfully (peak req ~1.2–1.3) → geometrically ~zero
  black border. The near-black pixel counter over-reports (dark scene content + 1–2 px warp edge);
  the required-vs-applied-zoom curve is the ground truth. → `SMOOTHING_RND.md` §8b.
- **Adaptive zoom beats static, either way you set it:** at equal average crop a static zoom
  black-borders ~2 % of frames; at equal (zero) black border it costs +20…48 % crop everywhere.
  → §8c.
- **Per config (all five smoothers):** dynamic zoom absorbs every config's crop-demand shape (~0 %
  black border for all); static exposes it, oppositely for different filter shapes — L1's hard crop
  box is *capped* (cheapest static-zero-BB +10–13 %, but worst at equal crop, 7–8 % BB) while the
  non-adaptive Gaussian is *peaky* (priciest static-zero-BB, +73 % on run 0001). Dynamic zoom
  matters most for exactly the aggressive configs that remove the most shake. → §8c′,
  `figures/dynamic_vs_static_zoom_blackborder.png`.
- **FOV look-ahead (`--zoom-look-ahead`, real-time builds):** 1 s of future does **not** reduce
  black borders (min-tracking already gives 0 in every mode) — it removes the causal zoom *pops*:
  max per-frame zoom jump 0.07–0.26 (causal) → 0.02–0.03 (1 s), mean crop back to offline levels.
  Render-confirmed: `dy` with 1 s look-ahead matches offline (0.683 vs 0.675; 1.372 vs 1.369),
  causal is slightly worse (0.871, 1.510). → §8h, `figures/zoom_lookahead_causal_vs_1s.png`.

## 5. Tier-1 decision — the full matrix

Rendered `dy` + black border, default / DCR / per-axis / DCR+per-axis (mz 130 and floored 170),
both clips:

| config | 0001 dy | 0002 dy | bb max % |
|---|---:|---:|---:|
| default | 0.675 | 1.369 | ~1.2 |
| **DCR** | **0.469 (−31 %)** | **0.896 (−35 %)** | ~1.2 |
| per-axis y0.9 | 0.508 | 1.152 | 1.5–1.6 |
| DCR+per-axis @130 | 0.386 | 0.902 | **8.6–8.8** ✗ |
| DCR+per-axis @170 | 0.417 | 1.066 (worse than DCR) | 0.3–0.6 |

DCR is the clean win on **both** clips. DCR+per-axis is unshippable at default zoom, and even with
the black-border floor it costs more crop and loses to DCR on the harder clip. → `SMOOTHING_RND.md`
§8d–8e, `figures/tier1_stack_eval.png`.

## 6. Head-to-head vs DJI in-camera (matched scene, FOV-matched 4:3)

Rendered our DCR at 4:3 (`--keep-sensor`) to match DJI's framing; `dy` RMS (px@640), with a
top/center/bottom band breakdown:

| pair | our DCR 4:3 (full) | DJI 4:3 (full) | verdict |
|---|---:|---:|---|
| run L0001 ↔ DJI0002 | 0.491 | 1.940 | **we're 4.0× steadier** |
| run L0002 ↔ DJI0004 | 0.962 | 3.842 | **we're 4.0× steadier** |
| bike 0005 ↔ DJI0004 | 0.391 | 0.311 | DJI 1.26× steadier |

**The aspect-ratio / FOV confound, resolved.** Our default deliverable is 16:9; DJI's is 4:3.
Comparing our 16:9 to DJI 4:3 flattered us (the 16:9 crop discards the high-residual sensor
periphery). Re-rendered at matched 4:3:
- On **run**, our 16:9→4:3 shift is only ~2 % (0.481→0.491) and we still beat DJI ~4× in **every
  band** — a FOV/periphery artifact cannot explain a 4× gap, so the running win is real.
- On **bike**, overall shake is tiny (~0.3 px), so the periphery becomes the dominant relative
  term: our 16:9→4:3 jumps +24 % (0.315→0.391) while DJI's full-4:3 stays clean (0.311). The gap
  is the **frame periphery** (fisheye-distortion / rolling-shutter residual), not the smoothing —
  in the **center band** we and DJI are on par.

**Interpretation:** our rotational smoothing (DCR) is excellent — it dominates DJI on the hard
(violent-bob) case and ties in the center on the easy case. DJI's remaining edge on smooth footage
is edge/periphery treatment, plus a possible wider-FOV (lighter distortion-correction) advantage we
could not fully measure (DJI's output FOV is not in telemetry).

**Spectral note (why DJI's residual can *look* cleaner):** DJI's run residual is one dominant
periodic component — the running cadence it did not remove (peak 1.4–1.9 Hz, in the worst bob
band) — which reads as a tidy sinusoid; ours removed that peak, leaving small broadband residual
that *reads* as noise. Band-split analysis shows ours is lower in **every** band (incl. 4–15 Hz
jitter, 2–3×) with **5× lower** frame-to-frame roughness — the "cleaner DJI" impression is a
plot-scale + spectral-peakiness illusion. → `SMOOTHING_RND.md` §8i,
`figures/dy_spectrum_ours_vs_dji.png`.

**What DJI's filter actually is (§8j, confirmed by experiment):** a **bounded-deviation
follower** — on violent footage its attenuation collapses frequency-flat (~2× in every band), the
signature of a crop budget saturating, not a differently-tuned low-pass (no τ/smoothness setting
reproduces it). Implemented `--deviation-clamp B` (clamp smoothed path to B° from raw; off by
default): a clamp-5° render matches DJI within ~15–20 % on every measure. The hard clamp leaves
saturation "burrs" (raw HF passes through while riding the box); the **soft variant**
(`--deviation-clamp-soft B`, smooth box center τ0.02 + tanh saturation, §8j-4) fixes them and
lands **exactly on DJI's amplitude (dy 5.944 vs 5.951) with 39 % less HF jitter and 37 % less
roughness than DJI itself**. Both clamps still distort the waveform (per-sample saturation clips
the cadence sinusoid → 2nd-harmonic 0.27–0.37 vs DJI's 0.039 — the "split peaks", §8j-5).
**The principled fix is joint optimization: L1 with a small box** (`--smoothing l1
--l1-deviation B`, this branch, §8j-6/7) — smooth+budget solved as ONE convex problem gives
piecewise-polynomial arcs that ride the box tangentially: harmonic 0.043–0.062 = DJI-clean.
Tuned to the crop budget (**box 12° = largest box inside the 130 % clamp**), **L1 beats DJI on
every measure**: dy 5.20 vs 5.95 (−12 %), every band lower, roughness −30 %, zero border.
Full 0004 ladder: DCR 1.95 < EMA 4.04 < **L1 box12 5.20** < soft clamp 5.94 ≈ DJI 5.95 < hard
clamp 6.95. Final map: **DCR/EMA for max steadiness (unbounded), L1-with-box for the
guaranteed-crop class (beats DJI), soft clamp as the cheap real-time approximation** (~µs vs
L1's ~2 s/66 s clip). Next: derive the box from `max_zoom` automatically (self-tuning
crop-budget L1). → `SMOOTHING_RND.md` §8j–§8j-8, `figures/all_bounded_experiments_dy*.png`.

**Independent replication — fresh footage (2026-07-08 shoot).** Two new matched pairs
(`dji6_L/20260708` 0003/0004, stab-off, our DCR 16:9 render vs `dji6_R/20260708` 0005/0006,
DJI in-camera 4:3; durations match to <1 s). dy RMS px @640, band split, roughness:

| pair | series | dy RMS | <1 Hz | 1–4 Hz | 4–15 Hz | roughness |
|---|---|---:|---:|---:|---:|---:|
| A (277 s, calm) | original | 1.511 | 0.468 | 0.672 | 1.269 | 2.207 |
| A | **our DCR** | **0.327** | 0.313 | **0.078** | **0.055** | **0.096** |
| A | DJI in-camera | 0.334 | 0.308 | 0.104 | 0.079 | 0.134 |
| B (65 s, violent) | original | 12.595 | 2.861 | 10.398 | 6.505 | 10.643 |
| B | **our DCR** | **1.953** | 1.365 | **1.282** | **0.554** | **0.975** |
| B | DJI in-camera | 5.951 | 2.267 | 4.564 | 3.073 | 5.300 |

→ Replicates every prior conclusion on unseen footage: on the **violent** clip (the most violent
measured, original 12.6 px) our DCR is **3.0× steadier overall** (shake removed −84 % vs DJI's
−53 %), 3.6× in the bob band, **5.5× in 4–15 Hz jitter**, 5.4× lower roughness — and this time DJI
fails to contain even the high frequencies, so the gap is robust to the 16:9-vs-4:3 caveat (~2 %
on violent clips). On the **calm** clip the two tie (0.327 vs 0.334) with ours slightly better in
every band — as-delivered framing; at matched 4:3 this could revert to a DJI edge (periphery, §6).
Our residual again concentrates <1 Hz (the §3 translational parallax); DJI's stays bob-dominated
(§8i). Outputs: `dji6_L/20260708/000{3,4}_D_cpp_dcr.mp4`.

## 7. What's confirmed & what's next

**Confirmed and landed:** DCR (`--enhanced`) as the Tier-1 stabilization enhancement — parity-safe,
−31…35 % vertical shake, ~4× better than DJI on running, on par (center) on biking.

**Next (priority order — mirrors `TODO.md` §0):**
1. **Velocity-adaptive Gaussian** (~1 day, best value): the fixed-σ Gaussian is done and evaluated
   (§3 — lowest accel, but loses to DCR on `dy`); drive its per-frame σ from the velocity ratio
   (± DCR gate) `default_algo` already computes, to chase DCR's amplitude *and* the Gaussian's
   smoothness at once. **Acceptance gate:** `dy` ≤ DCR AND accel < DCR on run+bike, black border not
   increased (`vertical_flow_compare.py` + `angular_derivatives_compare.py` + `zoom_vs_maxzoom.py`).
   Passing → new `--enhanced`.
2. **Frame-periphery residual** — the only place DJI leads (§6). Investigate per-row rolling-shutter
   and fisheye-distortion accuracy at the sensor edges (band analysis localizes it to bottom/edge).
3. **Branch consolidation** — merge `claude/gaussian-smoothing` (verified low-risk); **rebase**
   `claude/speed-bump-jolt-rnd` (real CLI conflicts + missing infra; unify its API to
   `(quats, duration_ms, params)` on the way).
4. **Translation-domain stabilization** (`SMOOTHING_RND.md` §3) — the visible "running float" is
   translational parallax no rotational smoother can remove; the largest remaining headroom,
   a separate larger effort (start with a design doc).
5. **Native 10-bit decode + DJI parse** (`TODO.md` §1/§3) — port-completeness (pixel fidelity),
   removes the bridge.

Reproduce everything: [`figures/README.md`](figures/README.md).
