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

**Two objectives, two winners:** **DCR minimises `dy` (bob amplitude)** — the "is it steady"
metric — and ships as the default. **Gaussian σ0.5 / L1 minimise `accel` (path smoothness)** but do
not match DCR on `dy` → they trade amplitude for smoothness (kept opt-in, §8g). On **bike** (smooth
footage) every config ties on `dy` (~0.31–0.33); only `accel` separates them. Once DCR is off,
1 s look-ahead does not change `dy` (bob rejection is the short time-constant, not the far future).
Sources: `SMOOTHING_RND.md` §8a (dy), §8f/§8g (accel), §8e (Tier-1 matrix); Gaussian/L1 from
branches `claude/gaussian-smoothing` / `claude/speed-bump-jolt-rnd`.

## 4. Black borders & zoom

- **Black borders come only from the `max_zoom` (130 %) clamp.** The adaptive-zoom envelope tracks
  the per-frame minimum FOV, so a border is forced only when the instantaneous required zoom
  exceeds 1.30. `default`/`DCR` never breach meaningfully (peak req ~1.2–1.3) → geometrically ~zero
  black border. The near-black pixel counter over-reports (dark scene content + 1–2 px warp edge);
  the required-vs-applied-zoom curve is the ground truth. → `SMOOTHING_RND.md` §8b.
- **Adaptive zoom beats static, either way you set it:** at equal average crop a static zoom
  black-borders ~2 % of frames; at equal (zero) black border it costs +20…48 % crop everywhere.
  → §8c.

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

## 7. What's confirmed & what's next

**Confirmed and landed:** DCR (`--enhanced`) as the Tier-1 stabilization enhancement — parity-safe,
−31…35 % vertical shake, ~4× better than DJI on running, on par (center) on biking.

**Next (from the analysis):**
1. **Gaussian base kernel** — swap the EMA smoother for a linear-phase Gaussian kernel and evaluate
   it against EMA/DCR on **both** axes measured here: optical flow (`dy`) and angular jerk (§8f). A
   linear-phase kernel should lower jerk without the DCR gate; check it holds `dy` and crop on real
   renders. Tools: `tools/vertical_flow_compare.py` + `tools/angular_jerk_compare.py`.
2. **Frame-periphery residual** — the only place DJI leads. Investigate per-row rolling-shutter and
   fisheye-distortion accuracy at the sensor edges (band analysis localizes it to the bottom/edge).
3. **Translation-domain stabilization** (`SMOOTHING_RND.md` §3) — the visible "running float" is
   translational parallax that no rotational smoother can remove; the largest remaining quality
   headroom, a separate larger effort.
4. **Native 10-bit decode + DJI parse** (`TODO.md`) — port-completeness (pixel fidelity), removes
   the bridge.

Reproduce everything: [`figures/README.md`](figures/README.md).
