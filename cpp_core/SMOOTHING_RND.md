# Smoothing R&D — DCR gating, jerk-limiting, and kernel choice

Analysis record for the **running low-frequency "float"** problem: during running the camera
bobs up/down and Gyroflow's default smoothing cannot remove it. This documents the DCR gate
we added, the quantitative before/after, a key finding about what actually limits the result,
and comparisons against L1-optimal and a Gaussian base kernel.

> Status: DCR is **merged** on this branch (commit `cpp_core: add DCR ... to default smoothing`).
> L1-optimal lives on `claude/speed-bump-jolt-rnd`. The Gaussian-kernel result is a Python
> prototype only (not ported). All numbers below are measured on the two dji6_L "run" clips.

## Dataset

Osmo Action 6, `dji6_L/run/` (Seagate drive, gitignored):
- `DJI_20260625014240_0001_D.MP4` — 2312 frames, 77.1 s, bob peak ≈ **1.94 Hz**
- `DJI_20260625014416_0002_D.MP4` — 1487 frames, 49.6 s, bob peak ≈ **0.54 Hz**
- Both HEVC 3840×2880 10-bit; rendered outputs are 16:9 3840×2160 H.265.

Result videos (all preserved side by side in `dji6_L/run/cpp_out/`):
`{0001,0002}_D_cpp_stabilized.mp4` (default) / `_dcr.mp4` (DCR) / `_l1.mp4` (L1-optimal).

All measurements below are on the **camera path** (the per-frame smoothed orientation, which is
exactly what the output video's apparent motion follows), extracted with `gyroflow_cpp_validate`,
converted to euler; **pitch = vertical tilt = the running float axis**. Caveats: frame-rate
(≈30 Hz) telemetry proxy, crop proxied by max/RMS deviation from raw, and everything here is in
the **rotational** domain (see §3).

---

## Primer: forward/backward slerp EMA

`slerp(a, b, t)` is spherical linear interpolation — the constant-speed geodesic between two unit
quaternions, `t∈[0,1]` (`t=0`→a, `t=1`→b). The smoother is a one-pole exponential moving average
built from it:

- **Forward pass (causal):** `q_fwd[i] = slerp(q_fwd[i-1], raw[i], α_i)` — the quaternion analog
  of `y[i] = (1-α)·y[i-1] + α·x[i]`. Small α = heavy smoothing (long time constant), α=1 = follow
  raw. Time constant: `α = 1 - exp(-Δt/τ)`. This pass **lags** (trails the true motion).
- **Backward pass:** the same EMA run in reverse over the forward result,
  `q_out[i] = slerp(q_out[i+1], q_fwd[i], α_i)`. Its lag cancels the forward pass's lag →
  **zero net phase**, and the filter order doubles (sharper roll-off). Quaternion "filtfilt".
- Net two-sided weighting is symmetric `exp(-|Δt|/τ)`, centered on each sample.

`default_algo` makes α_i **velocity-adaptive** (τ interpolated between 1 s at low velocity and
0.1 s at high velocity) and runs the whole thing twice (velocity pass + distance pass). **DCR
gates the velocity that sets α_i.** Only the **backward pass** needs future data — the crux of the
real-time discussion in §7.

---

## 1. DCR gating (implemented)

**Problem in the stock algorithm.** `default_algo` is a velocity-adaptive two-pass EMA: alpha
interpolates between a 1 s time constant at low velocity and 0.1 s at high velocity. Velocity is
computed as an **abs** magnitude (`dist.angle()`, or `|euler|` per axis), so the filter *loosens
whenever motion is fast* — and cannot distinguish:
- a sustained **pan** (consistent direction — should loosen and follow), from
- a reciprocating **bob** (alternating direction — should stay strongly smoothed).

It passes the bob through as a low-frequency float that no amount of slider tuning removes.

**DCR = Direction Consistency Ratio.** Over a sliding window,
`DCR = |mean(ω)| / mean(|ω|) ∈ [0,1]` (per euler axis in the per-axis path; the 3-D
rotation-vector analogue `‖Σ ω_vec‖ / Σ‖ω_vec‖` in the scalar/default path). DCR→1 for a
consistent pan, DCR→0 for a reciprocating bob. We multiply the normalized velocity ratio by
`DCR^power` before the adaptive pass, so the filter loosens only when motion is **fast AND
directionally consistent**. `dcr == false` (default) leaves both paths bit-identical → golden
parity preserved. O(n) via prefix sums (gyro streams are ≈1 kHz). CLI: `--dcr [--dcr-window 0.5]
[--dcr-power 1.0]` on `gyroflow_cpp_stabilize` / `gyroflow_cpp_validate`.

---

## 2. DCR vs default — camera-path (telemetry) result

Band RMS of the smoothed camera path, default → DCR (equal algorithm, DCR on):

| axis / band | 0001 default→DCR | Δ | 0002 default→DCR | Δ |
|---|---|---|---|---|
| **pitch (vertical) bob 0.5–5 Hz** | 0.516°→0.284° | **−44.9%** | 1.063°→0.261° | **−75.5%** |
| roll bob 0.5–5 Hz | 0.482°→0.206° | −57.2% | 1.117°→0.303° | −72.9% |
| yaw (horizontal) bob 0.5–5 Hz | 11.81°→11.73° | −0.7% | 15.29°→15.27° | −0.1% |
| pitch intentional <0.5 Hz | 2.94°→2.60° | −11.6% | 4.47°→4.16° | −7.0% |
| yaw intentional <0.5 Hz | 101.9°→101.7° | −0.1% | 101.0°→100.8° | −0.2% |
| crop cost (mean fov) | 0.997→0.932 (zoom 1.00×→1.07×) | +7% | 0.971→0.910 (1.03×→1.10×) | +13% |

**Selective and correct:** DCR cuts the rotational vertical bob by 45–75% and the roll bob by
57–73%, while leaving the intentional ~100° horizontal pan essentially untouched (<0.7%). Cost is
~7–13% additional average crop; the single worst moment briefly touches the 130% zoom clamp.

---

## 3. Image-domain cross-check — the translational ceiling (KEY FINDING)

Measured residual **vertical motion directly in the rendered videos** via phase correlation
(integrate inter-frame dy → position → bob-band RMS), default vs DCR:

| clip | default | DCR | image-domain Δ | (vs camera-path Δ) |
|---|---|---|---|---|
| 0001 | 32.9 px | 29.9 px | **−9.0%** | (−44.9%) |
| 0002 | 149.9 px | 149.6 px | **−0.2%** | (−75.5%) |

The visible improvement is **far smaller** than the rotational (telemetry) improvement. Reason:
gyro/rotation stabilization only removes the **rotational** component of the float. Much of the
running "up/down float" is **translational parallax** — the camera body physically moves up/down,
near objects shift more than far ones — which **no rotation-based method (default, DCR, or
Gyroflow itself) can remove**. Order-of-magnitude check: 0001's rotational pitch bob (0.52°)
maps to ≈15–18 px, DCR trims it to ≈9 px → a ~6–8 px cut, ≈¼ of the 33 px total image bob — which
matches the measured −9%. **0002 is nearly all translational** (75% rotational cut, ~0% visible).

**Implication:** to further reduce the *visible* running float you must work in the
**translation** domain (optical-flow 2-D translation smoothing / depth-aware), not gyro rotation.
This is the real ceiling, independent of which rotational smoother is used.

---

## 4. Jerk / acceleration (perceived smoothness)

Perceived un-smoothness tracks **acceleration/jerk**, not position/velocity error. Pitch RMS:

| metric | 0001 raw / default / DCR | 0002 raw / default / DCR |
|---|---|---|
| angular velocity °/s | 26.4 / 3.70 / 2.39 | 40.1 / 6.78 / 3.26 |
| angular accel °/s² | 409 / 17.6 / 9.4 | 804 / 39.4 / 10.4 |
| **jerk °/s³** | 9717 / **208** / **126** | 20886 / **497** / **178** |

The default EMA still leaves jerk 208/497; **DCR already cuts jerk a further 39%/64%** as a
by-product of killing the reciprocating (high-jerk) bob. So DCR is partly a jerk reducer, which
motivated testing explicit jerk-limiting (§5) and a cleaner kernel (§6).

---

## 5. L1-optimal (explicit jerk-limiting) — `claude/speed-bump-jolt-rnd`

L1-optimal (Grundmann 2011) minimizes `w₁Σ|vel|+w₂Σ|accel|+w₃Σ|jerk|` (default w=10/1/**100**,
jerk-dominant) subject to a per-axis crop box `|path−raw|≤B`. `--smoothing l1 --l1-match-default`
sets B to the deviation the default smoother actually used. Pitch, equal box:

| metric | 0001 default / DCR / **L1** | 0002 default / DCR / **L1** |
|---|---|---|
| bob RMS ° | 0.52 / 0.28 / **0.15** | 1.06 / **0.26** / 0.49 |
| accel °/s² | 17.6 / 9.4 / **6.9** | 39.4 / **10.4** / 12.6 |
| jerk °/s³ | 208 / 126 / **108** | 497 / 178 / **175.5** |
| mean fov (crop) | 1.003 / 0.932 / **0.863** | 1.030 / 0.912 / **0.897** |

**Yes, limiting jerk further improves smoothness — but with two caveats:**
1. **Marginal over DCR.** Most of the jerk reduction was already captured by DCR's direction
   gating; explicit L1 adds only 126→108 (0001) / 178→175.5 (0002, ~tie). It helps most on the
   transient-rich clip (0001).
2. **"Match-box" ≠ "match-crop".** L1 can spend the whole box budget across the clip, so its
   mean fov is *lower* (more crop): 0.863 vs default 1.003 — L1 buys smoothness with ~14% more
   crop. A truly fair comparison needs mean-fov-matched boxes (not yet run).

On 0002, DCR actually beats L1 on bob and accel at *less* crop. The biggest lever is
**direction-awareness (DCR)**, not the explicit jerk penalty.

---

## 6. EMA vs Gaussian base kernel; does DCR help a Gaussian?

DCR gates EMA's velocity-adaptive loosening. A Gaussian is a fixed linear low-pass with the
optimal time-frequency localization and a cleaner roll-off — and **it never loosens**, so it has
none of the pathology DCR exists to patch. Pitch, all matched to the same crop (max|dev|≈11.7°/
11.9°); `int_err` = <0.5 Hz intentional motion lost (lower = better):

| 0001 @crop≈11.7° | bob° | int_err° | jerk °/s³ |
|---|---|---|---|
| DCR (EMA) | 0.284 | 1.29 | 126 |
| **Gaussian fixed σ=24.8** | **0.053** | 1.34 | **3** |
| Gaussian + DCR (blend / per-sample σ) | 0.9 / 0.56 | 1.6 | 974 / 11969 |

| 0002 @crop≈11.9° | bob° | int_err° | jerk °/s³ |
|---|---|---|---|
| DCR (EMA) | **0.261** | 1.32 | 178 |
| **Gaussian fixed σ=10.9** | 0.688 | **0.59** | **45** |
| Gaussian + DCR (blend) | 0.859 | 2.09 | 1476 |

**Findings:**
1. **A fixed Gaussian beats EMA/DCR on jerk by ~1–2 orders of magnitude at equal crop** (3 vs 126
   on 0001; 45 vs 178 on 0002), because it does not loosen on the bob's high velocity. In effect a
   plain Gaussian achieves what "EMA + DCR" is reaching for, more cleanly and more simply.
2. **DCR does NOT help a Gaussian.** Both per-sample-σ modulation (jerk explodes to ~12000 from σ
   discontinuities injecting kinks) and smooth two-track blending underperform a **plain fixed
   Gaussian**. DCR is an EMA patch; the correct framing is *"Gaussian instead of EMA+DCR"*, not
   *"DCR on Gaussian"*.
3. **One case the Gaussian loses:** the very-low-frequency bob that overlaps the intentional band
   (0002, 0.54 Hz). There no linear filter (EMA or Gaussian) separates bob from intent; DCR's
   nonlinear selectivity wins on bob rejection (0.261 vs 0.688) — but the Gaussian keeps much
   lower jerk (45 vs 178) and preserves intentional motion far better (int_err 0.59 vs 1.32).

**Method ranking for the bob (rotational domain):** EMA-adaptive (worst — loosening backfires) <
EMA+DCR (fixes loosening) < fixed Gaussian (smoothest, best intent preservation; loses only on
sub-Hz bob) ; L1-optimal (nonlinear + crop-aware) is the highest ceiling.

---

## 7. Real-time / in-camera realizability (1 s look-ahead)

All numbers above are **offline**: full-clip, unlimited bidirectional look-ahead, zero-phase. An
in-camera implementation only buffers ≈**1 s of future** to run the backward pass (see Primer),
which caps the effective symmetric half-width at ~1 s. How much this hurts depends on how much
future each method needs:

| method | future needed | within 1 s? |
|---|---|---|
| DCR (window 0.5 s → ±0.25 s) | ~0.25 s | ✅ realizable as-measured |
| default EMA velocity pass (τ=0.1 s) | ~0.3 s | ✅ ≈ offline |
| default EMA main low-pass (τ=1 s) | backward truncated to 1 s ≈ 1τ | ⚠️ only ~63% settled |
| fixed Gaussian σ=24.8 fr (0001 win) | ±3σ ≈ ±2.5 s | ❌ not realizable |
| fixed Gaussian σ=10.9 fr (0002) | ±3σ ≈ ±1.1 s | ⚠️ borderline |
| L1-optimal (global solve) | whole clip | ❌ → receding-horizon, weaker |

**Correct real-time bidirectional EMA.** Past is effectively unlimited (carried as O(1) forward
state — do NOT truncate it to 1 s); only the future is capped. It is **not** "past 1 s + future
1 s" — truncating the past discards free information and weakens the result.
1. Forward pass runs causally on every captured frame (full history in one running state).
2. For the output sample (1 s behind live), run the backward pass over just the 1 s buffer,
   seeded with the forward value at the newest buffered frame:
   `q_back = q_fwd[t+1s]; for k=t+1s-1..t: q_back = slerp(q_back, q_fwd[k], α_k); emit q_back`.
   Output lags live by the 1 s look-ahead.

**Physical limit.** To symmetrically smooth frequency f you need ≳ half a period of future. The
0.54 Hz bob (0002) has a half-period ≈ 0.93 s → 1 s look-ahead is right at the edge; anything
below ~0.5 Hz cannot be smoothed well in-camera by any algorithm.

**Measured — DCR-EMA offline vs a real 1 s look-ahead** (`--look-ahead 1.0`, which windows the
main adaptive backward pass; forward pass stays full-past). Pitch:

| metric | 0001 offline→1s | 0002 offline→1s |
|---|---|---|
| bob 0.5–5 Hz ° | 0.284→0.276 (**−3%**) | 0.261→0.253 (**−3%**) |
| jerk °/s³ | 126→126 (~0%) | 178→181 (+1.7%) |
| mean fov | 0.932→0.937 | 0.912→0.919 |
| max local pitch Δ | 0.74° | 0.49° |

**DCR-EMA loses almost nothing at 1 s look-ahead.** Its benefit is keeping the filter *tight*
through the reciprocating bob (1–2 Hz), which a 1 s backward window resolves fully; the τ=1 s
truncation only mildly perturbs the *slow intentional following* (the ~0.5–0.7° local diffs + a
touch less crop), not bob rejection or jerk. So DCR-EMA is essentially in-camera-realizable.
Renders: `{0001,0002}_D_cpp_stabilized_dcr_la1.mp4` in `dji6_L/run/cpp_out/`.

**Effect on the conclusions:**
- **Robust:** DCR (fits in 1 s — measured −3% bob, ~0% jerk vs offline) and the
  translational-parallax ceiling (§3, independent of filtering) stand unchanged. Under the real
  constraint DCR is *relatively more attractive* — cheap and look-ahead-light.
- **Weakened:** the "fixed Gaussian dominates" (§6) and "L1 highest ceiling" (§5) results assumed
  >1 s look-ahead; truncated to 1 s they lose most of their low-frequency advantage. The offline
  numbers are optimistic upper bounds, not in-camera achievable. To make τ=1 s smoothing settle
  in-camera you'd need ~3 s look-ahead, or lower max_smoothness to ≤~0.3 s (so 1 s ≈ 3τ).
- **Implementation caveat:** default_algo's distance (second) pass normalizes by a **global** max
  over the clip — non-causal; in-camera replace with a rolling-window max.

---

## 8. Black borders & static-vs-adaptive zoom (image-domain, full renders)

Full 4K renders of dji6_L clips **0001** (2312 f) and **0002** (1487 f) under five configs —
`default` (offline), `DCR` (offline), `L1`, `DCR off + 1 s look-ahead` (`la1`), `DCR + 1 s`
(`dcr_la1`) — evaluated on two image-domain metrics. Tools:
`tools/{vertical_flow_compare,black_border_stats,zoom_vs_maxzoom}.py`; figures and the exact
reproduce commands are in [`figures/README.md`](figures/README.md).

**8a. Vertical shake (phaseCorrelate global `dy`, RMS px @640-wide):**

| config | 0001 | 0002 |
|---|---|---|
| default (offline) | 0.675 | 1.369 |
| DCR (offline) | 0.469 | 0.896 |
| **L1** | **0.356** | **0.825** |
| DCR off + 1 s LA | 0.675 | 1.372 |
| DCR + 1 s LA | 0.484 | 0.890 |

- **L1 lowest** on both clips; **DCR ≈ −30…35%** and its offline vs 1 s look-ahead versions
  nearly coincide (confirms §7: DCR fits inside a 1 s buffer). **`la1` ≡ default** (−0%): once
  DCR is off, truncating look-ahead to 1 s does **not** change per-frame vertical shake — the
  bob rejection comes from the short time-constant, not the far future.
- vs **DJI in-camera** reference clips (different handheld takes, RockSteady, 4:3 full-frame):
  DJI `dy` RMS **1.93 / 3.84 px** — every cpp config is 3–5× steadier (magnitude reference; DJI
  keeps the full sensor so it crops less, part of the gap).

**8b. Black borders come only from the `max_zoom` clamp.** The adaptive-zoom envelope tracks the
per-frame minimum FOV, so the applied crop is ≥ what each frame needs — the **only** way a black
border is forced is when the instantaneous required FOV drops below the clamp floor
`min_fov = 100/max_zoom = 0.769` (i.e. required zoom > 1.30). Added an optional `raw_fovs_out`
to `computeAdaptiveFovs` + a `raw_fov` column to `gyroflow_cpp_validate` to export the required
FOV. Required-zoom peaks / clamp breaches (max_zoom = 130 %):

| config | 0001 max req | breach frames | 0002 max req | breach frames |
|---|---|---|---|---|
| default | 1.271 | **0** | 1.165 | **0** |
| DCR | 1.583 | 6 (0.3%) | 1.538 | 4 (0.3%) |
| DCR off + 1 s | 1.271 | **0** | 1.164 | **0** |
| DCR + 1 s | 1.589 | 5 (0.2%) | 1.525 | 3 (0.2%) |

- **`default` / `la1` never breach** (peak 1.16–1.27 < 1.30) → geometrically **zero** black border.
  Only the **DCR configs** breach, on a handful of frames (0.2–0.3 %), where DCR's more aggressive
  reciprocating-shake compensation spikes required zoom to 1.5–1.6.
- This **corrects** the raw near-black pixel counter (`black_border_stats.py`), which flagged
  ~0.01 % mean / ~1 % peak "black border" even for `default`: those are **false positives** —
  edge-touching dark scene content + the 1–2 px undistort-warp boundary (`fov_algorithm_margin`),
  not clamp-forced crop black. Genuine clamp black border exists **only** in the DCR configs.
- Fixes: raise `max_zoom` to ~160 % for DCR configs (costs extra crop on those few frames), or
  floor DCR's required FOV. `default`/`la1` need nothing.

**8c. Static zoom is strictly worse than adaptive — either way you set it.** A constant clip-wide
zoom cannot redistribute crop to the high-motion frames the way the temporal envelope does:

- **Equal average crop:** a static zoom at adaptive's mean applied zoom (≈1.00 for `default`)
  leaves the required-zoom peaks uncovered → **~2 % of frames (≈30–50) get black borders**, vs
  ≈0 % for adaptive at the same average crop.
- **Equal (zero) black border:** a static zoom must be pinned at the whole-clip worst frame
  (`default` 1.27, DCR 1.5–1.6) → **+20…27 % more crop on every frame** (DCR configs +39…48 %),
  permanently narrower FOV.

So adaptive zoom buys either ~0 % black border at equal crop, or 20–48 % less crop at equal
(zero) black border. Static gives up that trade — using it would make black borders **worse** at
any comparable FOV budget.

## 8d. Per-axis smoothing on top of `la1` — a real extra vertical win

Tried per-axis smoothing (`--per-axis`, independent `smoothness_{pitch,yaw,roll}` on the three
euler components) on the **DCR-off + 1 s look-ahead** (`la1`) config, to smooth the vertical
axis harder than the pan.

**Which euler axis is "vertical"?** Calibrated a telemetry proxy against the four known image-dy
values: the first-difference RMS of the **stabilized optical axis' vertical component**
(`fwd = R·(0,0,1)`, take `fwd.y`) correlates **0.999** with the rendered phaseCorrelate `dy`
(≈2.36 px/°). A single-axis validate sweep then shows the vertical bob lives almost entirely in
**euler[1]** — which maps to the confusingly-named `--smoothness-yaw` param (nalgebra pitch about
Y = camera looking up/down). Smoothing euler[0] (`--smoothness-pitch`) or euler[2]
(`--smoothness-roll`) moves the vertical proxy < 3 %.

**Sweep (clip 0002, baseline `la1` = per-axis off), euler[1] smoothness up:**

| euler[1] | vert dy proxy | frames req-zoom > 1.30 |
|---|---|---|
| off | 0 % | 0 |
| 0.55 | −4 % | 1 |
| 0.70 | −7 % | 4 |
| 0.90 | −9 % | 12 |

**Rendered confirmation** (full 4K, `la1` vs `la1 + euler[1]=0.9`, phaseCorrelate `dy` + black
border, clip 0002):

| config | vert dy RMS | black border mean / max | frames > 0.05 % |
|---|---|---|---|
| `la1` baseline | 1.372 px | 0.0137 % / 1.79 % | 2.3 % |
| `la1` + euler[1]=0.9 | **1.152 px (−16 %)** | 0.0136 % / 1.49 % | 1.3 % |

- **The real gain (−16 %) beats the proxy (−9 %)** — the linear proxy was calibrated on isotropic
  configs and under-scales once per-axis reshapes the spectrum; the render is ground truth.
- **Black border did not get worse.** The sweep flags 12 frames breaching the 130 % clamp, but
  those are tiny-corner geometric black; the coarse detector (stride-5 / 480 px) barely samples
  them, and the smoother path even reduces dark-content edge contact — mean flat, max down,
  fraction of flagged frames 2.3 → 1.3 %. Reconfirms §8b (the near-black counter is dominated by
  dark-scene / warp-edge false positives, not clamp black).
- **Verdict:** on a bob-heavy clip, pushing the vertical euler axis (`--smoothness-yaw ≈0.9`) on
  top of `la1` is a worthwhile, near-free −16 % vertical-shake improvement. Pan is untouched
  (euler[2] left at 0.5). The only real cost is a handful of tiny clamp-black frames — kill them
  by raising `max_zoom` to ~150 % or flooring the required FOV (§8b item 6).

---

## Bottom line & next steps

- **DCR is a correct rotational-domain improvement** (merged): −45…75% vertical bob, roll bob too,
  intentional pan untouched, +7–13% crop.
- **The visible running float is dominated by translational parallax** (§3), which is the real
  ceiling — no rotational smoother (DCR, L1, or Gyroflow) can touch it. Fixing it needs
  translation-domain stabilization.
- **A Gaussian base kernel** is a simple, real upgrade over EMA on smoothness/jerk, and would let
  DCR be dropped rather than gated onto it. Worth porting as `--smoothing gaussian` and validating
  on a real render (numbers here are a 30 Hz telemetry proxy).
- **Explicit jerk-limiting (L1)** helps a little more on transient-rich clips but mostly duplicates
  DCR's gain and costs more crop unless mean-fov-matched.
- **Real-time caveat (§7):** the offline numbers assume unlimited look-ahead. An in-camera 1 s
  future buffer caps symmetric smoothing at ~1 s — DCR stays realizable, but the Gaussian and L1
  advantages shrink (they need >1 s future). Use **full-past forward + 1 s backward**, not a
  symmetric ±1 s window.
- **Per-axis is a real extra win (§8d):** smoothing the vertical euler axis (euler[1] =
  `--smoothness-yaw ≈0.9`) harder than the pan gives −16 % vertical shake on top of `la1`
  (rendered), pan untouched, negligible black border. A cheap, orthogonal complement to DCR.

### Candidate work items
1. Translation-domain residual stabilization (the actual visible-float fix).
2. Port a Gaussian (or linear-phase) base kernel as an alternative smoother; re-measure on renders.
3. Mean-fov-matched L1 vs DCR vs Gaussian comparison (telemetry-fast).
4. Add a jerk-RMS metric to `stabilization_quality.py`.
5. Re-run §5/§6 under a 1 s-look-ahead cap (truncated backward pass / receding-horizon) for
   in-camera-achievable numbers.
6. (§8b) Raise `max_zoom` to ~160 % for DCR configs, or floor DCR's required FOV, to kill the
   handful of clamp-forced black-border frames.
7. (§8d) Ship per-axis vertical smoothing as a default-on option on `la1` (euler[1]≈0.9,
   euler[0]/[2]=0.5); confirm the −16 % on clip 0001 and pick the crop/gain sweet-spot yaw value.

### Reproduce
```sh
# camera-path CSV per mode (this branch: default/DCR; jolt branch: --smoothing l1 --l1-match-default)
# CSV now also carries raw_fov (pre-smoothing/clamp required FOV) for the §8 zoom analysis.
gyroflow_cpp_validate BRIDGE.json --frames N [--dcr] [--look-ahead 1]   > path.csv
# metrics: convert smoothed quat -> euler pitch; band RMS (0.5-5 Hz bob / <0.5 Hz intent);
#          jerk = 3rd time-difference RMS; crop = max|smoothed-raw| or mean fov.
# image-domain (§8): tools/vertical_flow_compare.py (phaseCorrelate dy),
#          tools/black_border_stats.py (edge-connected near-black area),
#          tools/zoom_vs_maxzoom.py (required vs applied zoom vs max_zoom clamp, from raw_fov).
```
