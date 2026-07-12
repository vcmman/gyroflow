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

**8a. Vertical shake (phaseCorrelate global `dy`, RMS px @640-wide)** — `dy` = per-frame global
vertical shift between consecutive rendered frames; metric defined in
[`figures/README.md`](figures/README.md#metric-what-phasecorrelate-dy-means):

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

**8c′. Dynamic-vs-static, broken down by smoothing config — all five.** The static penalty is not
uniform: it depends on the *shape* of each config's per-frame required-FOV distribution. From the
`raw_fov` series of all five smoothing configs (default / DCR / per-axis on this branch; **gaussian**
from `claude/gaussian-smoothing` at σ0.5; **L1** from `claude/speed-bump-jolt-rnd`,
`--l1-match-default`, with the `raw_fov` export ported over), black-border frames under DYNAMIC vs
STATIC (equal mean crop), and the extra constant crop a STATIC zoom needs for zero black border:

| config | dynamic bb % | static bb % (equal crop) | static-for-0BB extra crop |
|---|---:|---:|---:|
| default | 0.0 | 2.2 – 4.5 | +19 … 27 % |
| DCR | 0.0 – 0.3 | 2.0 – 4.5 | +23 … 46 % |
| per-axis | 0.2 – 0.8 | 4.5 – 5.5 | +40 … 47 % |
| **gaussian** | 0.0 – 0.4 | 2.7 – 3.8 | **+29 … 73 %** |
| **L1** | **0.0** | **6.9 – 8.3** | **+10 … 13 %** |

(Per-clip numbers in `figures/dynamic_vs_static_zoom_blackborder.png`.)

- **Dynamic zoom wins for every config** — ~0–0.8 % black-border frames vs static's 2.0–8.3 % at
  equal crop. The temporal envelope absorbs whatever crop-demand shape the smoother produces.
- **Static exposes each config's required-FOV *shape*, and it differs sharply:**
  - **default** — flattest demand → cheapest all round (static +19–27 %, 2–4.5 % BB).
  - **DCR / per-axis** — spikier peaks (they zoom harder on high-motion frames) → static needs the
    most constant crop (+40–47 %) to avoid borders.
  - **gaussian** — non-adaptive, so it over-smooths sharp intentional pans into a huge *peak* required
    FOV → **priciest static-for-zero-BB (+73 % on run 0001)**; its bob-band demand is otherwise
    modest (static-equal-crop only 2.7–3.8 %).
  - **L1** — its hard crop box `|path−raw|≤B` *caps* the peak, so it **never breaches under dynamic
    (0.0 %)** and needs the **least** crop for static-zero-BB (+10–13 %). But L1 rides that box
    constantly (it minimises jerk by staying at the edge), so its required FOV is *consistently high*
    → at equal mean crop a static zoom under-covers the **most** frames (**7–8 % BB**, worst of all).
- **Takeaway:** dynamic zoom makes the smoother's crop-demand shape *irrelevant* to black borders
  (~0 % for all five); static zoom makes it decisive, and the exposure is opposite for a capped
  filter (L1: cheap zero-BB, bad equal-crop) vs a peaky one (gaussian: pricey zero-BB). So adaptive
  zoom is not optional for any of these — least of all the aggressive ones. Under dynamic zoom the
  residual black border is tiny for all (§8b: mostly clamp peaks, negligible in renders).

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
  **But it does not generalize — see §8e.**

## 8e. Tier-1 landing — full matrix, offline, both clips (CONFIRMED)

Decision run for shipping a default enhancement. Rendered the full matrix — `default`, `DCR`,
`per-axis` (euler[1]=0.9), `DCR+per-axis` at `max_zoom` 130 and (black-border floor) 170 — offline,
both clips, and measured rendered `dy` + black border. `figures/tier1_stack_eval.png`.

| clip | config | dy RMS | Δ | bb max % | crop (mean zoom) |
|---|---:|---:|---:|---:|---:|
| 0001 | default | 0.675 | — | 1.13 | 1.00 |
| 0001 | **DCR (`--enhanced`)** | **0.469** | **−31 %** | 1.15 | 1.08 |
| 0001 | per-axis y0.9 | 0.508 | −25 % | 1.63 | 1.10 |
| 0001 | DCR+per-axis @130 | 0.386 | −43 % | **8.56** ✗ | 1.13 |
| 0001 | DCR+per-axis @170 | 0.417 | −38 % | 0.58 | higher |
| 0002 | default | 1.369 | — | 1.84 | 0.97 |
| 0002 | **DCR (`--enhanced`)** | **0.896** | **−35 %** | 1.25 | 1.10 |
| 0002 | per-axis y0.9 | 1.152 | −16 % | 1.49 | 1.12 |
| 0002 | DCR+per-axis @130 | 0.902 | −34 % | **8.81** ✗ | 1.18 |
| 0002 | DCR+per-axis @170 | 1.066 | **−22 %** ✗ | 0.29 | higher |

**Conclusion (confirmed):** ship **DCR alone** as the Tier-1 preset (`--enhanced`, keeps the golden
default untouched). It is the clean, robust win on **both** clips — −31/−35 % vertical shake, black
border unchanged from default (~1.2 %), +8…13 % crop.

**Per-axis is excluded**, refuting the §8d hypothesis that it stacks with DCR:
- It only helps clip 0001. On **0002 it gives no benefit over DCR** (−16 % alone vs DCR's −35 %),
  and **DCR+per-axis is actually worse than DCR** there (−34 % → −22 % once the black border is
  floored at mz170).
- `DCR+per-axis` at the default `max_zoom` is **unshippable** (8.6–8.8 % black border — required zoom
  stacks to 1.9–2.0, far past the 1.30 clamp). The floor (mz170) fixes the black border but costs
  more crop everywhere and still doesn't beat DCR on 0002.
- §8d's −16 % was real but clip-specific; per-axis needs a per-clip crop budget it can't be given by
  a fixed default. Kept as an available flag, not in the preset.

Golden parity preserved: library/CLI defaults stay `dcr=false` (ctest 7/7, `--enhanced`≡`--dcr`,
default validate output unchanged).

## 8f. Smoothed-path derivatives by config — and is jerk worth it?

Telemetry-domain smoothness metrics: the 1st/2nd/3rd time-derivatives of the smoothed orientation
(angular velocity °/s, acceleration °/s², jerk °/s³). Tools:
`tools/{angular_velocity_compare,angular_derivatives_compare}.py` (read validate CSVs). Smoothed-path
RMS by config:

| clip | metric | default | DCR | per-axis y0.9 | default→DCR |
|---|---|---:|---:|---:|---:|
| run 0001 | vel / accel / jerk | 18 / 49 / 625 | 16 / 33 / 582 | 15 / 31 / 386 | −9 % / **−33 %** / −7 % |
| run 0002 | vel / accel / jerk | 24 / 93 / 1216 | 21 / 43 / 785 | 20 / 42 / 663 | −15 % / **−54 %** / −35 % |
| bike 0005 | vel / accel / jerk | 8 / 17 / 193 | 7 / 15 / 195 | 6 / 9 / 135 | −9 % / **−12 %** / +1 % |

(Raw path for reference: vel 58/87/28 °/s, jerk 26.8k/52.5k/26.6k °/s³ — smoothing cuts jerk 40–140×.)

**Is the jerk comparison necessary? No — it's largely redundant.**
- **Same ranking at every order.** All three derivatives rank the configs identically
  (per-axis < DCR < default) in 8/9 cases; the one exception is bike jerk (DCR 195 vs default 193 —
  a noise-level tie that flips the order). So jerk changes no conclusion that velocity doesn't
  already give.
- **Acceleration is the cleanest discriminator, not jerk.** default→DCR separates most in
  *acceleration* (−33/−54/−12 %) — consistently more than velocity *and* jerk — because DCR's job is
  to stop the smoother snapping (loosen→re-tighten) on reciprocating shake, and a snap is a
  high-*acceleration* event. Jerk (one more derivative) amplifies that difference but also amplifies
  quantization noise: its default→DCR deltas are erratic (−7/−35/+1 %) and the jerk/accel ratio jumps
  11–18× between configs.
- **Takeaway:** keep **velocity** (physical, intuitive) + **acceleration** (best smoothness
  discriminator); jerk is redundant and noise-fragile — demote it to a footnote.
- The per-axis-lowest-everywhere result restates the §8e tension (lowest derivative ≠ overall-best:
  per-axis buys it with black borders and inconsistent `dy`). Figures:
  `angular_derivatives_compare.png`, `angular_velocity_raw_vs_smoothed.png`.

> (§8g — Gaussian *smoothing* kernel — lives on branch `claude/gaussian-smoothing`.)

## 8h. FOV look-ahead for dynamic zoom — does it help black borders? (No; it fixes zoom pops)

The offline dynamic zoom smooths the per-frame required FOV with a non-causal two-pass envelope
(uses the whole clip's future). For an in-camera build you can buffer only ~1 s of future, so I
added a real-time finite-look-ahead FOV envelope (`AdaptiveZoomParams::look_ahead_s`,
`--zoom-look-ahead S`; `<0` = offline/unchanged, `0` = causal, `1` = 1 s): a look-ahead **minimum**
of required FOV over `[i, i+W]` drives an asymmetric one-pole EMA (fast tighten / slow open) with a
`min(state, required)` guard. Measured on run 0001/0002 + bike 0005, offline vs causal (0 s) vs 1 s:

| clip | mode | black border | clamp-BB | mean zoom | zoom \|Δ\| max | zoom \|Δ\| RMS |
|---|---|---:|---:|---:|---:|---:|
| run 0001 | offline | 0.00 % | 0.00 % | 1.002 | 0.005 | 0.0013 |
| run 0001 | causal 0 s | 0.00 % | 0.00 % | 0.908 | **0.261** | 0.0130 |
| run 0001 | **look-ahead 1 s** | 0.00 % | 0.00 % | 0.997 | **0.030** | 0.0024 |
| run 0002 | causal 0 s | 0.00 % | 0.00 % | 0.892 | 0.114 | 0.0141 |
| run 0002 | **look-ahead 1 s** | 0.00 % | 0.00 % | 0.972 | 0.021 | 0.0019 |
| bike 0005 | causal 0 s | 0.00 % | 0.00 % | 0.871 | 0.068 | 0.0067 |
| bike 0005 | **look-ahead 1 s** | 0.00 % | 0.00 % | 0.927 | 0.028 | 0.0016 |

**Black border is NOT improved by look-ahead — it is already 0 % for every mode.** The envelope is
**min-tracking** (`applied = min(state, required)`), so applied FOV never exceeds required regardless
of how much future it sees; a border can only come from the `max_zoom` clamp (a per-frame required
property, look-ahead-independent, and 0 % here). So the honest answer to "does 1 s future reduce
black borders": **no** — there were none to remove.

**What 1 s look-ahead actually fixes is the *zoom transition*.** Without future, the causal envelope
**snaps** the crop in the instant a shake arrives — a visible zoom *pop* (max per-frame zoom jump
**0.07–0.26**, RMS ~10× rougher than offline) — and it also wastes ~9 % more average crop (it sits
tight longer). 1 s of look-ahead lets the crop **ramp in** before the shake: it cuts the max pop
~5–10× (to 0.02–0.03), the RMS jitter to ~1.5× offline, and recovers offline's mean crop almost
exactly. Figure: `zoom_lookahead_causal_vs_1s.png`.

**Verdict:** use 1 s FOV look-ahead for an **in-camera** build — not to reduce black borders (min-
tracking already gives 0) but to make the real-time zoom as smooth and crop-efficient as the offline
result. Offline (`look_ahead_s < 0`) stays the default and is bit-identical / golden.

## 8i. Spectral shape of the residual — why DJI's dy *looks* cleaner but isn't

Visual impression from the dy traces (run clips): DJI's residual looks like a clean low-frequency
signal while ours, though smaller in RMS, looks spectrally "busier". Band-split analysis of the
residual dy (Welch PSD, fps 29.97; RMS px per band):

| clip | config | <1 Hz | 1–4 Hz (bob) | 4–15 Hz (jitter) | roughness* | centroid | flatness |
|---|---|---:|---:|---:|---:|---:|---:|
| run 0001 | DCR | 0.401 | 0.216 | 0.054 | 0.120 | 0.77 Hz | 0.044 |
| run 0001 | DJI | 1.314 | **1.396** | 0.168 | 0.654 | 1.40 Hz | 0.032 |
| run 0002 | DCR | 0.676 | 0.333 | 0.143 | 0.220 | 0.72 Hz | 0.106 |
| run 0002 | DJI | 1.389 | **3.539** | 0.324 | 1.237 | 1.41 Hz | 0.021 |
| bike | DCR | 0.274 | 0.059 | 0.095 | 0.165 | — | — |
| bike | DJI | 0.288 | 0.070 | 0.087 | 0.144 | — | — |

\*roughness = RMS of frame-to-frame dy change (the jitter-perception proxy).

**The impression is an illusion — three findings (run clips):**
1. **Ours is lower in every band, absolutely** — including the 4–15 Hz jitter band (2–3× lower)
   and frame-to-frame roughness (**5× lower**). Under *any* perceptual frequency weighting our
   residual is steadier; there is no hidden high-frequency penalty.
2. **DJI's residual is not "low-frequency"** — it is the **running cadence, un-removed**: a single
   dominant spectral peak at 1.9 Hz (run 0001, exactly the clip's bob peak) / 1.4 Hz (run 0002),
   sitting in the *worst* band (1–4 Hz bob, RMS 1.4–3.5 px). Its spectral centroid is actually
   *higher* than ours (1.4 vs 0.7–0.8 Hz).
3. **Why it looks "clean" vs our "complex":** DJI's spectrum is *peaky* (flatness 0.02–0.03) — one
   strong periodic component reads as a tidy sinusoid to the eye. We removed that peak; what's left
   is small broadband residual with no dominant component (flatness up to 0.11), which *reads* as
   noise — amplified by the plot scale (DJI's ±4–8 px trace compresses our ±1 px trace into a fuzzy
   band around zero). Zoomed to its own scale, our trace is the smoother one (roughness 5× lower).

**One honest nuance:** a rhythmic, predictable sway (DJI's residual) can be perceptually more
"tolerable" than unstructured residual of equal size because viewers read it as natural motion.
But it is not of equal size — ours is 2–10× smaller in every band. Our remaining <1 Hz residual
(0.3–0.7 px) is the translational-parallax float of §3, the known next frontier. On **bike** the
bands are all within noise of each other, and DJI's roughness is slightly better than DCR's
(0.144 vs 0.165) — consistent with §6 (periphery, not smoothing). Figure:
`dy_spectrum_ours_vs_dji.png`.

## 8j. What IS DJI's filter? A bounded-deviation follower — reproduced with a clamp (CONFIRMED)

Follow-up question: can our EMA be *tuned* to DJI's frequency characteristic? Answering it
uncovered what DJI's stabilizer structurally is.

**8j-1. DJI's measured transfer profile is amplitude-dependent, not a frequency shape.**
Per-band attenuation (original → stabilized) from the 2026-07-08 matched pairs:

| footage | series | <1 Hz | 1–4 Hz | 4–15 Hz |
|---|---|---:|---:|---:|
| calm (pair A) | ours (DCR) | 1.5× | 8.6× | 23× |
| calm | DJI | 1.5× | 6.4× | 16× |
| violent (pair B) | ours (DCR) | 2.1× | 8.1× | 11.7× |
| violent | DJI | **1.3×** | **2.3×** | **2.1×** |

On calm footage DJI's profile is the *same shape* as ours (slightly weaker). On violent footage it
collapses to a **frequency-flat ~2×** — all bands give up proportionally. A linear low-pass cannot
do that (attenuation must grow with frequency); a uniform collapse is the signature of an
**amplitude / deviation budget** being exhausted.

**8j-2. No τ/smoothness tuning reaches it.** Sweep on pair B (DCR off, telemetry proxy):
`--smoothness 0.25` matches DJI's bob band exactly (2.3×) but over-suppresses HF (8.5× vs 2.1×);
`0.10` matches HF (2.7×) but under-suppresses bob (1.4×). The EMA's attenuation always slopes up
with frequency — the flat profile is structurally out of reach for any time-constant setting.

**8j-3. A deviation clamp reproduces it (implemented + render-validated).** Added
`DefaultAlgoParams::deviation_clamp_deg` (`--deviation-clamp B`, off by default → golden intact):
clamp the smoothed path to a max geodesic angle from raw, `slerp(raw, smoothed, B/angle)` wherever
`angle > B`. Telemetry sweep (pair B, smoothness 0.5, DCR off):

| config | <1 Hz | 1–4 Hz | 4–15 Hz | max required zoom |
|---|---:|---:|---:|---:|
| no clamp | 1.1× | 4.2× | 34× | 1.18 |
| **clamp 5°** | **1.1×** | **1.9×** | **2.2×** | **0.88** |
| clamp 3° | 1.1× | 1.4× | 1.6× | 0.81 |
| DJI (target) | 1.3× | 2.3× | 2.1× | — |

Rendered validation (0004 full clip, image-domain dy bands):

| series | dy RMS | <1 Hz | 1–4 Hz | 4–15 Hz | attenuation | roughness |
|---|---:|---:|---:|---:|---|---:|
| original | 12.595 | 2.861 | 10.398 | 6.505 | — | 10.64 |
| our DCR | 1.953 | 1.365 | 1.282 | 0.554 | 2.1/8.1/11.7× | 0.98 |
| **clamp 5°** | **6.949** | 1.826 | 5.544 | 3.771 | **1.6/1.9/1.7×** | 6.07 |
| DJI in-camera | 5.951 | 2.267 | 4.564 | 3.073 | **1.3/2.3/2.1×** | 5.30 |

The clamp-5° render lands within ~15–20 % of DJI on **every** measure — total dy, all three bands,
roughness, and the frequency-flat attenuation signature. Output:
`dji6_L/20260708/0004_D_cpp_clamp5.mp4` (visually: follows the violent motion like DJI does).
Figure: `figures/deviation_clamp_vs_dji.png` (dy trace / PSD / smoothed-path angular acceleration —
the clamp tracks DJI in all three views; DCR is the only config that kills the cadence peak).

**Conclusions:**
1. **DJI in-camera ≈ "smooth + bounded deviation" (a crop-budget-limited follower), not a
   differently-tuned low-pass.** Its violent-footage behaviour is the budget saturating.
2. **To emulate the DJI look:** `--deviation-clamp 5` (DCR off). To merely match its bob band with
   a pure EMA: `--smoothness 0.25` (but that can't reproduce the flat profile).
3. **This is an emulation mode, not an improvement** — DCR beats it 3× on shake. The clamp's real
   value is the property that makes DJI's trade attractive: **hard-bounded crop demand** (max
   required zoom 0.88 at 5° = zero cropping needed, no black-border risk by construction) —
   directly useful for an in-camera product with a fixed crop budget, and structurally the same
   mechanism as L1's crop box.
4. **Hybrid worth testing next:** `--enhanced --deviation-clamp B` with a *generous* B (~8–10°) —
   DCR-quality smoothing in the common case with a hard crop/latency guarantee at the extremes.

**8j-4. Fixing the hard clamp's "burrs" — the SOFT clamp (implemented, render-validated).**
The hard clamp matches DJI's amplitude/spectrum but its trace is full of burrs at the violent
peaks. Diagnosis (measured on 0004): the clip rides the box **55 %** of the time; while saturated
the path moves at **0.70× raw speed** (raw's HF jitter passes through ~1:1, because the box is
centered on the *instantaneous* raw pose), and there are **~4.9 box entry/exits per second**, each
a C¹ kink. The hard projection is a memoryless non-smooth operator — DJI's limiter clearly isn't.

Fix — `--deviation-clamp-soft B [--deviation-clamp-ref-tau 0.02]` (off by default, golden intact):
1. the box center becomes a **smooth reference** (zero-phase EMA of raw). The ref τ is critical
   and must be *short*: τ=0.1 s failed outright (ref itself swings 10–15° from raw during the bob,
   so the box binds on the wrong thing — maxDev 15°, crop bound lost, spectrum ≈ unclamped).
   τ≈0.02 s tracks the 1–4 Hz bob ≈1:1 (box still binds, bound ≈ B+2°) while carrying no >6 Hz
   jitter into the output;
2. the hard wall becomes the smooth saturating map `d_soft = B·tanh(d/B)` — no kinks.

Rendered result (0004, image domain):

| series | dy RMS | 1–4 Hz | 4–15 Hz | roughness |
|---|---:|---:|---:|---:|
| hard clamp 5° | 6.949 | 5.544 | 3.771 | 6.074 |
| **soft clamp 5° τ0.02** | **5.944** | 5.375 | **1.889** | **3.317** |
| DJI in-camera | 5.951 | 4.564 | 3.073 | 5.300 |

The soft clamp lands **exactly on DJI's amplitude** (5.944 vs 5.951) and passes the cadence the
same way — but with **39 % less 4–15 Hz jitter and 37 % less roughness than DJI itself**. I.e. the
same bounded-deviation trade DJI makes, executed cleaner. Path acceleration −38 % vs the hard
clamp (899 → 557 °/s² telemetry; DCR = 60 for reference). This upgrades conclusion 4: the hybrid
should use the **soft** clamp (`--enhanced --deviation-clamp-soft 8..10`). Output:
`dji6_L/20260708/0004_D_cpp_softclamp5.mp4`; figure updated (`deviation_clamp_vs_dji.png`).

**8j-5. Residual gap — harmonic distortion; the elegant fix is joint optimization.** Even the soft
clamp leaves a visible artefact: where DJI's dy shows ONE smooth peak per cadence swing, ours shows
TWO small peaks. Diagnosis: any memoryless per-sample saturation (hard wall or tanh) *flattens the
tops* of the ~sinusoidal cadence deviation; the derivative (dy) of a flat-topped sine has a notch
where the peak was — the peak splits in two. Spectrally this is harmonic distortion, and it is
measured: 2nd-harmonic/fundamental power at f₀ = 1.38 Hz is **0.039 for DJI** (waveform-preserving
— its gain must vary on an *envelope* timescale, a compressor not a clipper), **0.269 for the soft
clamp**, 0.366 for the hard clamp. Figure: `clamp_harmonic_distortion.png`. Elegant fixes, in
increasing order of principle: (a) AGC — replace per-sample tanh with a slowly-varying envelope
gain `g = min(1, B/E(t))` (waveform-preserving by construction); (b) deviation-envelope-adaptive
alpha (the budget folded into the filter — likely DJI's actual form); (c) **joint optimization**:
the real defect is the *sequential* filter-then-limit structure — solving smooth+box as ONE
problem, `min Σ|derivatives| s.t. |path−raw| ≤ B`, is exactly the **L1-optimal smoother**
(`claude/speed-bump-jolt-rnd`) with a small box: its solution rides the box as smooth polynomial
arcs, no clipping, no harmonics, bound exact. Also explains §8c′ (L1 has the flattest crop demand).

---

## 8r. Crop-budget guard (`--fit-crop`) — DCR's black borders solved, and what the honest budget reveals

> Section numbering note: §8k–§8q are the bounded-mode / L1 / real-time R&D sections on the
> side branches (`claude/speed-bump-jolt-rnd`, `claude/deviation-agc`); §8r is their
> EMA-family landing on this branch.

**The problem.** At matched 4:3, DCR (`--enhanced`) breaches the 130 % zoom clamp on
9/37/5/190 frames across the four eval clips, with peak demand **2.397** on the violent clip
(0004) — DCR holds still through impacts, so deviation and crop demand spike; rendered, that is
a **17.7 % black wedge** (31 sampled frames > 1 % of frame area). §8p (side branch) showed a
static angle budget can never fix this tightly (per-axis combination + direction-dependent
angle→zoom slope varies ~3.5×), so the guard works directly in the zoom domain.

**Mechanism** (`smoothing/crop_guard.cpp`, `applyCropBudgetGuard`, post-pass over any
default_algo output): measure the per-frame required zoom of the smoothed path (lens-free
callback from `computeAdaptiveFovs`) → demand envelope (centered window-max 0.8 s + zero-phase
EMA + per-frame peak-hold) → slerp gain toward a fundamental-only reference (zero-phase EMA of
raw, τ 0.03 s) that brings each frame inside a 3 %-margined max_zoom → verify/re-tighten (up to
3 rounds; in practice **one round suffices on all four clips**). Envelope-speed gain =
compressor, not clipper (§8j-5/§8l law); mixing toward the fundamental keeps raw's impact
harmonics out. O(n), and the centered window needs only the pipeline's existing ~1 s
look-ahead → **in-camera realizable**, unlike the L1 branch's offline constraint generation.

**Telemetry**: breaches 9/37/5/190 → **0** (single round), maxReqZ 1.39–2.40 → 1.288–1.290,
deepest gain 0.245 (0004 impact burst = brief bounded follow-through, the DJI behaviour).

**Rendered (matched 4:3; border metric floor set by the borderless default render):**

| clip | DCR plain dy | DCR border | **DCR+guard dy** | guard border | L1 fit-crop dy (ref) |
|---|---:|---:|---:|---:|---:|
| run 0001 | 0.461 | ≈ floor | 0.605 (+31 %) | ≈ floor ✅ | 0.495 |
| run 0002 | 0.834 | 1.72 % | 1.391 (+67 %) | 0.18 % ✅ | 1.097 |
| 0003 | 0.344 | ≈ floor | 0.350 (+2 %) | ≈ floor ✅ | 0.385 |
| 0004 | 1.720 | **17.7 % wedges** | **6.639 (3.9×)** | 0.14 % ✅ | 5.798 |

Figure: `figures/dcr_fitcrop_guard.png`. Renders: `*_D_cpp_dcrfit_4x3.mp4`.

**The honest finding — DCR's violent-clip steadiness was border-financed.** DCR's spectacular
0004 number (1.72 vs everyone else's 4.6–6.6) required 2.4× zoom the budget doesn't allow;
17.7 % of the frame was simply black. Enforce the budget and DCR must follow the violence like
every bounded mode (6.64) — and it does so *worse* than L1's constraint generation (5.80),
because the guard's "collapse toward the fundamental" is a compressor heuristic while L1 plans
the optimal bounded arc (§8j-5c's sequential-vs-joint argument, measured). On calm-to-moderate
clips the guard costs little (+2…+67 %) and DCR+guard stays ahead of plain default.

**Postscript — Gyroflow has a NATIVE version of this mechanism we had not ported.** Rust
`lib.rs:549`: with the default `max_zoom: Some(130), max_zoom_iterations: 5`, Gyroflow loops:
compute fovs → frames whose applied fov falls below the max-zoom floor get
`smoothing_fov_limit_per_frame[f] *= {0.95,0.9,0.85,0.8}` → **re-run the whole smoothing**
(default_algo multiplies the limit into `fov_ratio`, shrinking `max_velocity`, loosening the
adaptive α exactly on those frames) → repeat up to 5 rounds. Same concept as our guard
(per-frame, zoom-domain, re-solve), different actuator: theirs modulates the filter's velocity
limit (a fast, per-frame α change — §8l predicts waveform distortion) and re-smooths up to 5×;
ours is a one-round envelope compressor post-pass. Why golden parity never caught the gap: at
the default 16:9 output_dimension the demand never crosses 1.30 on our eval clips (measured:
0 breaches, peaks 1.17–1.27) so the Rust loop exits at iteration 0 bit-identically — the GUI's
"no borders on default" is ① the 16:9 vertical margin doing most of the work, ② this loop as
the backstop. Porting the native loop for exact parity remains an option (TODO); the guard
covers the same failure mode with a cleaner waveform and lower cost.

**Validated against golden (same smoother, both zero-border).** Rendered Rust Gyroflow's
default config at 4:3 (its native `max_zoom_iterations` loop active) and our
`default --fit-crop` on the breaching clip 0004, and compared on the rendered pixels:

| 0004, 4:3, default EMA | dy RMS | roughness | border |
|---|---:|---:|---:|
| cpp default, no guard | 4.609 | 1.650 | 1.94 % wedges |
| **Rust Gyroflow default (native loop)** | 5.834 | 4.222 | ≈0 ✅ |
| **cpp default + `--fit-crop`** | **5.768** | **3.462** | ≈0 ✅ |

The guard reproduces the golden zero-border behaviour to **−1.1 % dy** with **18 % less
roughness** (trace corr 0.825; outside the burst the traces coincide — the guard is
transparent). Figure: `figures/rust_native_vs_fitcrop_guard_0004.png`. Two conclusions:
(1) the implementation is correct — same-smoother, same-constraint output matches Gyroflow;
(2) the "extra momentary jitter" one sees in guarded footage is the mechanism's physical
follow-through, not an artefact — the golden pipeline shows *more* of it (per-frame α
modulation vs our envelope compressor, §8l as predicted). Render:
`0004_D_cpp_defaultfit_4x3.mp4`, `DJI_20260707235321_0004_D_rust_default_4x3.mp4`.

**Final recommended configs (all zero-border at 4:3/130 %; figure
`figures/c0004_zeroborder_modes_dy.png` overlays them all on the violent clip vs golden vs
DJI):**

| tier | config | 0004 dy / rough | notes |
|---|---|---|---|
| Quality (offline) | `--smoothing l1 --l1-deviation 12 --l1-fit-crop` | 5.80 / 2.71 | best in-budget dy on 3/4 clips (l1 branch) |
| Realtime | `--dcr --fit-crop` | 6.64 / 3.99 | O(n) + 1 s look-ahead; DCR's mild-clip edge intact |
| Compatible | `--fit-crop` (default EMA) | 5.77 / 3.46 | = Gyroflow-default equivalent (golden 5.83 / 4.22) |

`--fit-crop` is recommended always-on in every tier: when demand never crosses the budget the
guard is bit-identical passthrough (measured: 0001/0002/0003 at 4:3 never breach on default,
so the existing `*_default_4x3.mp4` renders ARE the default+fit-crop videos for those clips;
only 0004 needed the guard: `0004_D_cpp_defaultfit_4x3.mp4`).

**Mode map after this section (all zero-border at 4:3/130 %):** L1 fit-crop is the *quality*
choice (best dy on 3 of 4 clips, offline or 1 s-buffer rt pending CG-in-window); **DCR+guard is
the real-time choice** (O(n), 1 s look-ahead, DCR's mild-clip advantage intact); raising
max-zoom remains the only way to keep DCR's violent-clip flatness — the budget, not the
algorithm, is the binding constraint there.

## 8s. Anatomy of the guard's "twitch" — crop-window dx/dy quantified (it's mostly HORIZONTAL)

Follow-up to §8r: guarded footage of the violent clip shows visible twitching. Since the guard
only edits the smoothed quaternions, the crop window's placement in the source frame is fully
determined by telemetry — so the *extra* motion the guard adds can be measured exactly, per
frame and per axis, with no video decode: compare the with/without-guard smoothed paths of the
same clip (`gyroflow_cpp_validate --keep-sensor [--dcr] [--fit-crop]` CSVs).

**Method** (`crop_window_shift.py`, alongside the data): per frame, project the stabilized
optical axis `fwd = R·(0,0,1)` and up vector for both paths; δdx/δdy = the arcsin-component
differences × the §8d calibration (2.36 px/° @640, proxy↔rendered-dy corr 0.999); δroll = the
signed angle between the up vectors; **added jitter** = the per-frame first difference of the
displacement (what reads as twitch); zoom pump = the fov ratio. Axis attribution uses the
rotation vector of `conj(q_noguard)·q_guard` in the IMU frame (X→horizontal, Y→vertical,
Z→roll; first-order-exact at these few-degree moves).

**Telemetry result (0004 burst, 683 frames, dcr→dcrfit):** crop-window displacement dy RMS/max
3.20/18.6 px, **dx RMS 7.13 px**, roll RMS 3.24°, added jitter RMS/max 1.13/7.79 px per frame —
**73.5 % of the guard's motion energy is horizontal** (16.7 % vertical, 9.8 % roll). Root cause
is upstream: DCR's burst deviation is itself yaw-dominated (RMS 9.6°, max **26.4°** horizontal
vs 5.9°/18.5° vertical), far beyond what the 130 % budget can absorb (±10–12° at 4:3), and the
guard's scalar slerp gain collapses each axis in proportion to its deviation. Mild clips
confirm transparency: default→defaultfit touches **0 frames** on 0001/0003, 19 frames at
sub-pixel level (max 0.18 px) on 0002. `cropshift_summary.csv` has all pairs.

**Rendered closure** (`translation_flow` in `gyro_analysis/video_metrics.py` now returns
(dx, dy); dx_analysis over the full render set, 0004, same smoother ± guard):

| 0004, 4:3 | dx RMS | dx rough | dy RMS | dy rough |
|---|---:|---:|---:|---:|
| default, no guard (1.9 % border) | 6.58 | 1.32 | 4.64 | 1.65 |
| default + `--fit-crop` | 6.70 | **1.95 (+47 %)** | 5.40 | 2.29 |
| DCR, no guard (17.7 % wedges) | 6.64 | **0.64** | 1.77 | 0.74 |
| DCR + `--fit-crop` | 7.00 | **2.25 (3.5×)** | 6.40 | 3.06 |

Two readings: (1) **dx RMS barely moves** — horizontal displacement is dominated by the
intentional heading/pan every config must follow, and the guard's follow-through is correlated
with raw so it adds no quadrature amplitude; the twitch lives entirely in **dx roughness**
(frame-to-frame Δdx), which the guard multiplies 3.5× on DCR, time-aligned with the burst.
(2) Ranking at zero border: Rust golden 1.84 ≈ **L1 fit 1.91** ≈ default+fit 1.95 < DCR+fit
2.25 < **DJI RockSteady+ 2.36** — every zero-border config of ours twitches horizontally *less*
than DJI's in-camera result on the same scene.

**Can it be removed?** Amplitude: no — 26° of yaw deviation does not fit a 130 % crop; the
horizontal follow-through is geometric (only raising max_zoom removes it). Jerk (the actual
complaint): yes, in order of value — ① bound the deviation per axis *upstream* (soft clamp on
yaw, §8j-4 mechanics: same amplitude, −37 % roughness measured) so the deviation never piles up
for the guard to collapse; ② L1 fit-crop already realizes the lowest bounded-mode twitch
(1.91 vs 2.25); ③ slow the guard's gain envelope (attack/release) within the 3 % margin;
④ axis-weighted guard exploiting the ~3.5× direction-dependent angle→zoom slope (§8p), with
roll handed to horizon lock (TODO #2; ~10 % of the energy).

**Joint dy+dx verdict across the tiers (rendered 4:3, all four clips).** Reading amplitude
(dy RMS) and twitch (dx roughness) together re-ranks the §8r tiers:

| config | dy RMS 0001/0002/0003/0004 | dx rough 0001/0002/0003/0004 | border |
|---|---|---|---|
| default+fit | 0.73 / 1.67 / 0.38 / 5.40 | 0.31 / 0.47 / 0.19 / 1.95 | 0 ✅ |
| DCR+fit | 0.61 / 1.46 / 0.37 / 6.40 | 0.54 / 0.67 / 0.18 / 2.25 | 0 ✅ |
| L1 fit-crop | 0.50 / 1.16 / 0.39 / 5.69 | 0.24 / 0.37 / 0.20 / 1.91 | 0 ✅ |
| **rt-L1 1 s (box 12, no fit-crop)** | **0.31 / 0.75 / 0.30 / 4.98** | **0.18 / 0.32 / 0.17 / 1.79** | 0004 real (max 3.4 %, 3 % frames >1 %); 0001 trace |
| Rust golden | 0.73 / 1.67 / 0.38 / 5.85 | 0.31 / 0.48 / 0.19 / 1.84 | ≈0 |
| DJI in-camera | 1.93 / 3.84 / 0.35 / 5.62 | 0.80 / 0.87 / 0.18 / 2.36 | full sensor |

- **DCR+fit loses its net advantage**: its run-clip dy edge over default+fit (−13…17 %, itself
  shrunk from the 16:9 no-guard era's −31…35 % — part of DCR's compensation was border-financed)
  is paid back with +43…76 % *horizontal* twitch on exactly those clips (DCR's
  direction-consistency gate follows the running body sway). default+fit or L1 dominate it.
- **L1 fit-crop beats DJI on both axes on 3 of 4 clips** (run clips 2–4×; violent: equal
  amplitude, half the dy roughness, −19 % dx twitch — against RockSteady+). DJI's only
  remaining edge is −11 % dy amplitude on the calm clip (the §6 periphery residual).
- **rt-L1 (§8o, 1 s buffer) posts the best numbers of every config on the mild clips** — dy
  0.31/0.75/0.30, dx roughness lowest across the board, matching §8o's offline-parity claim on
  rendered pixels, and it beats DJI RockSteady+ on the calm clip too. Its one gap — real 3.4 %
  border wedges on the violent clip (no fit-crop in the rt path) — is **CLOSED by §8t on the
  l1 branch**: E4's constraint generation now runs inside the §8o window
  (`--l1-look-ahead 1 --l1-fit-crop`), rendering 0004 at the l1fit border floor (max 0.17 %,
  none > 1 %) with dy 6.14 / dy rough 2.29 / **dx rough 1.41** — dominating DCR+guard on every
  metric at zero border; it takes the Realtime tier. §8t also *corrects* a presumption made
  here: the rt path needs **no tightening at all on 0001/0002/0003** (maxReqZ 1.21–1.29) — the
  offline global solve's 79-frame breach on 0001 comes from planning long box-riding arcs the
  receding-horizon solve never accumulates, so rt-L1's mild-clip wins were legitimate, not
  border-financed, and the "l1-fit-crop tightens everywhere" reading below applies to the
  offline solver only.
- Border-metric caveat reconfirmed (§8b): on the night-shot calm clip the edge-connected
  near-black counter reads ~1.5 % mean / 57–70 % max even for guaranteed-zero-border renders
  (dark scene content touching the frame edge; DJI's own render reads 0.6 %/12 %) — per-clip
  floors from a known-clean render remain mandatory before attributing "borders".

Data/figures/scripts: `dji6_L/results_4x3_0001-0004/` (all 4:3 renders of all tiers + rt-L1 +
Rust golden + DJI refs, hardlinked; `README.txt` maps sources — 0001/0002 refs are RockSteady,
0003/0004 refs RockSteady+ dual-camera takes) with `analysis/` (path/cropshift/dxy CSVs,
`results_analysis.py`, `crop_window_shift.py`, `dx_analysis.py`). Repo figures:
`figures/cropshift_0004_dcrfit.png`, `figures/dx_compare_0004.png`.

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
- **Tier-1 landing = DCR alone (§8e, CONFIRMED):** shipped as `--enhanced`. −31/−35 % rendered
  vertical shake on both clips, black border unchanged from default, +8…13 % crop; golden default
  untouched. **Per-axis (§8d) was evaluated and excluded** — it helps only one clip, gives no gain
  on 0002, and stacking it with DCR forces 8.6–8.8 % black border (needs more crop than a fixed
  default can give). Kept as a flag, not in the preset.

### Candidate work items
1. Translation-domain residual stabilization (the actual visible-float fix).
2. **(next) Swap the EMA base kernel for a Gaussian (linear-phase) kernel** and evaluate against the
   current EMA/DCR on **both** the optical-flow (`dy`, `tools/vertical_flow_compare.py`) **and the
   angular-acceleration** (`tools/angular_derivatives_compare.py`, §8f — accel is the cleanest
   smoothness discriminator; jerk is redundant) axes — a linear-phase Gaussian should lower the
   higher derivatives without the DCR gate; check whether it holds `dy` and crop, on real renders
   (the §6 numbers are a 30 Hz telemetry proxy).
3. Mean-fov-matched L1 vs DCR vs Gaussian comparison (telemetry-fast).
4. Add a jerk-RMS metric to `stabilization_quality.py`.
5. Re-run §5/§6 under a 1 s-look-ahead cap (truncated backward pass / receding-horizon) for
   in-camera-achievable numbers.
6. (§8b) Raise `max_zoom` to ~160 % for DCR configs, or floor DCR's required FOV, to kill the
   handful of clamp-forced black-border frames.
7. ~~(§8d) Ship per-axis vertical smoothing as a default-on option~~ — **done/closed by §8e:**
   evaluated the full matrix; **DCR alone** ships as `--enhanced`, per-axis excluded (no gain on
   0002, black-border cost). Next: after Tier-1, move to translation-domain stabilization (§3).

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
