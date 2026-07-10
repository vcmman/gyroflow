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

**8j-6. Joint optimization VERIFIED — L1 with a small box IS the clean DJI mode (CONFIRMED).**
The L1 branch was rebased onto the full current stack (this branch now carries DCR/`--enhanced`,
look-aheads, clamps, `raw_fov` AND `--smoothing l1`; golden md5 unchanged, ctest 7/7). Rendered
0004 with `--smoothing l1 --l1-deviation 5` and compared all three bounded-deviation
implementations on the actual pixels:

| series | dy RMS | <1 Hz | 1–4 Hz | roughness | **2nd-harm/fund** |
|---|---:|---:|---:|---:|---:|
| hard clamp 5° | 6.949 | 1.826 | 5.544 | 6.074 | 0.366 |
| soft clamp 5° | 5.944 | 1.697 | 5.375 | 3.317 | 0.269 |
| **L1 box 5°** | 7.328 | 2.282 | 6.348 | 5.197 | **0.043** |
| DJI in-camera | 5.951 | 2.267 | 4.564 | 5.300 | **0.039** |

- **The waveform-shape signature matches DJI exactly**: L1's 2nd-harmonic ratio 0.043 vs DJI's
  0.039 (clamps: 0.27–0.37). In the dy zoom the split peaks are gone — one smooth peak per swing,
  same as DJI (`clamp_harmonic_distortion.png`, updated). L1's <1 Hz band (2.282) is also nearly
  identical to DJI's (2.267).
- Amplitude is tunable by the box: the 5° *per-axis* box (combined geodesic up to ~9°) lets
  slightly more through than DJI (dy 7.33 vs 5.95); ~4° per-axis would land the amplitude. Path
  acceleration 326 °/s² — the lowest of the three bounded modes (hard 899 / soft 557).
- Telemetry HF is crushed (33×) yet image HF stays ~DJI-level — the image-domain HF at these
  amplitudes is the §3 translational/RS content, unreachable by any rotational path.

**Final ranking of the bounded-deviation ("DJI look") implementations:** L1-joint ≫ soft clamp >
hard clamp. The sequential filter+limit structure is inherently a waveform clipper; solving
smooth+box jointly is both the elegant formulation and the measured winner. Practical notes: L1
cost ~2 s for a 66 s clip (2000 ADMM iterations) vs microseconds for the clamps; the soft clamp
remains a reasonable cheap approximation for in-camera use, and the AGC/envelope-gain idea (8j-5a)
is the middle ground if L1 is too heavy. Output: `dji6_L/20260708/0004_D_cpp_l1box5.mp4`.

**8j-7. Tuning the box — L1 box 12° beats DJI on every measure (dy −29 %).** Two candidate levers
for lowering `dy` further, swept on telemetry then render-confirmed:
- **Velocity weight w₁ is NOT the lever** (10→100 gives −2 % dy and *worsens* the harmonic ratio
  0.087→0.139 — per-sample velocity pressure re-introduces mild waveform distortion). Keep the
  jerk-dominant defaults (10/1/100).
- **Box size is the lever**: 5→8→12° (per-axis) drops proxy dy 2.25→2.00→1.88, and — key point —
  **box 12° is the largest box whose required zoom stays inside the default 130 % clamp**
  (maxReqZ 1.278): the crop-budget-optimal L1. Rendered 0004:

| series | dy RMS | <1 Hz | 1–4 Hz | 4–15 Hz | roughness | 2nd-harm | bb mean |
|---|---:|---:|---:|---:|---:|---:|---:|
| L1 box 5° | 7.328 | 2.282 | 6.348 | 2.862 | 5.197 | 0.043 | 0.005 % |
| **L1 box 12°** | **5.204** | **1.881** | **4.353** | **2.144** | **3.720** | 0.062 | 0.005 % |
| DJI in-camera | 5.951 | 2.267 | 4.564 | 3.073 | 5.300 | 0.039 | 0.002 % |

→ **L1 box 12° is strictly better than DJI in the bounded-crop class**: dy −12 %, every band lower,
roughness −30 %, waveform still clean (0.062), zero geometric border, crop bounded by construction.
dy path: 7.33 (box 5) → **5.20 (box 12)**, i.e. −29 % from box tuning alone. Continuing to grow the
box converges toward DCR quality (1.95) but the required zoom leaves the clamp — the box↔max_zoom
coupling is the productization item already on the list (derive B from `max_zoom` automatically →
a self-tuning "max quality within the crop budget" L1 mode). Output:
`dji6_L/20260708/0004_D_cpp_l1box12.mp4`.

**8j-8. Converged three-way picture (EMA vs DJI vs L1), violent clip 0004** — figure
`ema_vs_dji_vs_l1_dy.png` (trace zoom / spectrum / summary bars):

| method | dy RMS | roughness | 2nd-harm (waveform) | max required zoom |
|---|---:|---:|---:|---:|
| plain two-pass EMA (no DCR) | **4.039** | **2.269** | 0.132 | 1.18 |
| DJI in-camera | 5.951 | 5.300 | **0.039** | bounded (n/a) |
| L1 box 12° | 5.204 | 3.720 | **0.062** | 1.28 |

- **Unconstrained class:** the plain velocity-adaptive EMA is the steadiest of the three (dy 4.04)
  at modest crop demand (1.18) — and DCR pushes further still (1.95 at required 1.8, clamped).
  Nothing bounded can match this; the guarantee costs amplitude.
- **Bounded class:** L1 box 12° beats DJI on amplitude (−12 %) AND roughness (−30 %) with an
  equally clean waveform — the class winner (§8j-7).
- **Waveform cleanliness is its own axis:** even the plain EMA shows a mild 2nd-harmonic (0.132,
  3× DJI) — its velocity-adaptive α modulation is itself a weak per-sample nonlinearity. Only the
  jointly-optimized L1 (0.062) reaches DJI's (0.039) distortion-free class.

So the final map: **DCR / EMA for maximum steadiness (unbounded crop demand), L1-with-box for the
guaranteed-crop class (beats DJI), soft clamp as its cheap real-time approximation.**

Merged evidence views (all eight configs — original / EMA / DCR / hard clamp / soft clamp /
L1 box5 / L1 box12 / DJI): `all_bounded_experiments_dy.png` (trace zooms + sorted summary bars),
`all_bounded_experiments_dy_full.png` (full 1973 frames), `all_bounded_experiments_dy_400_900.png`
(calm→violent transition window). Full dy ladder on 0004: DCR 1.95 < EMA 4.04 < **L1 box12 5.20**
< soft clamp 5.94 ≈ DJI 5.95 < hard clamp 6.95 < L1 box5 7.33 < original 12.60.

## 8j-9. Replication on run 0002 — the ladder INVERTS: L1 box12 beats even DCR

Repeated the whole experiment matrix on the original violent running clip (`dji6_L/run` 0002,
1487 frames, original dy 8.53 px — less violent than 0004's 12.6). Figures:
`run0002_bounded_experiments_dy.png` / `_full.png`.

| rank | config | dy RMS | roughness | note |
|---|---|---:|---:|---|
| **1** | **L1 box 12°** | **0.639** | **0.218** | **beats DCR** |
| 2 | DCR | 0.767 | 0.220 | |
| 3 | EMA default | 1.300 | 0.383 | |
| 4 | L1 box 5° | 3.100 | 1.009 | beats DJI |
| 5 | soft clamp 5° | 3.230 | 2.116 | beats DJI |
| 6 | hard clamp 5° | 3.814 | 3.687 | ≈ DJI |
| 7 | DJI in-camera | 3.815 | 1.237 | |
| 8 | original | 8.534 | 5.781 | |

**Key finding — the box-vs-violence law.** On 0002 the 12° box barely binds (max required zoom
1.329, 9 frames marginally past the clamp), so L1 acts as an (almost) *unconstrained global
optimizer* — and it **beats the greedy recursive EMA family outright** (0.639 vs DCR's 0.767 at
equal roughness). Combined with 0004 (box binds → DCR wins 1.95 vs 5.20) the rule is:

> **when the clip's demand fits inside the crop budget, jointly-optimized L1 is the best smoother
> we have; when demand exceeds the budget, the unbounded DCR wins on amplitude.**

This makes 0c′ (derive the box from `max_zoom` automatically) more than a convenience — a
self-tuning L1 would inherit the best of both regimes on a per-clip basis, and is the natural
candidate to challenge DCR as the default. Also note: on 0002 *every* bounded config of ours beats
DJI (even L1 box5 / soft clamp). (Harmonic-ratio caveat: for heavily-smoothed configs — L1 box12 /
DCR — the 2nd/fund ratio is meaningless because the fundamental itself is crushed; their absolute
harmonic energy is the lowest of all.)

## 8k. Plain EMA (no DCR) vs DJI — the stock defaults already win, with ZERO black borders

Isolated comparison of the **unmodified Gyroflow default smoother** against DJI in-camera, across
all four measured clips. Parameters — exactly `DefaultAlgoParams` defaults, the golden path:
`smoothness 0.5`, `max_smoothness 1.0 s` (zero-velocity time constant), `alpha_0_1s 0.1 s`
(max-velocity time constant), `second_pass on`; per-axis / DCR / clamps / look-ahead all off;
zoom envelope, window 4 s, `max_zoom 130 %` (identical for every config).

| clip | EMA `dy` | DJI `dy` | ratio | EMA rough | DJI rough | ratio |
|---|---:|---:|---:|---:|---:|---:|
| run 0001 | 0.675 | 1.926 | **2.9×** | 0.158 | 0.654 | **4.1×** |
| run 0002 | 1.369 | 3.843 | **2.8×** | 0.383 | 1.237 | **3.2×** |
| 0004 (violent) | 4.039 | 5.951 | **1.5×** | 2.269 | 5.300 | **2.3×** |
| bike (smooth) | 0.320 | 0.311 | tie | 0.145 | 0.144 | tie |

- **Amplitude AND smoothness: 1.5–4× better than DJI on violent footage, tie on smooth** —
  before any of our enhancements. (As-delivered framing caveat: ours 16:9 vs DJI 4:3; violent
  multiples are immune to it, the smooth-footage tie may tilt slightly DJI at matched 4:3 — the
  §6 periphery effect.)
- **Zero geometric black border**: the plain EMA's required zoom never breaches the 130 % clamp on
  any measured clip (peaks 1.16–1.27; even 0004 only 1.18) — unlike DCR, whose violent-clip spikes
  (1.4–1.8, up to 3 % of frames, wedges up to ~28 % of half-frame on breach frames) are the black
  borders visible in DCR exports.
- Waveform: near-DJI-clean on run 0002 (2nd-harm 0.020 vs 0.017), mildly distorted on 0004
  (0.132 vs 0.039 — the velocity-adaptive α is itself a weak per-sample nonlinearity, §8j-8).
- Mechanism note: EMA and DJI are the same *family* ("follow more when motion is big") — EMA
  modulates a time constant with no hard budget, so it keeps far more HF suppression than DJI's
  budget-saturation collapse (0004 attenuation profile 1.1/4.2/34× vs DJI's flat 1.3/2.3/2.1×).

**Practical config ladder** (updates the §8j-8 map with the border dimension):
1. **conservative / zero-border**: plain EMA — already ≥ DJI everywhere;
2. **max steadiness**: DCR + `--max-zoom 180` (kills the clamp wedges at zero cost on quiet
   sections — the envelope only takes what each frame needs);
3. **bounded-crop guarantee / cinematic**: L1 box12 (beats DCR outright when the box doesn't
   bind, §8j-9).

## 8l. Mechanism taxonomy — how each smoother's nonlinearity shapes the waveform

A unifying frame for everything measured in §8i–§8k. Decompose the shake residual into two
timescales:

```
d(t) ≈ A(t) · sin(2πf₀t)
        slow      fast
      envelope   carrier (cadence, ~1.4 Hz, period ~0.7 s)
```

The **carrier** is each individual swing; the **envelope** A(t) is how violent the motion is,
varying over seconds. Every stabilizer that limits amplitude applies some effective attenuation —
the question that decides waveform cleanliness is **how fast that attenuation is allowed to
change**:

- **Envelope-level slowly-varying gain** ("AGC" / compressor): `out = g(t)·d(t)` with g changing
  only across cycles (slower than the carrier). Within any one cycle g ≈ const, so a sinusoid
  stays a sinusoid — amplitude control happens *between* cycles, **zero harmonic distortion**.
  Audio analogy: a compressor's attack/release rides the envelope.
- **Per-sample saturation** (clipper): reacts *within* the cycle — reshapes the waveform itself
  (flattens the tops), pumping energy into harmonics. Audio analogy: a hard/soft clipper.

Five mechanism classes, anchored by the measured 2nd-harmonic ratio (0004, f₀ = 1.38 Hz;
DJI-clean ≈ 0.04):

| class | mechanism | gain-variation speed | 2nd-harm (measured) | budget? |
|---|---|---|---|---|
| linear filter (fixed-α EMA, Gaussian kernel) | LTI convolution | none | zero by definition | no |
| **envelope gain / AGC (= DJI's form)** | g(t) follows the envelope | **slower than the carrier** | **0.039** | soft, on the envelope |
| velocity-adaptive EMA (our default / DCR) | α driven by smoothed velocity (τ = 0.1 s) | **sub-cycle** (0.1 s < 0.7 s period) but smooth | 0.020 (mild motion) – 0.132 (violent) | no |
| per-sample saturation (hard / soft clamp) | project/compress each sample | sample rate | 0.269 – 0.366 | hard, exact |
| **joint optimization (L1 + box)** | not a gain — globally re-solves the path | — (global) | **0.043 – 0.062** | hard, exact |

Readings:
- **DJI's 0.039 identifies its limiter as an AGC**: no time-constant tuning of a low-pass can
  produce its frequency-flat, distortion-free budget collapse (§8j-1/2) — only an envelope-rate
  gain (or an equivalent global method) can.
- **Our velocity-adaptive EMA sits in between**: the *fixed-α* EMA core is linear (no distortion
  question at all); the mild distortion (0.132 under violence) comes purely from α being modulated
  at sub-cycle rate — the velocity smoothing τ (0.1 s) is *shorter* than the cadence period, so α
  breathes within each swing. Driving α from an **envelope** of velocity/deviation instead (the
  §8j-5b idea) would turn it into a true AGC — very likely DJI's actual implementation.
- **L1 is a third category, not a gain at all**: its nonlinearity (soft-thresholding) manifests as
  *sparse knot placement*, not waveform reshaping — within segments the output is an exact
  polynomial, and at the box it rides tangential arcs. It reaches AGC-level cleanliness (0.043)
  while *also* delivering what an AGC cannot: an exact deviation bound and global optimality.
- The clamps are the cautionary tale: any per-sample limiter — however soft — is a clipper, and
  the 6–9× harmonic penalty vs DJI is structural, not tunable away (§8j-4/5).

## 8m. Unified matched-4:3 comparison — 4 clips × {default, DCR, L1 box12} vs DJI

All configs re-rendered at `--keep-sensor` 4:3 (full sensor, same framing as DJI — the periphery
included, no 16:9 flattery), all four clips. dy = image domain; accel/jerk = telemetry (DJI n/a).
Figure: `unified_4x3_dy_accel_jerk.png`.

| clip | default | DCR | L1 box12 | DJI | dy winner |
|---|---:|---:|---:|---:|---|
| run 0001 | 0.727 | 0.461 | **0.304** | 1.924 | L1 (6.3× vs DJI) |
| run 0002 | 1.606 | 0.834 | **0.700** | 3.815 | L1 (5.5× vs DJI) |
| 0003 (calm, 277 s) | 0.359 | 0.344 | **0.274** | 0.334 | **L1 — beats DJI on calm too** |
| 0004 (most violent) | 4.609 | **1.720** | 4.901 | 5.951 | DCR (3.5× vs DJI) |

telemetry accel / jerk (°/s², °/s³):

| clip | default | DCR | L1 box12 |
|---|---|---|---|
| run 0001 | 49 / 625 | 33 / 582 | **19 / 121** |
| run 0002 | 93 / 1216 | 43 / 785 | **30 / 180** |
| 0003 | 25 / 198 | 23 / 204 | **18 / 75** |
| 0004 | 215 / 2856 | **65 / 1200** | 180 / 1759 |

**Headlines:**
1. **L1 box12 wins 3 of 4 clips outright — including the calm clip at matched 4:3** (0.274 vs
   DJI 0.334, −18 %), the exact scenario where default/DCR slightly *lose* to DJI (0.359/0.344 vs
   0.334 — the §6 periphery deficit, replicated on 0003). The globally-optimized polynomial path
   apparently also excites less periphery residual. This closes the last per-scenario gap vs DJI.
2. **The box-vs-violence law (§8j-9) holds at 4:3**: on the most violent clip (0004, original
   12.6 px) the box binds and DCR wins (1.720); L1 box12 degrades to ≈ default. Everywhere else
   L1 dominates all three metrics simultaneously (dy AND accel AND jerk — jerk 3–5× below DCR).
3. **vs DJI at fully matched framing: our best config beats DJI on every clip** (L1 on three,
   DCR on the fourth) — no remaining scenario where DJI leads.

This further sharpens 0c′: a self-tuning L1 (box from `max_zoom`, falling back toward DCR-like
behaviour only when the required box would exceed the budget) would be the best-of-all-rows
single default.

## 8o. Real-time L1 with a 1 s future buffer — offline quality at 56× realtime (CONFIRMED)

The last objection to L1 as an in-camera mode was cost/latency (global offline solve). Implemented
a **receding-horizon** variant (`--l1-look-ahead S`; `<0` = offline global, unchanged):

- window = [**2 s past context** | **commit block K=15 frames** | **S=1 s future buffer**];
- already-committed samples are pinned by a zero-width box (continuity anchor — the window
  solution must pass exactly through the emitted history);
- solve the small window by the same per-euler ADMM (800 iterations, converges fast at this size),
  commit K frames, slide. Streamable: needs exactly the S-second future buffer that the smoothing
  (§7) and zoom (§8h) look-aheads already share.

**Telemetry (0004, box 12°):**

| config | dy proxy | 2nd-harm | accel | maxReqZ | cost (66 s clip) |
|---|---:|---:|---:|---:|---:|
| offline L1 (global) | 1.880 | 0.036 | 161 | 1.278 | ~2 s |
| **rt-L1, 1 s buffer** | **1.879** | **0.033** | **166** | 1.319 | **1.17 s (56× RT)** |
| rt-L1, 0 s (causal) | 2.148 | 0.724 | 427 | 1.406 | — |

**Rendered (matched 4:3, image domain):**

| series | dy | 1–4 Hz | roughness | harm |
|---|---:|---:|---:|---:|
| offline L1 box12 | 4.901 | 4.145 | 3.169 | 0.042 |
| **rt-L1 1 s** | **4.915** | 4.101 | 3.357 | 0.079 |
| DJI in-camera | 5.951 | 4.564 | 5.300 | 0.039 |

- **1 s of future is sufficient**: the rt solution is metrically indistinguishable from the global
  offline solve (dy +0.3 % on pixels, harmonic same class) — and still beats DJI. The causal (0 s)
  ablation collapses (harm 0.72, accel 2.6×): the future buffer is where the quality comes from,
  exactly as in §7/§8h.
- **Tuning lesson**: the initial "commit-boundary artifacts" (harm 0.14 at 300 iters) were
  *under-convergence*, not a structural flaw — 800 iterations per window removes them entirely,
  and a larger commit block (K=15) is *better* (fewer, better-converged windows).
- **Cost**: 1.17 s for a 66 s clip single-threaded ≈ **56× realtime** at 800 iterations,
  O(nf · rt_iterations), constant memory — in-camera viable. Combined with §8m/§8j-9 this closes
  L1's last gap: it now wins 3 of 4 clips *and* runs in real time with the same 1 s buffer the
  rest of the pipeline uses.

**Full-matrix confirmation (all four clips, matched 4:3, unified figure updated):**

| clip | offline L1 dy | rt-L1 dy | offline accel | rt accel | rt jerk vs offline |
|---|---:|---:|---:|---:|---|
| run 0001 | 0.304 | **0.306** | 19 | 19 | 292 vs 121 |
| run 0002 | 0.700 | **0.669** | 30 | 29 | 388 vs 180 |
| 0003 | 0.274 | **0.283** | 18 | 21 | 236 vs 75 |
| 0004 | 4.901 | **4.915** | 180 | 180 | 1674 vs 1759 |

- dy and acceleration are **identical to offline on every clip**; the residual fingerprint is a
  2–3× jerk premium on calm clips (commit-boundary micro-kinks, invisible in dy/harm/accel).
- **Iteration economics**: 800 iters = 56× realtime but leaves jerk artifacts on calm clips
  (under-convergence at commit boundaries, e.g. run 0001 dy 0.412 vs 0.304); **4000 iters (the new
  rt default) ≈ 11× realtime** and closes dy/accel exactly (0.306 vs 0.304). The three euler
  channels are independent → parallelize to ~33× realtime on 3 cores if needed.

**rt-L1 vs the deviation AGC (the two real-time bounded modes, head-to-head; AGC from
`claude/deviation-agc`, §8n):**

| metric (4 clips) | AGC 8° | rt-L1 (1s) | ratio |
|---|---|---|---|
| dy (matched 4:3) | 0.727 / 1.606 / 0.351 / **4.647** | **0.306 / 0.669 / 0.283** / 4.915 | rt-L1 up to **2.4×** better |
| accel (°/s²) | 54 / 93 / 35 / 219 | **19 / 29 / 21 / 180** | rt-L1 2–3× better |
| jerk (°/s³) | 1300 / 1220 / 813 / 3242 | **292 / 388 / 236 / 1674** | rt-L1 3–4× better |
| waveform (2nd-harm) | 0.078 | ~0.079 (image) | same clean class |
| compute | **O(n), µs-class** | ~11× realtime (ADMM) | AGC ~1000× cheaper |

**rt-L1 dominates the AGC on every quality metric wherever its box doesn't bind** (2.4× dy on the
run clips); the one clip AGC edges it (0004, 4.647 vs 4.915) is the box-binding regime where the
AGC inherits its wrapped default-EMA — the same box-vs-violence law again. The AGC's remaining
claims are its trivial O(n) cost and implementation size (~50 lines vs the ADMM solver). Verdict
for an in-camera product: **rt-L1 is the quality choice; the AGC is the ultra-low-cost fallback**
for platforms where even 11× realtime (or ~33× parallelized) is too much.

**DCR + 1 s look-ahead at matched 4:3 — completing the real-time lineup.** §7/§8a established
"DCR fits in a 1 s buffer" on 16:9 run renders only; re-rendered all four clips at matched 4:3
(`--dcr --look-ahead 1 --keep-sensor`, zoom offline as in the rt-L1 renders) to make the
real-time-DCR row comparable with everything else in the unified matrix:

| clip | DCR offline dy | DCR + 1 s LA dy | Δ |
|---|---:|---:|---:|
| run 0001 | 0.461 | 0.481 | +4.3 % |
| run 0002 | 0.834 | 0.847 | +1.6 % |
| 0003 | 0.344 | 0.344 | ±0 % |
| 0004 | 1.720 | 1.726 | +0.3 % |

The 1 s-truncation cost is ≤4 % everywhere (roughness identical: 0.228 vs 0.227 on run 0002) —
the §7 conclusion holds unchanged at 4:3 and on the violent clip. So the full real-time (1 s
buffer) lineup is now: **DCR+LA1** (max steadiness, needs `--max-zoom 180`), **rt-L1 box 12°**
(bounded winner), **AGC** (µs-class fallback). Traces:
`figures/run0002_dy_traces_all_modes.png` (all seven series; DCR+LA1 rides on DCR offline),
`figures/run0002_dy_dcr_la1_vs_offline.png` (focused pair vs DJI), and
`figures/c0004_dy_traces_all_modes.png` (same seven series on the violent clip: the DCR pair
stays flat through the impact burst, RMS 1.72 vs DJI 5.95, while the bounded modes follow the
violence inside their budget as DJI does). Renders:
`{0001,0002}_D_cpp_stabilized_dcr_la1_4x3.mp4` (run/cpp_out), `{0003,0004}_D_cpp_dcr_la1_4x3.mp4`
(20260708).

---

## 8p. Eliminating L1 black borders — diagnosis + E2 (static box shrink): works, but too costly

**Why L1 box12 shows black borders at 4:3** (quantified via validate `raw_fov`, breach =
required zoom > the 1.30 max-zoom clamp): run0001 79 frames (3.4 %, peak demand 1.582),
run0002 100 (6.7 %, 1.515), 0003 9 (0.1 %), 0004 143 (7.2 %, 1.621). Three stacked root causes:

1. **Per-axis box ≠ total-deviation box**: 12° per euler axis allows a combined 3-axis deviation
   up to 12√3 ≈ 21° — breach frames measure 15–20° total geodesic deviation.
2. **Calibration framing mismatch**: box 12 was tuned at 16:9 (0004 maxReqZ 1.278 < 1.30); 4:3
   uses the full sensor height, so the same angular deviation demands more crop.
3. **Angle→zoom mapping is not constant**: measured slope k = (reqZ−1)/deviation varies 0.007 →
   0.0245 with deviation direction / lens position / RS — a static angle box can never be tight
   in the zoom domain. The constraint belongs in the *zoom* domain.

**Experiment ladder**: E1 raise max-zoom (reference line only) · E2 static box shrink (below) ·
E3 geometry-derived per-axis box from max_zoom (TODO 0c′) · E4 exact per-frame crop constraint
via constraint generation (Grundmann's original form) · E5 zoom-side soft ceiling (release the
clamp transiently for residual breaches).

**E2 result — box 7.5° is the largest all-safe static box at 4:3** (telemetry sweep: box 8
leaves 2 breach frames on run0001 @1.312; box 7.5 → zero breaches on all four clips, peaks
1.176–1.271; box 6 large-margin safe at ≤1.173). Rendered box 7.5 on all four clips, matched
4:3 (image-domain border metric = edge-connected near-black fraction; night-scene floor set by
the borderless default render):

| clip | box12 dy | **box7.5 dy** | Δ | box12 border (max %) | box7.5 border |
|---|---:|---:|---:|---:|---:|
| run 0001 | 0.304 | 0.599 | **+97 %** | 2.36 (real) | 0.84 < floor ✅ |
| run 0002 | 0.700 | 1.963 | **+180 %** (worse than default 1.606) | 1.42 | 0.32 ≈ floor ✅ |
| 0003 | 0.274 | 0.360 | +31 % | ≈ floor | ≈ floor ✅ |
| 0004 | 4.901 | 6.606 | **+35 %** (worse than DJI 5.951) | 3.55 | 0.20 < floor ✅ |

**Verdict: zero black borders achieved, but the static price is unacceptable** — where the box
binds, the shrink hands back half to all of L1's advantage (run0002 ends up worse than plain
default; 0004 worse than DJI). A single static box that survives the worst frame of the worst
clip over-constrains the other 93–99 % of frames. This is exactly the case for **E3/E4**: derive
the budget from max_zoom geometrically (E3) and tighten only the violating frames via
constraint generation (E4; §8q measures the real outcome — the naive "≈ box12 dy" hope was
wrong, breach frames are the dy-dominating peaks, but E4 still beats E2 on 3 of 4 clips).
Renders:
`{0001,0002}_D_cpp_l1box75_4x3.mp4` (run/cpp_out), `{0003,0004}_D_cpp_l1box75_4x3.mp4`
(20260708). Side note: DJI's dy (5.95) sits between our box7.5 (6.61) and box12 (4.90) on 0004 —
its effective deviation budget is bracketed by 7.5°–12° for this lens.

---

## 8q. Zero-border L1 SOLVED — E3 geometric budgets + E4 per-frame constraint generation

Implemented both zoom-domain fixes from the §8p ladder (branch: `--l1-fit-crop`,
`--l1-auto-box [scale]` on both CLIs; `smoothL1CropConstrained` in the library, lens-free via a
required-zoom callback; unit-tested with a synthetic demand model, ctest 7/7, golden md5
unchanged).

**E3 — geometric per-axis budgets** (`--l1-auto-box`): bisect, per euler axis, the constant
pure-axis offset whose instantaneous required zoom hits max_zoom (identity base, 8-frame probe
series — pure lens geometry, no clip data). For this lens at 4:3 / 130 %: **roll 11.7°,
pitch 16.0°, yaw 32.2°** (single-axis). The equal box 12 was thus *already over the roll budget
on its own* and wasted ~3× headroom on yaw — the per-axis asymmetry the static box ignores.
Scale 0.577 (1/√3, combined-use de-rate) is the default; also used as the fit-crop initial box.

**E4 — constraint generation** (`--l1-fit-crop`): outer loop around the per-channel ADMM —
solve, ask the zoom machinery for per-frame required zoom (instantaneous `raw_fov`), shrink the
per-frame per-axis boxes of violating frames (±2-frame halo) toward the deviation that meets a
3 %-margined target, re-solve. Telemetry: **all four clips converge in 3–4 rounds, breaches
79/100/9/143 → 0**, maxReqZ 1.51–1.62 → 1.25–1.29, cost +1.3–6.9 s per clip.

**Rendered verdict (matched 4:3, image domain — zero borders confirmed on all four clips**,
bbmax 0.09–0.16 % ≤ the borderless-default floor; 0003 floor-dominated by real dark scenery):

| clip | box12 dy (borders!) | box7.5 dy (E2) | **fit-crop dy (E4)** | E4 vs E2 | DJI |
|---|---:|---:|---:|---|---:|
| run 0001 | 0.304 | 0.599 | **0.495** | −17 % | — |
| run 0002 | 0.700 | 1.963 | **1.097** | **−44 %** | 3.815 |
| 0003 | 0.274 | 0.360 | 0.385 | +7 % (both ≈ floor) | — |
| 0004 | 4.901 | 6.606 | **5.798** | −12 % (back under DJI 5.951) | 5.951 |

- **Zero borders is now a solved constraint, not a trade-off knob**: fit-crop beats the best
  static zero-border box (E2) on 3 of 4 clips — dramatically on run0002 (1.10 vs 1.96, and well
  under default's 1.61) — and returns 0004 to *better than DJI* (5.80 vs 5.95), which E2 had
  lost (6.61).
- **The residual dy cost vs box12 (+18…63 %) is irreducible, not algorithmic**: the breach
  frames ARE the violent peaks that dominate dy; box12's lower dy was literally purchased with
  black borders. Fit-crop's number is the best dy *achievable inside the true crop budget*.
- 0003 is the one nuance: with only 9 breach frames the global re-solve after tightening cost
  slightly more than the static 7.5° box (0.385 vs 0.360) — both negligible in absolute terms.
- Figure: `figures/l1_fitcrop_dy_vs_borders.png`. Renders: `*_D_cpp_l1fit_4x3.mp4` +
  `*_D_cpp_l1box75_4x3.mp4` alongside the other 4:3 renders.
- **E5 (transient clamp release) is now moot as a primary mechanism** — E4 leaves nothing to
  guard on these clips — but remains the right belt-and-braces default for RS/numeric residuals
  on unseen footage.
- **Realtime**: fit-crop is offline (global CG loop). The rt path composes naturally — run CG
  per receding-horizon window (the window solver already takes per-sample bounds) — future
  work, noted in TODO 0c′.

**E1 quantified (raise max-zoom; reference line only).** box12 + `--max-zoom 170` does give
zero breaches (peak demand 1.62 < 1.70) — but the 4 s zoom envelope holds every impact's deep
crop for seconds, so the cost is not "a brief dip": on the violent clips **48–69 % of all
frames ride above the old 1.30 ceiling** (run0001 48 %, run0002 66 %, 0004 69 %; mean zoom
1.29→1.36 on 0004, p95 1.46–1.54, transient max 1.62 = 38 % narrower view), plus zoom
breathing and digital-zoom softening. The calm clip (0003) is nearly free (1.5 %). Verdict
unchanged: E1 is a reference line, not a mode; zoom-side release survives only as E5's
transient guard.

**E3-as-initializer, rendered (`--l1-auto-box --l1-fit-crop`, "l1auto", all four clips 4:3):**

| clip | fit-crop (box12 init) dy | l1auto (E3 ×0.577 init) dy | border |
|---|---:|---:|---|
| run 0001 | **0.495** | 1.049 (2.1×) | zero ✅ |
| run 0002 | **1.097** | 2.647 (2.4×) | zero ✅ |
| 0003 | 0.385 | **0.295** | zero ✅ |
| 0004 | **5.798** | 7.707 | zero ✅ |

The guarantee holds (all zero-border), but the **1/√3 de-rate hits the wrong axis**: ×0.577
turns the geometric budgets (roll 11.7 / pitch 16.0 / yaw 32.2°) into roll 6.8 / **pitch 9.2** /
yaw 18.6° — and the running bob lives on *pitch*, where 9.2° is tighter than the old box12,
while the opened-up yaw goes unused on run clips. 0003 (pan-flavoured) is the one clip it
helps, confirming the mechanism. **Lesson: E4 is the guarantee, so the initial box should be
generous, not de-rated** — use scale 1.0 (pitch 16° > 12°) and let constraint generation absorb
the axis-combination overflow (pending experiment, TODO 0c′). Renders: `*_D_cpp_l1auto_4x3.mp4`.

**E-ladder final scoreboard (zero-border at 4:3/130 %):**

| rung | verdict |
|---|---|
| E1 raise max-zoom | works, costs 48–69 % of frames above the old ceiling — reference line only |
| E2 static box 7.5° | works, hands back L1's advantage where it binds (+35…180 % dy) — dominated |
| E3 geometric budgets | right idea, wrong de-rate at ×0.577 (pitch-starved); use ×1.0 as E4's initializer |
| **E4 `--l1-fit-crop`** | **the answer**: zero borders, best in-budget dy on 3/4 clips, 3–4 rounds, +1.3–6.9 s |
| E5 transient clamp release | demoted to belt-and-braces guard for unseen footage |

(The EMA-family sibling of E4 — the crop-budget guard `--fit-crop`, its DCR rendered verdict,
and the discovery of Gyroflow's native `max_zoom_iterations` equivalent — is §8r on
`claude/new-cpp-impl`.)

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
