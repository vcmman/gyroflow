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

### Candidate work items
1. Translation-domain residual stabilization (the actual visible-float fix).
2. Port a Gaussian (or linear-phase) base kernel as an alternative smoother; re-measure on renders.
3. Mean-fov-matched L1 vs DCR vs Gaussian comparison (telemetry-fast).
4. Add a jerk-RMS metric to `stabilization_quality.py`.

### Reproduce
```sh
# camera-path CSV per mode (this branch: default/DCR; jolt branch: --smoothing l1 --l1-match-default)
gyroflow_cpp_validate BRIDGE.json --frames N [--dcr]      > path.csv
# metrics: convert smoothed quat -> euler pitch; band RMS (0.5-5 Hz bob / <0.5 Hz intent);
#          jerk = 3rd time-difference RMS; crop = max|smoothed-raw| or mean fov.
# image-domain: cv2.phaseCorrelate consecutive frames -> integrate dy -> bob-band RMS.
```
