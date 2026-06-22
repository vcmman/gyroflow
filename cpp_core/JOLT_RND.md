# Speed-bump jolt R&D — the "大坑" (cpp_core/TODO.md #6)

> **Status: research record — the gated-smoothing code was NOT merged.** The
> `jolt_rejection` / `jolt_z` parameters and the `--jolt-rejection`/`--jolt-z` CLI flags
> described below were a prototype and have been removed from the tree (the smoothing-side
> gate alone trades a shaky jolt for a black-border jolt; see "Next step"). The **measurement
> tooling is kept**: `tools/jolt_analysis.py`, `tools/make_synthetic_jolt.py`, and the
> jerk-RMS / P95 / ITF-P05 metrics in `tools/stabilization_quality.py`. This document is the
> reference for re-implementing the gate as part of crop-constrained joint smooth↔zoom.

## The scenario: a speed-bump jolt

The "大坑" is the **speed-bump jolt** — the case where the camera is moving more or less
normally and then takes a sharp impact: a bicycle (or handheld / car) rolling over a **speed
bump (减速带)**, a curb, a pothole. The body and mount get a quick vertical shock, so the camera
**pitches sharply, overshoots, and rings down** over ~0.1–0.3 s — a brief, high-velocity
*oscillation* (the 抖动), not a single deflection. With two wheels it happens twice (front then
rear). General category: an **impulsive / transient impact**, as opposed to an intentional pan.

We model it as a damped oscillation `A·exp(−(t−t0)/τ)·sin(2πf·(t−t0))` (`make_synthetic_jolt.py
--profile bump`): amplitude `A` (deg), decay `τ`, ring frequency `f`. A speed bump on a bike is
roughly `A ≈ 5–15°`, `f ≈ 5–8 Hz`, `τ ≈ 0.08 s`.

## The pathology

Gyroflow's `default_algo` is a velocity-adaptive low-pass: the slerp `alpha` interpolates
between `max_smoothness` (tight, low velocity) and `alpha_0_1s` (loose ~0.1 s, high velocity),
gated by **normalized angular velocity**. The intent is to follow intentional fast pans. But a
speed-bump jolt is *also* high velocity, so the filter loosens and **follows part of the jolt
through** instead of rejecting it. The residual swing is the visible shake, and it also makes
the adaptive zoom breathe (and, for a large enough sustained excursion, slam into the `max_zoom`
clamp → black borders).

Root cause: velocity alone can't tell a **pan** (high velocity, *steady*) from a **jolt** (a
brief velocity *excursion*).

## How the existing algorithm handles a speed-bump jolt

Measured on the synthetic bump (gentle 10°/s pan + a single pitch impact, `f = 6 Hz`,
`τ = 0.08 s`, `max_zoom = 130%` ⇒ clamp floor fov 0.769), as the bump amplitude grows. "raw"
and "smoothed" are the peak frame-to-frame angular velocity (the impact); pass-through =
smoothed/raw over the worst frames (1 = fully followed, 0 = fully rejected):

| bump amp | raw peak vel | smoothed peak vel | pass-through | min fov | clamp? | fov breathing |
|---:|---:|---:|---:|---:|:--:|---:|
| 4°  |  62°/s | 14°/s | 0.46 | 0.986 | no | 19% |
| 8°  | 122°/s | 20°/s | 0.33 | 0.987 | no | 18% |
| 16° | 243°/s | 30°/s | 0.23 | 0.982 | no | 18% |
| 25° | 380°/s | 38°/s | 0.18 | 0.973 | no | 16% |
| 35° | 532°/s | 42°/s | 0.14 | 0.918 | no | 20% |

(realistic two-wheel 8°+5° bump: pass-through 0.36, min fov 0.985 — same regime.)

**What the existing algorithm does well / badly on this case:**

1. **It attenuates but does not remove the jolt.** It strips ~55–85% of the impact, but a
   clearly visible fraction survives (pass-through 0.46 → 0.14) — the residual smoothed peak
   *grows* with severity (14 → 42°/s vs the ~8°/s baseline). This is the "明显的抖动" the viewer
   sees: a quick bounce that the stabilizer rode along with instead of holding still.
2. **The harder the bump, the smaller the *fraction* it rejects in relative terms but the
   bigger the *absolute* residual.** Because at the impact's high velocity the filter loosens
   (root cause above), and the second-pass distance term only partly compensates.
3. **No black borders in this scenario.** An oscillatory bump returns to baseline quickly, so
   the smoothed path stays near the mean and the residual *angle* the zoom must cover is small —
   min fov stays well above the 0.769 clamp even at 35°. The zoom still **breathes ~15–20%**
   around the impact. (Contrast a *sustained* one-way deflection of the same amplitude, which
   does drive the zoom to the clamp — see E5.)

In short: for the speed-bump jolt the current pipeline produces **a visibly-shaky-but-
border-free** result. It is partial-follow by construction; that is exactly why it avoids the
clamp, and exactly why it leaves residual shake. Removing the shake means rejecting the jolt,
which raises the zoom demand — the trade-off the prototype below ran into.

## Measurement (added this round)

The pre-existing mean-based metrics average jolts away. New transient-sensitive metrics:

- **`tools/stabilization_quality.py`** (image layer) — added shift **P95**, shift **jerk
  (RMS)** = RMS of the frame-to-frame change in the global-motion vector (the metric that
  actually moves when a jolt is removed; a steady pan has ~zero jerk), and **ITF P05** (worst
  consecutive-frame pair).
- **`tools/jolt_analysis.py`** (IMU layer, new) — reads the `gyroflow_cpp_validate` CSV
  (raw `o*` + smoothed `s*` quats + `fov`). Reports raw vs smoothed angular velocity & jerk,
  the **jolt pass-through ratio** (smoothed_vel / raw_vel at the worst raw-velocity frames;
  0 = rejected, 1 = passed straight through), and zoom pumping (fov std/range, deepest
  zoom-ins vs jolt frames).
- **`tools/make_synthetic_jolt.py`** (new) — controlled testbed: a constant-rate pan (which a
  good stabilizer should follow) + an injected impact at a known time (which it should reject).
  Two profiles: `--profile bump` (damped oscillation — the speed-bump model, default) and
  `--profile gaussian` (simple deflect-and-recover). Reuses a real lens profile so undistort/zoom
  run unchanged. This is the development harness — real footage mixes the jolt with continuous
  shake, so the ground truth is unclear.

## The prototype that was tried (NOT merged): jerk/transient-gated smoothing

`DefaultAlgoParams::jolt_rejection` (0 = OFF, **default**, bit-exact Gyroflow parity; the
off-path output is `diff`-identical and ctest stays 6/6). When > 0, the velocity ratio that
drives the adaptive loosening is scaled down where a **transient** is detected, so the filter
stays tight through a jolt.

Transient detector = the angular velocity *not explained by a long-window (sustained-pan)
baseline*: `excess = |v − v_slow|`, scored robustly with **median + MAD** (so continuous shake
— uniformly high excess — is NOT flagged; only statistically anomalous excursions are). Chosen
over instantaneous jerk because for a Gaussian bump the jerk peaks where velocity is *zero*
(impulse center) and is ~zero at the velocity *peaks* — misaligned with where the filter
actually loosens. `excess` self-aligns with the velocity peaks and covers the whole impulse.
A `jolt_z` deadband+ramp (z ≤ jolt_z → 0, z ≥ 2·jolt_z → 1) keeps it off for ordinary motion.

Exposed as `--jolt-rejection 0..1` / `--jolt-z` on both `gyroflow_cpp_validate` and
`gyroflow_cpp_stabilize`.

### Prototype results

**Synthetic** (25° jolt on a 20°/s pan): worst-jolt pass-through **0.34 → 0.09**; but local
min fov at the jolt **0.863 → 0.769** = the `max_zoom` 130% clamp floor → **black borders**.

**Real dji6** (DJI_…_0005, 11 934 frames), strength sweep — top-30 worst-jolt pass-through /
global min fov / smoothed jerk RMS:

| jolt_rejection | pass-through | min fov | max_zoom clamp? | smoothed jerk RMS |
|---:|---:|---:|:--:|---:|
| 0.0 (Gyroflow) | 0.472 | 0.902 | no  | 13.52 |
| 0.3            | 0.395 | 0.894 | no  | **12.37** |
| 0.5            | 0.335 | 0.857 | no  | 14.18 |
| 0.7            | 0.258 | 0.800 | no  | 18.86 |
| 1.0            | 0.114 | 0.769 | **YES** | 30.34 |

**Key findings**

1. The pathology is real on genuine footage: Gyroflow follows ~49 % of the worst jolts.
2. The gate reduces pass-through monotonically and is a **strict net win at ~0.3–0.5**:
   16–29 % less jolt followed, comfortable crop margin (no clamp/borders), and at 0.3 the
   smoothed jerk even *improves*. **Recommended default for the feature: ~0.4.**
3. **Full strength overcorrects two ways**: it eats the crop budget until it hits the
   `max_zoom` clamp (black borders), and it introduces its *own* jerk (RMS 13.5 → 30.3, max
   252 → 978) as the tight filter snaps at the jolt edges.

→ The inherent tension: rejecting a jolt makes the camera steadier but the real footage swung
away, so it **needs more crop** exactly there. Smoothing-side rejection alone just trades a
*shaky* jolt for a *black-border* jolt at high strength.

## Next step

The high-strength regime needs **crop-constrained joint smoothing↔zoom** (TODO #3): limit the
rejection (or per-jolt) to the available crop budget so it rejects as much as the crop allows
without clamping — and shape the gate to avoid the snap-jerk at jolt edges. The current gate is
the building block; making it budget-aware is the next phase (requires coupling smoothing,
which currently runs before zoom, to the zoom margin).

## Reproduce

Analyze the **existing** algorithm on a speed-bump jolt (works with the current tree):

```sh
cmake --build cpp_core/build -j
# bicycle speed bump: gentle pan + front+rear wheel impacts (8°+5°, 0.2 s apart), pitch axis
python3 tools/make_synthetic_jolt.py -o /tmp/bump.json --duration 10 --pan-rate 10 \
    --profile bump --jolt-axis 1,0,0 --jolt-amp 8,5 --jolt-sigma 0.08 --bump-freq 6 --jolt-time 5,5.2
./cpp_core/build/gyroflow_cpp_validate /tmp/bump.json --frames 300 > /tmp/v_bump.csv
python3 tools/jolt_analysis.py /tmp/v_bump.csv --fps 29.97 --plot /tmp/bump.png
```

The **gated-smoothing prototype** below was removed from the tree. To re-run its A/B you must
re-add `jolt_rejection` to `DefaultAlgoParams` + the `--jolt-rejection`/`--jolt-z` flags (this
doc records the algorithm in full); then `--jolt-rejection 0|0.4|1.0` on `gyroflow_cpp_validate`
reproduces the strength sweep, and `gyroflow_cpp_stabilize --jolt-rejection 0.4` renders a clip.

---

## Experiment log (chronological — the path, including dead-ends)

Kept so a future session can see *why* the design is what it is, not just *what* it is. Dates:
2026-06-22.

### E0 — Why a new metric first
The existing `stabilization_quality.py` reports **mean** ITF / shift / flow. A single severe
jolt barely moves a mean over hundreds of frames, yet it is exactly what a viewer notices. Both
`TODO.md` #6 and the `stab-analysis-toolchain` memory independently concluded "quality = RMS
jerk, not mean flow". So step 1 was transient-sensitive metrics (P95, jerk-RMS, ITF-P05) +
an IMU-layer view (`jolt_analysis.py`) that reads raw+smoothed quats from the validate CSV.
Without this we couldn't *score* any fix.

### E1 — First characterization on the local 0032 clip — surprising
Ran `gyroflow_cpp_validate data/dji_bridge.json --frames 100000`. **Trap:** validate generates
exactly `--frames` frames regardless of clip length, so 99 % were padding past the telemetry
end (sampleQuaternion clamps → velocity 0 → P95 = 0.00). The real clip is **973 frames**
(`CAP_PROP_FRAME_COUNT`). Lesson: always pass the true frame count (or omit `--frames`).

Re-run at 973: raw vel mean **76°/s** (very shaky), worst-jolt **pass-through 0.03** — i.e. the
smoother *already rejects* jolts here. Surprising vs the TODO premise. Insight: this clip is
**continuously oscillatory**, not isolated impulses; two-pass smoothing cancels oscillation,
and there is no clean "jolt" to follow. → A real clip is a bad *development* harness because
the ground truth is unclear. Hence E2.

### E2 — Synthetic testbed reproduces the pathology cleanly
`make_synthetic_jolt.py`: constant 25°/s pan + an 8° Gaussian impulse (σ=0.05 s) at mid-clip.
Validate → `jolt_analysis`: **pass-through 0.80** (median 0.67). Clean reproduction — the
velocity-adaptive filter cannot tell the impulse from a deliberate fast move and follows it.
This confirmed the root cause and became the dev harness.

### E3 — Designing the transient detector (two rejected ideas)
- **Hampel / median spike rejection** (TODO candidate #4) — *rejected.* A Hampel filter targets
  single-sample sensor glitches. A *physical* jolt is a smooth ~100 ms bump (~100 samples at
  1 kHz IMU); it is not an outlier at sample scale, so Hampel either misses it or needs a huge
  window. Wrong tool for a physical impact.
- **Instantaneous jerk `|dv/dt|`** — *rejected after analysis.* For a Gaussian angle bump
  θ=A·exp(−t²/2σ²): velocity θ′ is antisymmetric (zero at center, peaks at ±σ); jerk |θ″| peaks
  at the **center** (where velocity is ~0) and at the wings, and is ~**zero at the velocity
  peaks** (±σ). So jerk-gating would tighten the filter exactly where it *isn't* loose and stay
  loose at the velocity peaks. Misaligned.
- **Excess over a long-window baseline `|v − v_slow|`** — *adopted.* A sustained pan tracks its
  own slow baseline (excess≈0); a jolt is a brief excursion above it, large across the **whole**
  impulse including its velocity peaks. Self-aligning. Scored robustly (median+MAD) so uniformly
  shaky clips like 0032 — high *median* excess — are not flagged. Deadband+ramp via `jolt_z`.

### E4 — Implement + parity gate
Added `jolt_rejection` (default 0). Verified the off-path is `diff`-identical to the pre-change
output and ctest stayed 6/6. Synthetic A/B (8° jolt) sweep: pass-through 0.80 → 0.60 → 0.45 →
0.33 for jr 0 → 0.5 → 0.8 → 1.0. Monotonic — the gate works.

### E5 — "Why didn't the zoom change?" (a measurement artifact, then the real trade-off)
On the 8° jolt the global **min fov was unchanged** across all strengths. Investigated the
per-frame residual (`angle(smoothedInv·raw)`) and fov near the jolt: residual at the jolt only
6.6° → 7.4°, fov ≈ 1.15 (barely zoomed); the global min fov 0.811 was at **frame 299** — a
*clip-boundary* artifact of the two-pass smoother on the pan, not the jolt. Two lessons: (a) an
8° jolt is tiny vs a ~120° FOV, so it barely drives zoom; (b) "global min fov" is contaminated
by boundary effects — measure **locally at the jolt**.

Bigger jolt (25°, σ=0.04): pass-through 0.34 → 0.09 (off→full), **local** min fov at the jolt
0.863 → **0.769** = the `max_zoom` 130 % clamp floor → black borders. **This is the real
trade-off**: rejecting the jolt means the footage swung away, so it needs more crop right there.

### E6 — Real dji6 confirmation + strength sweep
Generated the dji6 bridge (`export_bridge_json`, 397 k quats) and ran the full 11 934-frame A/B
and a strength sweep (table above). Confirmed: pathology is real (0.47 pass-through off);
**net-win band jr≈0.3–0.5** (no clamp, no jerk penalty, jerk even improves at 0.3); **full
strength overcorrects** — hits the clamp *and* adds its own snap-jerk (RMS 13.5→30.3) because
the tight filter snaps at the jolt edges. → recommend default ~0.4; high-strength needs the
crop-constrained joint smooth↔zoom (next phase).

### E7 — Re-scope to the speed-bump jolt; re-analyze the existing algorithm (after the prototype was removed)
Clarified that the "大坑" is specifically the **speed-bump jolt** (bicycle over a 减速带): an
*oscillatory* impact (抖动), not the monotonic Gaussian deflection used in E2–E6. Added
`make_synthetic_jolt.py --profile bump` (damped sinusoid) and re-characterized the **stock
Gyroflow algorithm** (no rejection) over an amplitude sweep ("How the existing algorithm handles
a speed-bump jolt" table). Findings: it **partial-follows** the bump (pass-through 0.46→0.14 as
amp 4°→35°), leaving a visible residual (smoothed peak 14→42°/s); **no clamp / no black borders**
because an oscillation returns to baseline so the zoom only breathes ~15–20%. This refines E5:
*sustained* deflections drive the clamp, *oscillatory* speed bumps drive visible residual shake
instead. The two failure modes share the same root cause (velocity-adaptive loosening) and the
same real fix (crop-constrained joint smooth↔zoom).
