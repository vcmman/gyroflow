# Evaluation figures — how to reproduce

Image-domain evaluation of the smoothing/zoom configs (vertical shake, black borders,
required-vs-applied zoom). The findings and numbers are written up in
[`../SMOOTHING_RND.md` §8](../SMOOTHING_RND.md); this file is the **how-to-run** for the
three analysis scripts that produced the PNGs here.

All commands are run from the **repo root**. On a headless box prefix Python with
`MPLBACKEND=Agg`.

## What produced each figure

| figure | script | metric |
|---|---|---|
| `vertical_flow_la1_vs_dcr.png` | `tools/vertical_flow_compare.py` | per-frame global vertical shift `dy` (`cv2.phaseCorrelate`) — DCR off vs on |
| `vertical_flow_all_configs.png` | `tools/vertical_flow_compare.py` | same, all five configs overlaid |
| `vertical_flow_all_configs_vs_dji.png` | `tools/vertical_flow_compare.py` | dy vs DJI reference per subplot — run 0001/0002 (default/DCR/gaussian σ0.5) + bike 0005 (default/DCR); default = orange, DJI = black dashed |
| `black_border_stats.png` | `tools/black_border_stats.py` | edge-connected near-black area per frame (mean/max + time series) |
| `zoom_vs_maxzoom.png` | `tools/zoom_vs_maxzoom.py` | required zoom (`1/raw_fov`) vs applied zoom (`1/fov`) vs the `max_zoom` clamp |
| `rust_vs_cpp_default_dy.png` | `tools/rust_vs_cpp_dy.py` | `dy` of Rust vs C++ **default** renders, identical params — port-parity check |
| `tier1_stack_eval.png` | `vertical_flow_compare.py` + `black_border_stats.py` | Tier-1 decision matrix (`default`/DCR/per-axis/combo) — backs `SMOOTHING_RND.md` §8e |
| `dji_headtohead_summary.png` | matched-pair `dy` (see `../EVALUATION_SUMMARY.md`) | DCR vs DJI in-camera, FOV-matched 4:3, run + bike |
| `bike0005_dcr_4x3_vs_dji.png` | `gyro_analysis.video_metrics` | bike 0005: our DCR 4:3 vs DJI 4:3 (16:9 ref) — FOV-matched |
| `bike0005_dcr_vs_dji.png` | `gyro_analysis.video_metrics` | bike 0005: original vs DCR vs DJI (16:9) |
| `angular_velocity_raw_vs_smoothed.png` | `tools/angular_velocity_compare.py` | telemetry-domain: raw gyro vs smoothed (default/DCR) angular velocity, run 0001/0002 + bike |
| `angular_derivatives_compare.png` | `tools/angular_derivatives_compare.py` | angular velocity/accel/jerk RMS by config — same ranking at every order (accel discriminates most, jerk redundant); §8f |
| `dynamic_vs_static_zoom_blackborder.png` | validate `raw_fov`, 5 configs (incl. gaussian + L1) | dynamic vs static zoom black border by config — dynamic ≈0 for all; static exposes each config's crop-demand shape (L1 capped, gaussian peaky); §8c′ |
| `zoom_lookahead_causal_vs_1s.png` | validate `--zoom-look-ahead` (offline/causal/1s) | FOV look-ahead: black border 0% for all modes; 1s look-ahead removes the causal zoom pops (ramps vs snaps); §8h |
| `dy_spectrum_ours_vs_dji.png` | Welch PSD of cached dy | residual spectrum: DJI = one dominant cadence peak (reads "clean", is the un-removed bob); ours lower in every band, 5× lower roughness; §8i |
| `clamp_harmonic_distortion.png` | PSD + zoom of cached dy | double-peak diagnosis + L1 fix: saturation pumps 2nd-harmonic (clamps 0.27–0.37 vs DJI 0.04); L1 joint optimization restores the clean waveform (0.043); §8j-5/§8j-6 |
| `ema_vs_dji_vs_l1_dy.png` | cached dy (0004) | converged three-way: plain EMA steadiest (unbounded class); L1 box12 beats DJI in the bounded class; waveform-cleanliness axis; §8j-8 |
| `deviation_clamp_vs_dji.png` | cached dy + validate CSVs | 3-panel (dy trace / PSD / smoothed-path accel): hard clamp reproduces DJI; SOFT clamp matches DJI’s amplitude with 37 % less roughness (burrs fixed); §8j/§8j-4 |

The first two scripts read **rendered videos**; the third reads **validate CSVs** (no video,
much faster).

## Metric: what `phaseCorrelate dy` means

`dy` is the **per-frame global vertical shift, in pixels, between consecutive frames** of the
rendered (already-stabilized) video — a direct proxy for residual vertical shake / bob.

- **How:** `cv2.phaseCorrelate(prev, cur)` estimates the single global translation `(dx, dy)`
  that best aligns two frames, to sub-pixel precision, by comparing their phase in the frequency
  domain (FFT). We keep the vertical component `dy` and discard `dx`. A Hanning window
  (`createHanningWindow`) is applied first to suppress FFT edge effects.
- **Reading it:** a perfectly steady result has `dy ≡ 0` (the picture doesn't move frame-to-frame);
  larger `|dy|` = more vertical jitter. The **sign** is just direction (up/down). The single
  headline number per config is **`RMS dy`** over all frames — the overall vertical-shake
  magnitude; lower is steadier. This is what ranks the configs in `SMOOTHING_RND.md` §8a.
- **Units / comparability:** frames are resized to a fixed **640 px width** before correlation, so
  `dy` is in pixels *at that analysis scale* (not native 4K). The scale is identical for every
  video (square pixels, same width), so values are directly comparable across configs and across
  the 16:9 cpp renders vs the 4:3 DJI reference.
- **Scope:** phase correlation measures only *global translation* — exactly the whole-frame drift
  that camera shake produces. It does not resolve local object motion, rotation, or zoom; for
  "is the stabilized frame still bobbing as a whole?" that is the right quantity.

## Prerequisites

- Build the C++ core CLIs (`gyroflow_cpp_stabilize` needs OpenCV; `gyroflow_cpp_validate` is
  OpenCV-free):
  ```sh
  cmake -S cpp_core -B cpp_core/build -DCMAKE_BUILD_TYPE=Release
  cmake --build cpp_core/build -j
  ```
- `ffmpeg` on `PATH` (encode) and Python 3.10+ with `numpy`, `opencv-python`, `matplotlib`.
- A bridge-JSON telemetry sidecar per clip (`export_bridge_json.py`, needs a Gyroflow binary):
  ```sh
  python3 tools/export_bridge_json.py INPUT.MP4 -o cpp_out/CLIP_bridge.json
  ```

Below, `RUN` is the working dir holding the input `DJI_*_{0001,0002}_D.MP4` and a `cpp_out/`
subdir with the `{0001,0002}_bridge.json` sidecars (in this checkout:
`/media/yc/Seagate Backup Plus Drive/dji6/dji6_L/run`).

## Step 1 — render the configs (needed for the two video-domain figures)

The scripts locate renders by the exact name `cpp_out/{CLIP}_D_cpp_stabilized{SUFFIX}.mp4`.
Render each config (default 16:9 framing, envelope adaptive zoom, max_zoom 130 %):

```sh
BIN=cpp_core/build/gyroflow_cpp_stabilize
for CLIP in 0001 0002; do
  IN="$RUN"/DJI_*_${CLIP}_D.MP4
  B="$RUN/cpp_out/${CLIP}_bridge.json"
  O="$RUN/cpp_out/${CLIP}_D_cpp_stabilized"
  $BIN $IN --telemetry "$B"                    -o "${O}.mp4"          # default (offline)
  $BIN $IN --telemetry "$B" --dcr              -o "${O}_dcr.mp4"      # DCR
  $BIN $IN --telemetry "$B" --look-ahead 1     -o "${O}_la1.mp4"      # DCR off + 1 s look-ahead
  $BIN $IN --telemetry "$B" --dcr --look-ahead 1 -o "${O}_dcr_la1.mp4" # DCR + 1 s look-ahead
done
```

> **L1** (`_l1.mp4`) is produced on a different branch (`claude/speed-bump-jolt-rnd`,
> `--smoothing l1 --l1-match-default`); it is not reproducible on this branch. If the `_l1.mp4`
> files are absent the scripts just skip that config.

## Step 2 — export zoom CSVs (needed for `zoom_vs_maxzoom.png`)

`gyroflow_cpp_validate` recomputes the smoothing + adaptive zoom from the bridge JSON (no video)
and, since this branch, emits a `raw_fov` column — the per-frame **required** FOV before temporal
smoothing and the `max_zoom` clamp. The zoom script expects `cpp_out/zoom_{CLIP}_{CFG}.csv`:

```sh
VAL=cpp_core/build/gyroflow_cpp_validate
for CLIP in 0001 0002; do
  B="$RUN/cpp_out/${CLIP}_bridge.json"; Z="$RUN/cpp_out/zoom_${CLIP}"
  $VAL "$B"                    > "${Z}_default.csv"
  $VAL "$B" --dcr              > "${Z}_dcr.csv"
  $VAL "$B" --look-ahead 1     > "${Z}_la1.csv"
  $VAL "$B" --dcr --look-ahead 1 > "${Z}_dcr_la1.csv"
done
```

## Step 3 — generate the figures

```sh
CPP_OUT="$RUN/cpp_out"

# Vertical shake vs DJI reference (--ref per clip). --configs filters the --dir pattern
# suffixes ("" = default, "_dcr" = DCR, ...); --series adds explicit files, either extra
# lines on a pattern clip (e.g. the gaussian run renders) or a whole new subplot (bike).
# --cache makes plot iterations instant (per-video dy .npy).
MPLBACKEND=Agg python3 tools/vertical_flow_compare.py --dir "$CPP_OUT" --width 640 \
  --cache /tmp/dycache --configs ",_dcr" \
  --series 0001 "gaussian σ0.5" "$CPP_OUT/0001_D_cpp_gauss05.mp4" \
  --series 0002 "gaussian σ0.5" "$CPP_OUT/0002_D_cpp_gauss05.mp4" \
  --ref 0001 "$DJI_REF_DIR/DJI_20260625014752_0002_D.MP4" \
  --ref 0002 "$DJI_REF_DIR/DJI_20260625014927_0004_D.MP4" \
  --series bike0005 "default (offline)" "$BIKE_DIR/0005_D_cpp_default.mp4" \
  --series bike0005 "DCR (offline)"     "$BIKE_DIR/DJI_..._0005_D_cpp_dcr.mp4" \
  --ref bike0005 "$DJI_REF_BIKE/DJI_20260617031609_0004_D.MP4" \
  -o cpp_core/figures/vertical_flow_all_configs_vs_dji.png

# Black-border statistics (stride-sampled for speed)
MPLBACKEND=Agg python3 tools/black_border_stats.py --dir "$CPP_OUT" --width 480 --stride 5 --thresh 8 \
  -o cpp_core/figures/black_border_stats.png

# Required vs applied zoom vs max_zoom clamp (reads the validate CSVs)
MPLBACKEND=Agg python3 tools/zoom_vs_maxzoom.py --dir "$CPP_OUT" --max-zoom 1.30 \
  -o cpp_core/figures/zoom_vs_maxzoom.png

# Rust-vs-C++ default parity (needs a Rust render per clip first — see ../COMPARISON.md §4)
MPLBACKEND=Agg python3 tools/rust_vs_cpp_dy.py \
  --pair 0001 "$RUN/DJI_..._0001_D_rust_default.mp4" "$CPP_OUT/0001_D_cpp_stabilized.mp4" \
  --pair 0002 "$RUN/DJI_..._0002_D_rust_default.mp4" "$CPP_OUT/0002_D_cpp_stabilized.mp4" \
  -o cpp_core/figures/rust_vs_cpp_default_dy.png
```

Each script also prints a summary table to stdout (RMS `dy` per config; black-border
mean/p99/max/%frames; max required zoom + clamp-breach frame counts).

### Notes

- The DJI reference overlay is supplied per clip with `--ref CLIP VIDEO` (repeatable); omit the
  flag to plot the configs only. On this machine the references live under
  `/media/yc/Seagate Backup Plus Drive/dji6/dji6_R/run/`.
- `dy` is measured in pixels at the analysis width (640 px), square pixels, so values are
  comparable across the 16:9 cpp renders and the 4:3 DJI reference.
- Black borders are geometrically forced **only** where required zoom exceeds `max_zoom`
  (see `zoom_vs_maxzoom.png`); the near-black pixel counter in `black_border_stats.png`
  additionally picks up dark scene content and the thin undistort-warp edge (§8b).

## Figures

### Vertical shake — all configs vs DJI in-camera
![vertical flow all configs vs DJI](vertical_flow_all_configs_vs_dji.png)

### Vertical shake — all configs
![vertical flow all configs](vertical_flow_all_configs.png)

### Vertical shake — DCR off vs on (1 s look-ahead)
![vertical flow la1 vs dcr](vertical_flow_la1_vs_dcr.png)

### Black-border statistics
![black border stats](black_border_stats.png)

### Required vs applied zoom vs max_zoom clamp
![zoom vs max_zoom](zoom_vs_maxzoom.png)

### Tier-1 stack evaluation (DCR vs per-axis vs combo)
Rendered `dy` + black border across the candidate default-enhancement configs; confirms **DCR
alone** (`--enhanced`) and excludes per-axis. Full table in
[`../SMOOTHING_RND.md` §8e](../SMOOTHING_RND.md).
![tier1 stack eval](tier1_stack_eval.png)

### DCR vs DJI in-camera — matched scene, FOV-matched 4:3
Head-to-head against DJI RockSteady on matched same-scene pairs (see
[`../EVALUATION_SUMMARY.md`](../EVALUATION_SUMMARY.md) §6): DCR ~4× steadier on running (violent
bob), DJI ~1.3× steadier on smooth biking (frame-periphery residual).
![dji head-to-head](dji_headtohead_summary.png)

### Camera angular velocity — raw vs smoothed (telemetry domain)
How the smoothing (and DCR) attenuates the raw camera angular velocity while keeping intentional
motion. Raw→DCR RMS: run0001 58→16 °/s (−72 %), run0002 87→21 °/s (−76 %), bike 28→7 °/s (−76 %).
![angular velocity raw vs smoothed](angular_velocity_raw_vs_smoothed.png)

### Smoothed-path derivatives by config (velocity / acceleration / jerk)
RMS (log) of the 1st/2nd/3rd derivatives. All three rank configs identically (per-axis < DCR <
default), so **jerk is redundant**; **acceleration** discriminates most (default→DCR −33/−54/−12 %).
See [`../SMOOTHING_RND.md` §8f](../SMOOTHING_RND.md).
![angular derivatives compare](angular_derivatives_compare.png)

### Rust vs C++ default — port parity (`dy`)
Same-metric head-to-head of the Rust and C++ **default** renders under identical params; the
traces overlap (RMS agrees ≤0.3 %, per-frame corr 0.998–1.000). Reproduce steps in
[`../COMPARISON.md` §4](../COMPARISON.md).
![rust vs cpp default dy](rust_vs_cpp_default_dy.png)
