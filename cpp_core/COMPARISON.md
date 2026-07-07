# C++ vs Rust Gyroflow — Quantitative Comparison (same parameters)

Head-to-head comparison of the `cpp_core` stabilizer against Rust Gyroflow 1.6.3 on the
sample clip (`data/DJI_20260605174353_0032_D.MP4`, Osmo Action 4), under **identical
parameters**:

| Parameter | Value (both tools) |
|-----------|--------------------|
| Output dimension | 3840×2160 (16:9, lens `output_dimension`) |
| Smoothing | Default algo, smoothness 0.5 / max 1.0 s / alpha_0_1s 0.1 |
| Adaptive zoom | window 4 s, max_zoom 130%, EnvelopeFollower (method 1) |
| Rolling shutter | 21.82 ms, TopToBottom |
| Interpolation | **Bilinear** (Gyroflow forced to bilinear to match the C++ kernel) |
| Source frames | identical 8-bit transcode of the clip (see note) |

> **Source note.** Gyroflow's render is blocked on the original 10-bit file in this
> environment (no GPU encoder; `YUV420P10LE` rejected by the software encoder). To compare
> like-for-like, the clip was transcoded once to 8-bit (`ffmpeg -crf 12 -pix_fmt yuv420p`)
> and **both** tools rendered from that same file (Gyroflow via a project copy whose
> `gyro_source.filepath` still points at the original for telemetry). So both decode
> byte-identical input frames; the comparison isolates the stabilization math + rendering.

Reproduce with the three scripts/tools below (all in-repo).

## 1. Algorithm math vs golden metadata (encoder-independent)

`gyroflow_cpp_validate` dumps the smoothed quaternions + adaptive FOVs; compared against
`gyroflow --export-metadata "3:..."` with `tools/compare_gyroflow_metadata.py`. All 973
frames:

| Quantity | max | mean |
|----------|-----|------|
| smoothed orientation vs `stab_quat` | **0.0147°** | 0.0037° |
| adaptive fov vs `fov_scale` (relative) | **0.0012%** | 0.0004% |
| org_quat sampling (sanity) | 0.0144° | 0.0037° |
| per-frame timestamp | 4.6e-5 ms | 2.7e-5 ms |

→ The smoothing and adaptive-zoom ports (incl. the 16:9 framing) match Gyroflow's golden
output essentially exactly. **RESULT: PASS.**

Both dynamic-zoom methods are validated the same way (export golden with the matching
`adaptive_zoom_method`, then `gyroflow_cpp_validate --zoom-method ...`):

| `adaptive_zoom_method` | adaptive fov vs `fov_scale` (relative) |
|------------------------|----------------------------------------|
| 1 — EnvelopeFollower (default) | max **0.0012%**, mean 0.0004% |
| 0 — GaussianFilter             | max **0.0011%**, mean 0.0004% |

## 2. Frame PSNR (rendered pixels, both bilinear, identical source)

49 frames sampled (every 20th of 973), C++ output vs Gyroflow output:

| Metric | Value |
|--------|-------|
| PSNR | mean **33.46 dB** (min 28.71, max 38.57) |
| mean abs pixel error | 4.11 / 255 (worst frame 6.99) |
| max single-pixel error | 93 |
| global shift (phase-corr) | ~0 px (no translation/framing offset) |

**Attribution of the residual.** A single x264 CRF-12 encode has a ~41 dB noise floor
(mae 1.7); two independent encodes ≈ 38 dB (mae ~2.4). Subtracting that in quadrature leaves
a real geometry residual of mae ≈ 3.3 ≈ **~0.11 px** local sub-pixel misalignment. With both
kernels on bilinear the PSNR rose only ~0.6 dB vs Gyroflow's default Lanczos4 — so the
residual is **not** mainly interpolation. It is dominated by tiny quaternion-sampling
differences (Gyroflow rounds the lookup to integer microseconds; mean 0.0037° ≈ 0.09 px) plus
minor bilinear-resampling implementation differences. There is **no systematic framing
error** (zero global shift, matched brightness). ~0.11 px is at the floor of what's
achievable between two independent codebases through two lossy encoders.

## 3. Stabilization quality (does each remove the same shake?)

`tools/stabilization_quality.py` on the first 300 frames, each output vs the original:

| Metric | original | Gyroflow | C++ |
|--------|---------:|---------:|----:|
| ITF (consecutive-frame PSNR, dB) | 17.57 | 19.32 | 19.15 |
| camera shake — phase-corr shift (px) | 10.47 | 1.16 | 1.16 |
| optical-flow magnitude (px) | 12.59 | 7.54 | 7.59 |

| Improvement | Gyroflow | C++ |
|-------------|---------:|----:|
| camera shake removed | **88.9%** | **89.0%** |
| ITF gain | +1.75 dB | +1.58 dB |

→ The two stabilize **equally well**: identical 88.9% / 89.0% global-shake removal and
near-identical residual optical flow (7.54 vs 7.59 px — both retain the same real scene
motion, the walking subject). The small ITF gap (0.17 dB) reflects the ~0.11 px pixel
residual from §2, not a difference in how much shake is removed.

## 4. Full-clip vertical-shake parity (phaseCorrelate `dy`)

A same-metric head-to-head on the **full length** of two more clips (`dji6_L` 0001 / 0002),
default params on both sides: the Rust render is produced from the exported `.gyroflow` project
(smoothness 0.5, per_axis 0, adaptive zoom envelope, max_zoom 130, 16:9 `output_dimension`) with
CPU x264 + 8-bit `yuv420p` to match the C++ 8-bit decode path; the C++ render is
`*_cpp_stabilized.mp4`. Metric = per-frame global vertical shift `dy`
([`figures/README.md`](figures/README.md#metric-what-phasecorrelate-dy-means)):

| clip | Rust RMS `dy` | C++ RMS `dy` | ratio | per-frame RMS(rust−cpp) | corr |
|---|---:|---:|---:|---:|---:|
| 0001 (2312 f) | 0.675 px | 0.675 px | 1.001 | **0.017 px** | **1.000** |
| 0002 (1487 f) | 1.374 px | 1.369 px | 0.997 | **0.074 px** | 0.998 |

→ The two outputs are **indistinguishable on the actual pixels**: overall vertical-shake
magnitude agrees to ≤0.3 %, and the per-frame `dy` traces track each other to <0.1 px (corr
0.998–1.000). The sub-0.1 px residual is the same non-geometric noise as §2 (8-bit decode:
OpenCV vs libav, x264 encode settings, end-frame alignment 2312 vs 2307), not a stabilization
difference. Figure: `figures/rust_vs_cpp_default_dy.png`.

## Conclusion

Under identical parameters the C++ port is **functionally equivalent** to Rust Gyroflow:
the stabilization math matches golden metadata to ≤0.015°/≤0.0012%, both remove the same
camera shake (88.9% vs 89.0%), and rendered frames agree to ~33 dB with only a ~0.11 px
sub-pixel residual (quaternion-sampling FP + resampling + encode noise), no systematic
framing error.

## Field validation — second clip (dji6_L 0005, full-length run)

Independent of the sample clip above, the port was run end-to-end on a fresh DJI clip and
evaluated for stabilization quality. **Clip:** `dji6_L/DJI_20260617031058_0005_D.MP4`
(DJI Osmo Action 6, 3840×2880 10-bit, 29.97 fps, **11 934 frames / ~6.6 min**, readout
12.98 ms, opencv_fisheye). Telemetry bridged via `tools/export_bridge_json.py` (397 666
quaternions). Two full-length results were rendered (H.265 CRF 18, adaptive zoom, audio
copied):

- 16:9 lens `output_dimension` crop → 3840×2160 (`*_cpp_stab_full.mp4`)
- `--keep-sensor` full 4:3 sensor → 3840×2880 (`*_cpp_stab_full_4x3.mp4`)

**Stability** (`tools/stabilization_quality.py`, first 1800 frames / ~60 s of each), with the
pre-existing Gyroflow render of the same clip and a separate raw clip (`dji6_R 0004`, used
as a steadiness reference) for context:

| Clip (first 1800 frames)     | ITF (dB) ↑ | phase-corr shift (px) ↓ | optical-flow (px) ↓ |
|------------------------------|-----------:|------------------------:|--------------------:|
| L — raw                      |      21.18 |                   3.670 |               3.543 |
| L — **cpp_core** stabilized  |      23.05 |                   1.541 |               3.073 |
| L — **Gyroflow** stabilized  |      23.07 |                   1.484 |               3.006 |
| R — reference (raw, as-is)   |      23.12 |                   1.747 |               2.939 |

→ On the same L footage, **cpp_core and Gyroflow are essentially tied** — Gyroflow steadier
by a sub-pixel margin (shift 1.484 vs 1.541 px, ~3.8%; ITF within 0.02 dB), consistent with
the ~0.11 px residual characterised in §2/§3. Both cut L's global camera shake ~58–60%
(3.670 → ~1.5 px) and end up steadier than the raw R reference on true camera motion.

**Dynamic-zoom methods on this clip.** Both `adaptive_zoom_method`s were also validated
against golden `fov_scale` over the full 11 934 frames of this clip (export golden with the
matching method, then `gyroflow_cpp_validate --zoom-method ...`): EnvelopeFollower max
**0.0047%** / mean 0.0004% rel; GaussianFilter max **0.0047%** / mean 0.0005% rel — both PASS.

> Notes: L vs R are different flights, so cross-clip optical-flow (which also reflects scene
> motion) is not a clean camera-only comparison; phase-corr shift is the most reliable metric
> here. This run used cpp_core's 8-bit OpenCV decode vs Gyroflow's native 10-bit — a
> pixel-fidelity difference, not a geometric one. Outputs + a `stabilization_comparison.md`
> report were saved next to the source clip (external media, not in-repo).

## How to run Gyroflow and reproduce this comparison

Prereqs: a `gyroflow` binary on `PATH` (see the README Quick Start), the built C++ stabilizer
(`cpp_core/build/gyroflow_cpp_stabilize`), `ffmpeg`, and `python3` with `opencv-python`+`numpy`.
Throughout, `CLIP=data/DJI_20260605174353_0032_D` and its `.gyroflow` project + `dji_bridge.json`.

### A. Run Gyroflow on its own (produce a stabilized reference)

Gyroflow's CLI renders a `.gyroflow` project (the project already carries the lens profile,
smoothing, adaptive zoom, and 16:9 output settings):

```sh
gyroflow "$CLIP.gyroflow" -f -p "{'codec':'H.264/AVC','audio':true}"
# -f overwrite; -p out-params (codec/bitrate/use_gpu/audio). Output path is in the project.
```

> **10-bit caveat (this clip / this environment).** Gyroflow may fail with
> `Pixel format YUV420P10LE is not supported` when there's no working GPU encoder and the
> source is 10-bit. Either render on a machine with a GPU encoder, or transcode the source to
> 8-bit first (next section — which is also what the apples-to-apples comparison needs).

### B. Apples-to-apples comparison (both tools read an identical 8-bit source)

```sh
# 0) one-time: 8-bit transcode so both decoders see byte-identical frames
ffmpeg -y -i "$CLIP.MP4" -an -c:v libx264 -crf 12 -pix_fmt yuv420p /tmp/src8.mp4

# build a Gyroflow project that renders the transcode but keeps telemetry from the original,
# and forces Bilinear to match the C++ kernel (helper avoids hand-editing the project JSON):
python3 tools/make_comparison_project.py \
    --project "$CLIP.gyroflow" --transcode /tmp/src8.mp4 --original "$CLIP.MP4" \
    --out /tmp/proj_cmp.gyroflow --output /tmp/gf_cmp.mp4 --interpolation Bilinear

# 1) MATH vs golden metadata (encoder-independent)
gyroflow "$CLIP.gyroflow" --export-metadata "3:/tmp/gf_meta.json"
./cpp_core/build/gyroflow_cpp_validate data/dji_bridge.json --frames 973 > /tmp/cpp.csv
python3 tools/compare_gyroflow_metadata.py /tmp/gf_meta.json /tmp/cpp.csv     # -> PASS

# 2) render BOTH from the same 8-bit source, then FRAME PSNR
gyroflow /tmp/proj_cmp.gyroflow -f -p "{'codec':'H.264/AVC','use_gpu':false,'audio':false}"
./cpp_core/build/gyroflow_cpp_stabilize /tmp/src8.mp4 --telemetry data/dji_bridge.json \
    -o /tmp/cpp_cmp.mp4 --crf 12 --no-audio
python3 tools/frame_psnr.py /tmp/gf_cmp.mp4 /tmp/cpp_cmp.mp4                   # ~33.5 dB

# 3) STABILIZATION QUALITY of each output vs the original (does each remove the same shake?)
python3 tools/stabilization_quality.py "$CLIP.MP4" /tmp/gf_cmp.mp4 /tmp/cpp_cmp.mp4
```

Tools used above (all in `tools/`): `make_comparison_project.py` (wire the 8-bit project),
`compare_gyroflow_metadata.py` (math vs golden), `frame_psnr.py` (rendered-pixel PSNR),
`stabilization_quality.py` (ITF + residual motion). `gyroflow_cpp_validate` lives in
`cpp_core/build/`.
