# `tools/` — DJI telemetry & cpp_core evaluation scripts

Python 3.10+ helper scripts (need `numpy`, `matplotlib`, and — for the image-domain metrics —
`opencv-python`). On a headless box set `MPLBACKEND=Agg`. Run everything **from the repo root**
(`python3 tools/<script>.py …`); the scripts import the shared `gyro_analysis` package by relative
name, which resolves because `tools/` is on `sys.path[0]` when a script there is run directly.

Several scripts shell out to a built **Gyroflow** binary; they locate it via `--gyroflow-bin`,
`$GYROFLOW_BIN`, or `gyroflow`/`Gyroflow` on `PATH`.

These are **not** part of upstream Gyroflow — they support the DJI-quaternion analysis and the
`cpp_core/` C++ port (see `cpp_core/README.md`, `cpp_core/COMPARISON.md`, `cpp_core/SMOOTHING_RND.md`,
and `cpp_core/figures/README.md`).

## 1. DJI telemetry extraction & analysis
Full command reference: [`README_DJI_quaternion.md`](README_DJI_quaternion.md).

| script | purpose |
|---|---|
| `inspect_dji_meta.py` | minimal MP4 parser: inspect DJI metadata tracks, estimate sample rates |
| `decode_pb.py` | walk raw protobuf wire format (no schema) and summarize structure |
| `dji_imu_rate.py` | estimate DJI IMU / attitude sample frequency from the MP4 metadata track |
| `dji_export_quaternions_full.py` | export the COMPLETE DJI fused-quaternion stream from an MP4 |
| `dji_mp4_quaternion_analysis.py` | extract quaternions via Gyroflow, plot angle / angular-velocity curves |
| `gcsv_simple_gyro_compare.py` | integrate a GCSV with the Gyroflow "Simple gyro", compare curves |

## 2. cpp_core bridge & golden validation

| script | purpose |
|---|---|
| `export_bridge_json.py` | export the Phase-1 "bridge" telemetry JSON (fused quats + lens + readout) the C++ stabilizer consumes |
| `compare_gyroflow_metadata.py` | validate `gyroflow_cpp_validate` output against Rust Gyroflow's golden per-frame metadata (math parity) |
| `make_comparison_project.py` | build a `.gyroflow` project for an apples-to-apples C++-vs-Gyroflow render (8-bit source, Bilinear) |
| `frame_psnr.py` | frame-by-frame PSNR / pixel error between two equally-sized, frame-aligned videos |

## 3. Image-domain stabilization evaluation
Metric definitions + reproduce commands: [`../cpp_core/figures/README.md`](../cpp_core/figures/README.md).

| script | purpose |
|---|---|
| `stabilization_quality.py` | how *stable* a video is (ITF + residual inter-frame motion), before vs after |
| `vertical_flow_compare.py` | per-frame vertical shift `dy` (`phaseCorrelate`) across smoothing configs (+ optional DJI reference) |
| `black_border_stats.py` | edge-connected near-black area per frame (mean / max + time series) |
| `zoom_vs_maxzoom.py` | required zoom (`1/raw_fov`) vs applied zoom vs the `max_zoom` clamp, from `gyroflow_cpp_validate` CSVs |
| `rust_vs_cpp_dy.py` | Rust-vs-C++ **default**-render parity via `dy` (backs `COMPARISON.md` §4) |

## Shared package — `gyro_analysis/`

| module | contents |
|---|---|
| `math_utils.py` | quaternion / euler / integration helpers |
| `gyroflow_export.py` | drive a Gyroflow binary, load its exported camera-data CSV / quaternions |
| `gcsv.py` | read GCSV gyro logs |
| `plotting.py` | shared quaternion angle/velocity plots + CSV writers |
| `video_metrics.py` | image-domain video metrics: `vertical_flow` (phaseCorrelate `dy`) and `edge_black_series` (border-connected black area) — shared by the §3 scripts |
