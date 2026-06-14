# DJI MP4 quaternion analysis tools

These tools inspect DJI MP4/MOV telemetry and the quaternion stream that
Gyroflow imports for stabilization. They are intended for debugging DJI clips
whose attitude data is stored in DJI metadata tracks such as `djmd`.

## Requirements

- Python 3.10+
- `numpy` and `matplotlib`
- A Gyroflow CLI/AppImage when exporting the same camera-data CSV that Gyroflow
  uses internally

The scripts discover Gyroflow in this order:

1. `--gyroflow-bin /path/to/gyroflow`
2. `$GYROFLOW_BIN`
3. `gyroflow` or `Gyroflow` on `PATH`

For headless machines, set `MPLBACKEND=Agg` and pass `-o` when plotting.

## Main quaternion plot

```bash
MPLBACKEND=Agg python3 tools/dji_mp4_quaternion_analysis.py \
  /path/to/DJI_clip.MP4 \
  -o /tmp/dji_quat_analysis.png \
  --csv-output /tmp/dji_quat_analysis.csv
```

This script asks Gyroflow to export camera data with `export_all_samples=true`,
then reads `timestamp_ms` and `org_quat_w/x/y/z`. The reported sample count and
rate therefore describe the full Gyroflow-imported quaternion stream, not a
per-video-frame interpolation.

Typical output:

```text
quaternion samples: 32465
quaternion duration: 32.464 s
quaternion frequency: 1000.014 Hz
quaternion median interval: 1.011 ms (989.120 Hz)
```

## Full quaternion CSV

```bash
python3 tools/dji_export_quaternions_full.py \
  /path/to/DJI_clip.MP4 \
  -o /tmp/dji_quaternions_full.csv \
  --keep-export-csv /tmp/dji_camera_data.csv
```

The output CSV contains one row per quaternion sample:

- `imu_index`
- `frame`
- `frame_timestamp_ms`
- `quat_timestamp_ms`
- `quat_w`, `quat_x`, `quat_y`, `quat_z`

Use `--camera-data-csv` to reuse an existing Gyroflow camera-data CSV. Make sure
that CSV was exported with `export_all_samples=true`; otherwise it may only
contain one interpolated quaternion per video frame.

## Direct MP4 metadata checks

These scripts do not require Gyroflow. They are useful for validating whether
the MP4 itself contains a high-rate quaternion array.

```bash
python3 tools/dji_imu_rate.py /path/to/DJI_clip.MP4
```

This walks the `djmd` protobuf payloads, finds repeated quaternion-like
submessages, counts them per metadata sample, and prints the effective attitude
sample frequency.

```bash
python3 tools/inspect_dji_meta.py /path/to/DJI_clip.MP4
```

This lists MP4 tracks, timescales, durations, sample counts, and sample rates.
Add `--dump-first-samples` only when you want to write the first `djmd`/`dbgi`
payloads next to the input file for raw protobuf inspection:

```bash
python3 tools/inspect_dji_meta.py /path/to/DJI_clip.MP4 --dump-first-samples
python3 tools/decode_pb.py /path/to/sample_track3_djmd.bin
```

## Gyroflow stabilization path

For DJI clips that provide imported quaternions, Gyroflow stores them in
`file_metadata.quaternions` and uses integration method `0`. Stabilization does
not reduce this source stream to video frame rate first; it samples/interpolates
the original and smoothed quaternion maps at each frame or rolling-shutter row
timestamp.

That means the source quaternion rate can be around 1000 Hz even when the video
is 29.97 fps. A 29.97 Hz camera-data export usually means the export was made
without `export_all_samples=true`, not that Gyroflow's internal quaternion
stream is only video-rate.

## Common issues

- `Could not find a Gyroflow binary`: install Gyroflow, set `GYROFLOW_BIN`, or
  pass `--gyroflow-bin`.
- `NotAFile("...csv")`: use an absolute output path for Gyroflow exports. The
  helper does this automatically for `--keep-export-csv`.
- `Invalid json: key must be a string`: pass metadata fields in Gyroflow's
  single-quoted CLI style. `gyro_analysis/gyroflow_export.py` handles this.
- Matplotlib display errors on servers: use `MPLBACKEND=Agg` and `-o`.
