#!/usr/bin/env python3
"""Export the COMPLETE DJI quaternion stream from an MP4.

For every IMU/attitude sample this writes:
  - imu_index            running index across the whole clip
  - frame                video frame the sample belongs to
  - frame_timestamp_ms   presentation time of that video frame (image timestamp)
  - quat_timestamp_ms    timestamp of the IMU/quaternion sample itself
  - quat_w/x/y/z         the attitude quaternion

Quaternion extraction is delegated to Gyroflow (telemetry-parser), exactly like
dji_mp4_quaternion_analysis.py. The video frame rate is read straight from the
MP4 video track so frame timestamps are accurate without hard-coding fps.

Examples
--------
  # auto-discover gyroflow on PATH, write next to the video
  python3 tools/dji_export_quaternions_full.py data/clip.MP4 -o data/clip_quats_full.csv

  # reuse an already-exported Gyroflow camera-data CSV (no Gyroflow needed)
  python3 tools/dji_export_quaternions_full.py data/clip.MP4 \
      --camera-data-csv data/clip_camera_data.csv -o data/clip_quats_full.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gyro_analysis.gyroflow_export import export_camera_data_csv  # noqa: E402
from inspect_dji_meta import find_box, read_boxes, parse_trak  # noqa: E402


def video_fps_from_mp4(mp4_path: Path) -> float | None:
    """Frame rate of the first video track, or None if it can't be read."""
    try:
        with mp4_path.open("rb") as f:
            fsize = mp4_path.stat().st_size
            moov = find_box(f, 0, fsize, ["moov"])
            if not moov:
                return None
            mp, me = moov
            for bt, _, p, end in read_boxes(f, mp, me):
                if bt != "trak":
                    continue
                t = parse_trak(f, p, end)
                if t.get("handler") == "vide" and t.get("timescale"):
                    dur_s = t["duration"] / t["timescale"]
                    n = t.get("sample_count", 0)
                    if dur_s > 0 and n > 0:
                        return n / dur_s
    except Exception:
        return None
    return None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mp4", type=Path, help="Input DJI .mp4/.mov file")
    p.add_argument("-o", "--output", type=Path, required=True,
                   help="Output CSV with the full quaternion stream.")
    p.add_argument("--gyroflow-bin", help="Gyroflow CLI/AppImage. Default: auto-discover on PATH / $GYROFLOW_BIN.")
    p.add_argument("--camera-data-csv", type=Path,
                   help="Reuse an existing Gyroflow camera-data CSV instead of re-exporting.")
    p.add_argument("--keep-export-csv", type=Path,
                   help="Where to keep the intermediate Gyroflow camera-data CSV (default: temp dir).")
    p.add_argument("--fps", type=float,
                   help="Override video fps used for frame timestamps (default: read from the MP4).")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # 1. Get the Gyroflow camera-data CSV (frame, timestamp_ms, org_quat_*).
    if args.camera_data_csv:
        cam_csv = args.camera_data_csv
    else:
        cam_csv = export_camera_data_csv(
            args.mp4.resolve(),
            gyroflow_bin=args.gyroflow_bin,
            keep_csv=(args.keep_export_csv.resolve() if args.keep_export_csv else None),
        )

    # 2. Determine the video frame rate for frame (image) timestamps.
    fps = args.fps or video_fps_from_mp4(args.mp4)
    if not fps:
        print("warning: could not read fps from MP4; deriving from frame count/duration", file=sys.stderr)

    # 3. Read every quaternion row and emit the enriched stream.
    rows_out = []
    quat_ts = []
    frames = []
    with Path(cam_csv).open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        required = {"frame", "timestamp_ms", "org_quat_w", "org_quat_x", "org_quat_y", "org_quat_z"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"{cam_csv} missing columns: {', '.join(sorted(missing))}")
        for row in reader:
            frames.append(int(row["frame"]))
            quat_ts.append(float(row["timestamp_ms"]))
            rows_out.append((
                int(row["frame"]),
                float(row["timestamp_ms"]),
                float(row["org_quat_w"]),
                float(row["org_quat_x"]),
                float(row["org_quat_y"]),
                float(row["org_quat_z"]),
            ))

    if not rows_out:
        raise SystemExit(f"No quaternion rows in {cam_csv}")

    # Fallback fps from the data itself if the MP4 read failed.
    if not fps:
        span_s = (quat_ts[-1] - quat_ts[0]) / 1000.0
        fps = (max(frames) + 1) / span_s if span_s > 0 else 30.0
    frame_duration_ms = 1000.0 / fps

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["imu_index", "frame", "frame_timestamp_ms", "quat_timestamp_ms",
                    "quat_w", "quat_x", "quat_y", "quat_z"])
        for i, (frame, qts, qw, qx, qy, qz) in enumerate(rows_out):
            frame_ts = frame * frame_duration_ms
            w.writerow([i, frame, f"{frame_ts:.6f}", f"{qts:.6f}",
                        f"{qw:.8f}", f"{qx:.8f}", f"{qy:.8f}", f"{qz:.8f}"])

    span_s = (quat_ts[-1] - quat_ts[0]) / 1000.0
    print(f"video fps:            {fps:.5f}  (frame duration {frame_duration_ms:.5f} ms)")
    print(f"video frames:         {max(frames) + 1}")
    print(f"quaternion samples:   {len(rows_out)}")
    print(f"quat time span:       {span_s:.3f} s  ({quat_ts[0]:.3f} .. {quat_ts[-1]:.3f} ms)")
    print(f"quaternion frequency: {(len(rows_out) - 1) / span_s:.3f} Hz")
    print(f"source camera-data:   {cam_csv}")
    print(f"output:               {args.output}")


if __name__ == "__main__":
    main()
