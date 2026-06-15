#!/usr/bin/env python3
"""Export a Phase-1 "bridge" telemetry JSON for the C++ stabilizer (cpp_core).

The C++ port does not yet parse DJI djmd/DVTM protobuf. This script bridges the gap by
reusing the Rust Gyroflow binary to obtain the lens profile + rolling-shutter readout time
(via a .gyroflow project export) and the org/raw attitude quaternions (via a camera-data
CSV export), then emits the JSON schema documented in cpp_core/include/gyroflow/telemetry_io.hpp.

If a project file and/or camera CSV already exist they are reused as-is; otherwise Gyroflow
is invoked to produce them (see tools/gyro_analysis/gyroflow_export.py for the CLI used).

Example:
    python3 tools/export_bridge_json.py data/DJI_..._0032_D.MP4 \
        --project data/DJI_..._0032_D.gyroflow \
        --camera-csv data/dji_camera_data.csv \
        -o data/dji_bridge.json
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gyro_analysis.gyroflow_export import export_camera_data_csv, resolve_gyroflow_binary  # noqa: E402


def export_project(mp4: Path, gyroflow_bin: str | None) -> Path:
    import subprocess

    gyroflow = resolve_gyroflow_binary(gyroflow_bin)
    out = mp4.with_suffix(".gyroflow")
    subprocess.run([gyroflow, str(mp4), "--export-project", "1"], check=True)
    if not out.exists():
        raise FileNotFoundError(f"Expected project file not produced: {out}")
    return out


def read_org_quaternions(camera_csv: Path) -> list[list[float]]:
    """Return [[t_us, w, x, y, z], ...] from a Gyroflow camera-data CSV."""
    rows: list[list[float]] = []
    with camera_csv.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        required = {"timestamp_ms", "org_quat_w", "org_quat_x", "org_quat_y", "org_quat_z"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{camera_csv} missing columns: {', '.join(sorted(missing))}")
        for row in reader:
            t_us = float(row["timestamp_ms"]) * 1000.0
            rows.append([
                t_us,
                float(row["org_quat_w"]),
                float(row["org_quat_x"]),
                float(row["org_quat_y"]),
                float(row["org_quat_z"]),
            ])
    if not rows:
        raise ValueError(f"No quaternion rows in {camera_csv}")
    return rows


def build_bridge(project: dict, quaternions: list[list[float]]) -> dict:
    vid = project.get("video_info", {})
    cal = project.get("calibration_data", {})
    stab = project.get("stabilization", {})
    fp = cal.get("fisheye_params", {}) or {}

    cam = fp.get("camera_matrix")
    coeffs = fp.get("distortion_coeffs", [])

    readout = stab.get("frame_readout_time") or cal.get("frame_readout_time") or 0.0
    direction = stab.get("frame_readout_direction") or cal.get("frame_readout_direction") or "TopToBottom"

    model = cal.get("distortion_model") or "opencv_fisheye"

    calib = cal.get("calib_dimension", {}) or {}
    out_dim = cal.get("output_dimension", {}) or {}

    lens = {
        "camera_brand": cal.get("camera_brand", ""),
        "camera_model": cal.get("camera_model", ""),
        "lens_model": cal.get("lens_model", ""),
        "calib_width": calib.get("w", vid.get("width", 0)),
        "calib_height": calib.get("h", vid.get("height", 0)),
        "output_width": (out_dim or calib).get("w", vid.get("width", 0)),
        "output_height": (out_dim or calib).get("h", vid.get("height", 0)),
        "camera_matrix": cam,
        "distortion_model": model,
        "distortion_coeffs": coeffs[:4],
        "frame_readout_time_ms": readout,
    }

    return {
        "detected_source": " ".join(x for x in [cal.get("camera_brand", ""), cal.get("camera_model", "")] if x).strip()
        or project.get("title", ""),
        "fps": vid.get("fps", 0.0),
        "width": vid.get("width", 0),
        "height": vid.get("height", 0),
        "frame_readout_time_ms": readout,
        "frame_readout_direction": direction,
        "quaternions": quaternions,
        "lens_profile": lens,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mp4", nargs="?", type=Path, help="DJI MP4 (optional if --project and --camera-csv given)")
    ap.add_argument("--project", type=Path, help="existing .gyroflow project file")
    ap.add_argument("--camera-csv", type=Path, help="existing Gyroflow camera-data CSV")
    ap.add_argument("--gyroflow-bin", help="path to gyroflow binary")
    ap.add_argument("-o", "--output", type=Path, required=True, help="output bridge JSON path")
    args = ap.parse_args()

    project_path = args.project
    if project_path is None:
        if args.mp4 is None:
            ap.error("either --project or the mp4 positional argument is required")
        project_path = export_project(args.mp4, args.gyroflow_bin)

    camera_csv = args.camera_csv
    if camera_csv is None:
        if args.mp4 is None:
            ap.error("either --camera-csv or the mp4 positional argument is required")
        camera_csv = export_camera_data_csv(args.mp4, gyroflow_bin=args.gyroflow_bin)

    project = json.loads(Path(project_path).read_text(encoding="utf-8"))
    quaternions = read_org_quaternions(Path(camera_csv))
    bridge = build_bridge(project, quaternions)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bridge), encoding="utf-8")
    print(f"Wrote {args.output} "
          f"({bridge['width']}x{bridge['height']} @ {bridge['fps']} fps, "
          f"{len(quaternions)} quaternions, readout {bridge['frame_readout_time_ms']:.3f} ms, "
          f"model {bridge['lens_profile']['distortion_model']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
