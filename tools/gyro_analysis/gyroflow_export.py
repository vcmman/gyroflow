from __future__ import annotations

import csv
import json
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class QuaternionSeries:
    timestamps_s: np.ndarray
    quaternions_wxyz: np.ndarray
    source_csv: Path


def resolve_gyroflow_binary(explicit: str | None = None) -> str:
    candidates = [
        explicit,
        os.environ.get("GYROFLOW_BIN"),
        "gyroflow",
        "Gyroflow",
    ]
    for candidate in candidates:
        if not candidate:
            continue
        candidate_path = Path(candidate)
        if candidate_path.suffix == ".app":
            app_binary = candidate_path / "Contents" / "MacOS" / "Gyroflow"
            if app_binary.exists():
                return str(app_binary)
        if candidate_path.exists():
            return candidate
        resolved = shutil.which(candidate)
        if resolved:
            return resolved
    raise FileNotFoundError(
        "Could not find a Gyroflow binary. Pass --gyroflow-bin or set GYROFLOW_BIN. "
        "The script uses Gyroflow/telemetry-parser to extract DJI quaternion metadata from MP4."
    )


def export_camera_data_csv(mp4_path: Path, gyroflow_bin: str | None = None, keep_csv: Path | None = None) -> Path:
    gyroflow = resolve_gyroflow_binary(gyroflow_bin)
    out_csv = keep_csv or Path(tempfile.gettempdir()) / f"{mp4_path.stem}_gyroflow_camera_data.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    fields = {
        "original": {
            "gyroscope": False,
            "accelerometer": False,
            "quaternion": True,
            "euler_angles": True,
        },
        "stabilized": {
            "quaternion": False,
            "euler_angles": False,
        },
        "zooming": {
            "minimal_fovs": False,
            "fovs": False,
            "focal_length": False,
        },
    }
    cmd = [
        gyroflow,
        str(mp4_path),
        "--export-metadata",
        f"3:{out_csv}",
        "--export-metadata-fields",
        json.dumps(fields),
    ]
    subprocess.run(cmd, check=True)
    return out_csv


def read_gyroflow_camera_csv(path: Path) -> QuaternionSeries:
    timestamps: list[float] = []
    quats: list[list[float]] = []

    with path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        required = {"timestamp_ms", "org_quat_w", "org_quat_x", "org_quat_y", "org_quat_z"}
        missing = required - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path} is missing fields: {', '.join(sorted(missing))}")

        for row in reader:
            timestamps.append(float(row["timestamp_ms"]) / 1000.0)
            quats.append(
                [
                    float(row["org_quat_w"]),
                    float(row["org_quat_x"]),
                    float(row["org_quat_y"]),
                    float(row["org_quat_z"]),
                ]
            )

    if not timestamps:
        raise ValueError(f"No quaternion rows found in {path}")

    timestamps_s = np.array(timestamps, dtype=np.float64)
    timestamps_s = timestamps_s - timestamps_s[0]
    quaternions = np.array(quats, dtype=np.float64)
    return QuaternionSeries(timestamps_s=timestamps_s, quaternions_wxyz=quaternions, source_csv=path)


def load_dji_quaternions_from_mp4(
    mp4_path: Path,
    gyroflow_bin: str | None = None,
    camera_data_csv: Path | None = None,
    keep_export_csv: Path | None = None,
) -> QuaternionSeries:
    if camera_data_csv:
        return read_gyroflow_camera_csv(camera_data_csv)
    exported = export_camera_data_csv(mp4_path, gyroflow_bin=gyroflow_bin, keep_csv=keep_export_csv)
    return read_gyroflow_camera_csv(exported)
