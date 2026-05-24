from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .math_utils import orient_vec


@dataclass
class GcsvData:
    timestamps_s: np.ndarray
    gyro_raw: np.ndarray
    gyro_rad_s: np.ndarray
    header: dict[str, str]


def read_gcsv(path: Path, respect_orientation: bool = False) -> GcsvData:
    header: dict[str, str] = {}
    data_header: list[str] | None = None
    rows: list[list[str]] = []

    with path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        for raw_row in reader:
            row = [item.strip() for item in raw_row]
            if not row or all(not item for item in row):
                continue
            if data_header is None:
                if row[0] in {"t", "time"}:
                    data_header = row
                    continue
                if len(row) >= 2:
                    header[row[0]] = row[1]
                continue
            rows.append(row)

    if data_header is None:
        raise ValueError("No GCSV data header found. Expected a row starting with 't' or 'time'.")

    time_scale = float(header.get("tscale", "0.001"))
    # telemetry-parser's GCSV path stores Scale = (1 / gscale) * pi/180,
    # then normalized_imu returns degrees/s. SimpleGyro converts back to rad/s,
    # so the effective rad/s value is raw_gcsv_value * gscale.
    gyro_scale_to_rad_s = float(header.get("gscale", "1.0"))

    parsed = np.array([[float(x) for x in row[:4]] for row in rows if len(row) >= 4], dtype=np.float64)
    if parsed.size == 0:
        raise ValueError("No gyro samples found. Expected at least columns: t,gx,gy,gz.")

    timestamps_s = parsed[:, 0] * time_scale
    timestamps_s = timestamps_s - timestamps_s[0]
    gyro_raw = parsed[:, 1:4]
    gyro_rad_s = gyro_raw * gyro_scale_to_rad_s

    if respect_orientation:
        gyro_rad_s = orient_vec(gyro_rad_s, header.get("orientation", "xzY"))

    return GcsvData(
        timestamps_s=timestamps_s,
        gyro_raw=gyro_raw,
        gyro_rad_s=gyro_rad_s,
        header=header,
    )

