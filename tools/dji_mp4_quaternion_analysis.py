#!/usr/bin/env python3
"""Extract DJI MP4 quaternions through Gyroflow and plot angle/angular velocity curves.

DJI MP4 telemetry uses DJI metadata tracks and protobuf payloads. This script
delegates that extraction to Gyroflow/telemetry-parser, then uses the same
Python quaternion smoothing and angular-velocity helpers as the GCSV script.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from gyro_analysis.gyroflow_export import load_dji_quaternions_from_mp4
from gyro_analysis.math_utils import (
    angular_velocity_from_quats,
    quat_to_euler_xyz_deg,
    smooth_quaternions,
)
from gyro_analysis.plotting import plot_quaternion_angle_velocity, write_quaternion_analysis_csv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mp4", type=Path, help="Input DJI .mp4/.mov file")
    parser.add_argument("-o", "--output", type=Path, help="Output plot path. If omitted, show an interactive window.")
    parser.add_argument("--csv-output", type=Path, help="Optional CSV dump of angles and angular velocity.")
    parser.add_argument("--smooth-ms", type=float, default=250.0, help="Forward/backward quaternion smoothing window in ms.")
    parser.add_argument("--max-points", type=int, default=20000, help="Maximum plotted points per line.")
    parser.add_argument("--gyroflow-bin", help="Gyroflow CLI/app binary used for metadata extraction.")
    parser.add_argument(
        "--camera-data-csv",
        type=Path,
        help="Use an existing Gyroflow camera-data CSV instead of extracting from the MP4.",
    )
    parser.add_argument(
        "--keep-export-csv",
        type=Path,
        help="Path to keep the intermediate Gyroflow camera-data CSV.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    series = load_dji_quaternions_from_mp4(
        args.mp4,
        gyroflow_bin=args.gyroflow_bin,
        camera_data_csv=args.camera_data_csv,
        keep_export_csv=args.keep_export_csv,
    )
    quats = series.quaternions_wxyz
    smoothed_quats = smooth_quaternions(series.timestamps_s, quats, args.smooth_ms)

    euler_deg = quat_to_euler_xyz_deg(quats)
    smoothed_euler_deg = quat_to_euler_xyz_deg(smoothed_quats)
    omega_t_s, omega_rad_s = angular_velocity_from_quats(series.timestamps_s, quats)
    smoothed_omega_t_s, smoothed_omega_rad_s = angular_velocity_from_quats(series.timestamps_s, smoothed_quats)

    title = f"{args.mp4.name}: DJI quaternion angles and angular velocity (smooth={args.smooth_ms:g} ms)"
    plot_quaternion_angle_velocity(
        series.timestamps_s,
        euler_deg,
        smoothed_euler_deg,
        omega_t_s,
        omega_rad_s,
        smoothed_omega_t_s,
        smoothed_omega_rad_s,
        args.output,
        title,
        args.max_points,
    )

    if args.csv_output:
        write_quaternion_analysis_csv(
            args.csv_output,
            series.timestamps_s,
            euler_deg,
            smoothed_euler_deg,
            omega_t_s,
            omega_rad_s,
            smoothed_omega_rad_s,
        )

    quaternion_count = len(series.timestamps_s)
    duration_s = series.timestamps_s[-1] - series.timestamps_s[0]
    intervals_s = np.diff(series.timestamps_s)
    quaternion_rate_hz = (quaternion_count - 1) / duration_s if duration_s > 0 and quaternion_count > 1 else 0.0

    print(f"quaternion samples: {quaternion_count}")
    print(f"quaternion duration: {duration_s:.3f} s")
    print(f"quaternion frequency: {quaternion_rate_hz:.3f} Hz")
    if len(intervals_s):
        median_interval_s = float(np.median(intervals_s))
        median_rate_hz = 1.0 / median_interval_s if median_interval_s > 0 else 0.0
        print(f"quaternion median interval: {median_interval_s * 1000.0:.3f} ms ({median_rate_hz:.3f} Hz)")
    print(f"camera-data csv: {series.source_csv}")
    if args.output:
        print(f"plot: {args.output}")
    if args.csv_output:
        print(f"csv: {args.csv_output}")


if __name__ == "__main__":
    main()
