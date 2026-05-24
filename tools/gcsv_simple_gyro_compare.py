#!/usr/bin/env python3
"""Read GCSV, integrate with Gyroflow Simple gyro, smooth quats, and compare gyro curves."""

from __future__ import annotations

import argparse
from pathlib import Path

from gyro_analysis.gcsv import read_gcsv
from gyro_analysis.math_utils import (
    angular_velocity_from_simple_gyro_quats,
    integrate_simple_gyro,
    smooth_quaternions,
)
from gyro_analysis.plotting import plot_gyro_compare, write_gyro_compare_csv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gcsv", type=Path, help="Input .gcsv file")
    parser.add_argument("-o", "--output", type=Path, help="Output plot path. If omitted, show an interactive window.")
    parser.add_argument("--csv-output", type=Path, help="Optional CSV dump of raw and reconstructed gyro values.")
    parser.add_argument("--smooth-ms", type=float, default=250.0, help="Forward/backward quaternion smoothing window in ms.")
    parser.add_argument("--max-points", type=int, default=20000, help="Maximum plotted points per line.")
    parser.add_argument(
        "--respect-gcsv-orientation",
        action="store_true",
        help="Apply the GCSV orientation header. Gyroflow's current load path forces XYZ before later UI transforms, so this is off by default.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    data = read_gcsv(args.gcsv, respect_orientation=args.respect_gcsv_orientation)
    quats = integrate_simple_gyro(data.timestamps_s, data.gyro_rad_s)
    smoothed_quats = smooth_quaternions(data.timestamps_s, quats, args.smooth_ms)
    derived_t_s, derived_rad_s = angular_velocity_from_simple_gyro_quats(data.timestamps_s, smoothed_quats)

    title = (
        f"{args.gcsv.name}: raw gyro vs quaternion-derived gyro "
        f"(Simple gyro, smooth={args.smooth_ms:g} ms)"
    )
    plot_gyro_compare(
        data.timestamps_s,
        data.gyro_rad_s,
        derived_t_s,
        derived_rad_s,
        args.output,
        title,
        args.max_points,
    )

    if args.csv_output:
        write_gyro_compare_csv(args.csv_output, data.timestamps_s, data.gyro_rad_s, derived_t_s, derived_rad_s)

    print(f"samples: {len(data.timestamps_s)}")
    print(f"duration: {data.timestamps_s[-1] - data.timestamps_s[0]:.3f} s")
    print(f"gscale: {data.header.get('gscale', '1.0')} -> plotted gyro unit: rad/s")
    if args.output:
        print(f"plot: {args.output}")
    if args.csv_output:
        print(f"csv: {args.csv_output}")


if __name__ == "__main__":
    main()
