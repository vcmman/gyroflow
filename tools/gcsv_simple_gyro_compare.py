#!/usr/bin/env python3
"""Compare GCSV gyro values with angular velocity reconstructed from quaternions.

This intentionally mirrors Gyroflow's SimpleGyroIntegrator path:

    omega = [-gyro_y, gyro_x, gyro_z] * pi / 180
    orientation = Rx(90deg)
    orientation = orientation * exp(omega * dt)

The script then applies a simple forward/backward quaternion smoothing pass,
reconstructs angular velocity from adjacent smoothed quaternions, converts it
back to the original GCSV gyro axes, and plots both curves.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_cache_root = Path(tempfile.gettempdir()) / "gyroflow_gcsv_compare_cache"
_cache_root.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_cache_root / "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(_cache_root / "xdg"))


@dataclass
class GcsvData:
    timestamps_s: np.ndarray
    gyro_raw: np.ndarray
    gyro_rad_s: np.ndarray
    header: dict[str, str]


def quat_normalize(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.where(norm > 0.0, norm, 1.0)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )


def quat_inv(q: np.ndarray) -> np.ndarray:
    ret = np.array(q, dtype=np.float64, copy=True)
    ret[..., 1:] *= -1.0
    norm2 = np.sum(q * q, axis=-1, keepdims=True)
    return ret / np.where(norm2 > 0.0, norm2, 1.0)


def quat_from_scaled_axis(v: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = v / angle
    half = 0.5 * angle
    return np.array(
        [math.cos(half), *(axis * math.sin(half))],
        dtype=np.float64,
    )


def quat_slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    a = quat_normalize(a)
    b = quat_normalize(b)
    cos_theta = float(np.dot(a, b))
    if cos_theta < 0.0:
        b = -b
        cos_theta = -cos_theta

    t = max(0.0, min(1.0, t))
    if cos_theta > 0.9995:
        return quat_normalize(a + t * (b - a))

    theta = math.acos(max(-1.0, min(1.0, cos_theta)))
    sin_theta = math.sin(theta)
    wa = math.sin((1.0 - t) * theta) / sin_theta
    wb = math.sin(t * theta) / sin_theta
    return quat_normalize(a * wa + b * wb)


def quat_log_to_omega(q_delta: np.ndarray, dt_s: float) -> np.ndarray:
    q_delta = quat_normalize(q_delta)
    if q_delta[0] < 0.0:
        q_delta = -q_delta

    v = q_delta[1:]
    v_norm = float(np.linalg.norm(v))
    if v_norm < 1e-12 or dt_s <= 0.0:
        return np.zeros(3, dtype=np.float64)

    angle = 2.0 * math.atan2(v_norm, float(q_delta[0]))
    return (v / v_norm) * (angle / dt_s)


def orient_vec(values: np.ndarray, orientation: str) -> np.ndarray:
    if len(orientation) != 3:
        raise ValueError(f"orientation must have 3 characters, got {orientation!r}")

    mapping = {
        "X": values[:, 0],
        "x": -values[:, 0],
        "Y": values[:, 1],
        "y": -values[:, 1],
        "Z": values[:, 2],
        "z": -values[:, 2],
    }
    return np.stack([mapping[c] for c in orientation], axis=1)


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


def integrate_simple_gyro(timestamps_s: np.ndarray, gyro_rad_s: np.ndarray) -> np.ndarray:
    if len(timestamps_s) == 0:
        return np.empty((0, 4), dtype=np.float64)

    # UnitQuaternion::from_euler_angles(FRAC_PI_2, 0, 0)
    orientation = np.array(
        [math.cos(math.pi / 4.0), math.sin(math.pi / 4.0), 0.0, 0.0],
        dtype=np.float64,
    )

    sample_time = (timestamps_s[-1] - timestamps_s[0]) / max(1, len(timestamps_s) - 1)
    prev_time = timestamps_s[0] - sample_time
    quats = np.empty((len(timestamps_s), 4), dtype=np.float64)

    for i, (timestamp, gyro) in enumerate(zip(timestamps_s, gyro_rad_s)):
        # Gyroflow SimpleGyroIntegrator applies this axis shuffle before exp-map integration.
        omega = np.array([-gyro[1], gyro[0], gyro[2]], dtype=np.float64)
        dt = float(timestamp - prev_time)
        delta_q = quat_from_scaled_axis(omega * dt)
        orientation = quat_normalize(quat_mul(orientation, delta_q))
        quats[i] = orientation
        prev_time = timestamp

    return quats


def smooth_quaternions(timestamps_s: np.ndarray, quats: np.ndarray, smooth_ms: float) -> np.ndarray:
    if len(quats) <= 1 or smooth_ms <= 0.0:
        return np.array(quats, copy=True)

    smooth_s = smooth_ms / 1000.0
    out = np.array(quats, copy=True)

    for i in range(1, len(out)):
        dt = max(0.0, timestamps_s[i] - timestamps_s[i - 1])
        alpha = 1.0 - math.exp(-dt / smooth_s)
        out[i] = quat_slerp(out[i - 1], quats[i], alpha)

    for i in range(len(out) - 2, -1, -1):
        dt = max(0.0, timestamps_s[i + 1] - timestamps_s[i])
        alpha = 1.0 - math.exp(-dt / smooth_s)
        out[i] = quat_slerp(out[i + 1], out[i], alpha)

    return out


def angular_velocity_from_quats(timestamps_s: np.ndarray, quats: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if len(quats) < 2:
        return np.empty(0, dtype=np.float64), np.empty((0, 3), dtype=np.float64)

    out_t = np.empty(len(quats) - 1, dtype=np.float64)
    out_gyro = np.empty((len(quats) - 1, 3), dtype=np.float64)

    for i in range(1, len(quats)):
        dt = float(timestamps_s[i] - timestamps_s[i - 1])
        q_delta = quat_mul(quat_inv(quats[i - 1]), quats[i])
        omega_integrator_axes = quat_log_to_omega(q_delta, dt)

        # Invert Gyroflow's SimpleGyro axis shuffle:
        # omega_integrator = [-raw_y, raw_x, raw_z]
        out_gyro[i - 1] = np.array(
            [omega_integrator_axes[1], -omega_integrator_axes[0], omega_integrator_axes[2]],
            dtype=np.float64,
        )
        out_t[i - 1] = 0.5 * (timestamps_s[i - 1] + timestamps_s[i])

    return out_t, out_gyro


def decimate_for_plot(*arrays: np.ndarray, max_points: int) -> list[np.ndarray]:
    n = len(arrays[0])
    step = max(1, int(math.ceil(n / max_points)))
    return [arr[::step] for arr in arrays]


def plot_compare(
    timestamps_s: np.ndarray,
    gyro_rad_s: np.ndarray,
    derived_t_s: np.ndarray,
    derived_rad_s: np.ndarray,
    output: Path | None,
    title: str,
    max_points: int,
) -> None:
    if output:
        import matplotlib

        matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    plot_t, plot_raw = decimate_for_plot(timestamps_s, gyro_rad_s, max_points=max_points)
    plot_dt, plot_derived = decimate_for_plot(derived_t_s, derived_rad_s, max_points=max_points)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(13, 8))
    labels = ["X", "Y", "Z"]
    for axis, label in zip(axes, labels):
        i = labels.index(label)
        axis.plot(plot_t, plot_raw[:, i], linewidth=0.8, label=f"GCSV gyro {label}")
        axis.plot(plot_dt, plot_derived[:, i], linewidth=0.9, label=f"quat-derived {label}")
        axis.set_ylabel(f"{label} rad/s")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right")

    axes[-1].set_xlabel("time (s)")
    fig.suptitle(title)
    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=160)
    else:
        plt.show()


def write_csv(path: Path, timestamps_s: np.ndarray, raw: np.ndarray, derived_t_s: np.ndarray, derived: np.ndarray) -> None:
    raw_interp = np.column_stack(
        [
            np.interp(derived_t_s, timestamps_s, raw[:, 0]),
            np.interp(derived_t_s, timestamps_s, raw[:, 1]),
            np.interp(derived_t_s, timestamps_s, raw[:, 2]),
        ]
    )
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "timestamp_s",
                "raw_gyro_x_rad_s",
                "raw_gyro_y_rad_s",
                "raw_gyro_z_rad_s",
                "quat_gyro_x_rad_s",
                "quat_gyro_y_rad_s",
                "quat_gyro_z_rad_s",
            ]
        )
        for t, r, d in zip(derived_t_s, raw_interp, derived):
            writer.writerow([f"{t:.9f}", *[f"{v:.9f}" for v in r], *[f"{v:.9f}" for v in d]])


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
    derived_t_s, derived_rad_s = angular_velocity_from_quats(data.timestamps_s, smoothed_quats)

    title = (
        f"{args.gcsv.name}: raw gyro vs quaternion-derived gyro "
        f"(Simple gyro, smooth={args.smooth_ms:g} ms)"
    )
    plot_compare(
        data.timestamps_s,
        data.gyro_rad_s,
        derived_t_s,
        derived_rad_s,
        args.output,
        title,
        args.max_points,
    )

    if args.csv_output:
        write_csv(args.csv_output, data.timestamps_s, data.gyro_rad_s, derived_t_s, derived_rad_s)

    print(f"samples: {len(data.timestamps_s)}")
    print(f"duration: {data.timestamps_s[-1] - data.timestamps_s[0]:.3f} s")
    print(f"gscale: {data.header.get('gscale', '1.0')} -> plotted gyro unit: rad/s")
    if args.output:
        print(f"plot: {args.output}")
    if args.csv_output:
        print(f"csv: {args.csv_output}")


if __name__ == "__main__":
    main()
