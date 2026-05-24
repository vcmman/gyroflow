from __future__ import annotations

import csv
import math
import os
import tempfile
from pathlib import Path

import numpy as np

_cache_root = Path(tempfile.gettempdir()) / "gyroflow_analysis_cache"
_cache_root.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_cache_root / "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(_cache_root / "xdg"))


def decimate_for_plot(*arrays: np.ndarray, max_points: int) -> list[np.ndarray]:
    n = len(arrays[0])
    step = max(1, int(math.ceil(n / max_points)))
    return [arr[::step] for arr in arrays]


def pyplot_for_output(output: Path | None):
    if output:
        import matplotlib

        matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    return plt


def plot_gyro_compare(
    timestamps_s: np.ndarray,
    gyro_rad_s: np.ndarray,
    derived_t_s: np.ndarray,
    derived_rad_s: np.ndarray,
    output: Path | None,
    title: str,
    max_points: int,
) -> None:
    plt = pyplot_for_output(output)
    plot_t, plot_raw = decimate_for_plot(timestamps_s, gyro_rad_s, max_points=max_points)
    plot_dt, plot_derived = decimate_for_plot(derived_t_s, derived_rad_s, max_points=max_points)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(13, 8))
    labels = ["X", "Y", "Z"]
    for i, (axis, label) in enumerate(zip(axes, labels)):
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


def plot_quaternion_angle_velocity(
    timestamps_s: np.ndarray,
    euler_deg: np.ndarray,
    smoothed_euler_deg: np.ndarray,
    omega_t_s: np.ndarray,
    omega_rad_s: np.ndarray,
    smoothed_omega_t_s: np.ndarray,
    smoothed_omega_rad_s: np.ndarray,
    output: Path | None,
    title: str,
    max_points: int,
) -> None:
    plt = pyplot_for_output(output)
    plot_t, plot_euler, plot_sm_euler = decimate_for_plot(
        timestamps_s, euler_deg, smoothed_euler_deg, max_points=max_points
    )
    plot_ot, plot_omega = decimate_for_plot(omega_t_s, omega_rad_s, max_points=max_points)
    plot_sot, plot_sm_omega = decimate_for_plot(smoothed_omega_t_s, smoothed_omega_rad_s, max_points=max_points)

    fig, axes = plt.subplots(6, 1, sharex=True, figsize=(14, 12))
    angle_labels = ["roll", "pitch", "yaw"]
    gyro_labels = ["omega X", "omega Y", "omega Z"]
    for i, label in enumerate(angle_labels):
        axes[i].plot(plot_t, plot_euler[:, i], linewidth=0.8, label=f"raw {label}")
        axes[i].plot(plot_t, plot_sm_euler[:, i], linewidth=0.9, label=f"smoothed {label}")
        axes[i].set_ylabel(f"{label} deg")
        axes[i].grid(True, alpha=0.25)
        axes[i].legend(loc="upper right")

    for i, label in enumerate(gyro_labels):
        axis = axes[i + 3]
        axis.plot(plot_ot, plot_omega[:, i], linewidth=0.8, label=f"raw quat {label}")
        axis.plot(plot_sot, plot_sm_omega[:, i], linewidth=0.9, label=f"smoothed quat {label}")
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


def write_gyro_compare_csv(
    path: Path,
    timestamps_s: np.ndarray,
    raw: np.ndarray,
    derived_t_s: np.ndarray,
    derived: np.ndarray,
) -> None:
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


def write_quaternion_analysis_csv(
    path: Path,
    timestamps_s: np.ndarray,
    euler_deg: np.ndarray,
    smoothed_euler_deg: np.ndarray,
    omega_t_s: np.ndarray,
    omega_rad_s: np.ndarray,
    smoothed_omega_rad_s: np.ndarray,
) -> None:
    euler_interp = np.column_stack([np.interp(omega_t_s, timestamps_s, euler_deg[:, i]) for i in range(3)])
    sm_euler_interp = np.column_stack([np.interp(omega_t_s, timestamps_s, smoothed_euler_deg[:, i]) for i in range(3)])
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "timestamp_s",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "smooth_roll_deg",
                "smooth_pitch_deg",
                "smooth_yaw_deg",
                "quat_omega_x_rad_s",
                "quat_omega_y_rad_s",
                "quat_omega_z_rad_s",
                "smooth_quat_omega_x_rad_s",
                "smooth_quat_omega_y_rad_s",
                "smooth_quat_omega_z_rad_s",
            ]
        )
        for t, e, se, o, so in zip(omega_t_s, euler_interp, sm_euler_interp, omega_rad_s, smoothed_omega_rad_s):
            writer.writerow([f"{t:.9f}", *[f"{v:.9f}" for v in e], *[f"{v:.9f}" for v in se], *[f"{v:.9f}" for v in o], *[f"{v:.9f}" for v in so]])

