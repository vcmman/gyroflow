#!/usr/bin/env python3
"""Angular velocity / acceleration / jerk of the smoothed path, by filter config.

Compares the 1st/2nd/3rd time-derivatives of the smoothed orientation (angular
velocity °/s, acceleration °/s², jerk °/s³) across configs. Perceived smoothness
tracks the higher derivatives; this tool lets you see whether they add information
over one another (SMOOTHING_RND §8f: they give the SAME config ranking — acceleration
is the cleanest discriminator, jerk is redundant and noisier).

Reads gyroflow_cpp_validate CSVs (org quats = raw path, stab quats = smoothed).
Input: repeated --series LABEL CONFIG CSV; --orders "1 2 3" picks the derivative panels.
"""
import os, csv, argparse
os.environ.setdefault("MPLBACKEND", "Agg")
import numpy as np
import matplotlib.pyplot as plt

ORDER_NAME = {1: "angular velocity (°/s)", 2: "angular acceleration (°/s²)", 3: "angular jerk (°/s³)"}


def load(path):
    o, s, ts = [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            o.append([float(r["ow"]), float(r["ox"]), float(r["oy"]), float(r["oz"])])
            s.append([float(r["sw"]), float(r["sx"]), float(r["sy"]), float(r["sz"])])
            ts.append(float(r["ts_ms"]))
    return np.array(o), np.array(s), np.array(ts)


def omega_vec(q, ts_ms):
    """Per-frame angular velocity vector (deg/s) from consecutive quaternions."""
    w0, v0 = q[:-1, 0], q[:-1, 1:]
    w1, v1 = q[1:, 0], q[1:, 1:]
    dw = w0 * w1 + np.sum(v0 * v1, axis=1)                  # conj(q_i)*q_{i+1} scalar part
    dv = w0[:, None] * v1 - w1[:, None] * v0 - np.cross(v0, v1)
    nv = np.linalg.norm(dv, axis=1)
    ang = 2.0 * np.arctan2(nv, np.abs(dw))
    axis = np.divide(dv, nv[:, None], out=np.zeros_like(dv), where=nv[:, None] > 1e-12)
    dt = (np.diff(ts_ms) / 1000.0)[:, None]
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 1.0
    return np.degrees(axis * ang[:, None]) / dt


def deriv_rms(q, ts_ms, order):
    s = omega_vec(q, ts_ms)
    dt = np.median(np.diff(ts_ms) / 1000.0)
    for _ in range(order - 1):
        s = np.diff(s, axis=0) / dt
    return float(np.sqrt(np.mean(np.sum(s ** 2, axis=1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--series", nargs=3, action="append", metavar=("LABEL", "CONFIG", "CSV"),
                    required=True)
    ap.add_argument("--orders", default="1 2 3", help="derivative orders to panel, e.g. \"2 3\"")
    ap.add_argument("-o", "--out", default="angular_derivatives_compare.png")
    args = ap.parse_args()
    orders = [int(x) for x in args.orders.split()]

    clips, configs, val = [], [], {}
    for label, cfg, path in args.series:
        if label not in clips:
            clips.append(label)
        if cfg not in configs:
            configs.append(cfg)
        _o, s, ts = load(path)
        for k in orders:
            val[(label, cfg, k)] = deriv_rms(s, ts, k)

    colors = {"default": "#1f77b4", "DCR": "#2ca02c", "per-axis": "#9467bd"}
    x = np.arange(len(clips)); w = 0.8 / len(configs)
    fig, axes = plt.subplots(1, len(orders), figsize=(6 * len(orders), 5))
    if len(orders) == 1:
        axes = [axes]
    for ax, k in zip(axes, orders):
        for i, cfg in enumerate(configs):
            vals = [val[(c, cfg, k)] for c in clips]
            ax.bar(x + (i - (len(configs) - 1) / 2) * w, vals, w, label=cfg,
                   color=colors.get(cfg))
        ax.set_yscale("log"); ax.set_xticks(x); ax.set_xticklabels(clips, fontsize=8)
        ax.set_ylabel(ORDER_NAME[k] + " RMS (log)"); ax.set_title(ORDER_NAME[k])
        ax.grid(alpha=0.2, axis="y", which="both"); ax.legend(fontsize=8)
    fig.suptitle("Smoothed-path derivatives by config — same ranking at every order "
                 "(acceleration discriminates most; jerk redundant + noisier)", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)

    hdr = f"{'clip':10} {'config':9} " + " ".join(f"{['','vel','accel','jerk'][k]:>9}" for k in orders)
    print(hdr)
    for c in clips:
        for cfg in configs:
            print(f"{c:10} {cfg:9} " + " ".join(f"{val[(c,cfg,k)]:9.0f}" for k in orders))
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
