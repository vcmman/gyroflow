#!/usr/bin/env python3
"""Angular jerk (perceived-smoothness metric) across filter configs.

Jerk = d³θ/dt³ = 2nd time-derivative of the angular velocity vector. Perceived
smoothness tracks jerk (SMOOTHING_RND §4): lower jerk = smoother-looking motion.
Reads gyroflow_cpp_validate CSVs and reports jerk RMS (deg/s³) of the raw camera
path (org quats) and each smoothed config (stab quats), as grouped bars per clip.

Input: repeated --series LABEL CONFIG CSV  (LABEL groups the x-axis clip;
CONFIG is the bar category; the raw path is derived once per clip from org quats).
"""
import os, csv, argparse
os.environ.setdefault("MPLBACKEND", "Agg")
import numpy as np
import matplotlib.pyplot as plt


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
    # relative rotation dq = conj(q_i) * q_{i+1}
    dw = w0 * w1 + np.sum(v0 * v1, axis=1)
    dv = w0[:, None] * v1 - w1[:, None] * v0 - np.cross(v0, v1)
    nv = np.linalg.norm(dv, axis=1)
    ang = 2.0 * np.arctan2(nv, np.abs(dw))                 # rad, [0,pi]
    axis = np.divide(dv, nv[:, None], out=np.zeros_like(dv), where=nv[:, None] > 1e-12)
    rotvec = axis * ang[:, None]
    dt = (np.diff(ts_ms) / 1000.0)[:, None]
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 1.0
    return np.degrees(rotvec) / dt                         # deg/s, (n-1, 3)


def jerk_rms(q, ts_ms):
    w = omega_vec(q, ts_ms)
    dt = np.median(np.diff(ts_ms) / 1000.0)
    accel = np.diff(w, axis=0) / dt                        # deg/s^2
    jerk = np.diff(accel, axis=0) / dt                     # deg/s^3
    return float(np.sqrt(np.mean(np.sum(jerk ** 2, axis=1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--series", nargs=3, action="append", metavar=("LABEL", "CONFIG", "CSV"),
                    required=True)
    ap.add_argument("-o", "--out", default="angular_jerk_compare.png")
    args = ap.parse_args()

    clips, configs, jerk = [], [], {}
    raw = {}
    for label, cfg, path in args.series:
        if label not in clips:
            clips.append(label)
        if cfg not in configs:
            configs.append(cfg)
        o, s, ts = load(path)
        jerk[(label, cfg)] = jerk_rms(s, ts)
        if label not in raw:
            raw[label] = jerk_rms(o, ts)

    cats = ["raw"] + configs
    colors = {"raw": "#999999", "default": "#1f77b4", "DCR": "#2ca02c", "per-axis": "#9467bd"}
    x = np.arange(len(clips))
    w = 0.8 / len(cats)
    fig, ax = plt.subplots(figsize=(12, 6))
    print(f"{'clip':16} {'raw':>10} " + " ".join(f"{c:>10}" for c in configs))
    for i, cat in enumerate(cats):
        vals = [raw[c] if cat == "raw" else jerk[(c, cat)] for c in clips]
        ax.bar(x + (i - (len(cats) - 1) / 2) * w, vals, w, label=cat,
               color=colors.get(cat, None))
    for c in clips:
        row = f"{c:16} {raw[c]:10.0f} " + " ".join(f"{jerk[(c,cfg)]:10.0f}" for cfg in configs)
        print(row)
    ax.set_yscale("log")
    ax.set_xticks(x); ax.set_xticklabels(clips)
    ax.set_ylabel("angular jerk RMS (deg/s³, log scale) — lower = smoother")
    ax.set_title("Angular jerk after filtering, by config (raw for reference)")
    ax.legend(); ax.grid(alpha=0.2, axis="y", which="both")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
