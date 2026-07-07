#!/usr/bin/env python3
"""Overlay smoothed vs raw camera angular velocity per config.

Reads gyroflow_cpp_validate CSVs (org_quat ow..oz = raw fused attitude,
stab_quat sw..sz = smoothed) and plots the angular-velocity magnitude (deg/s)
of the raw camera motion against the smoothed path under each config — showing
how the smoothing (and DCR) attenuates shake while keeping intentional motion.

Angular speed between consecutive orientations q_i, q_{i+1}:
    angle = 2*acos(|<q_i, q_{i+1}>|)   (geodesic),  speed = angle / dt.
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


def ang_speed(q, ts_ms):
    d = np.abs(np.sum(q[:-1] * q[1:], axis=1)).clip(0.0, 1.0)   # |<q_i,q_{i+1}>|
    ang = 2.0 * np.arccos(d)                                    # rad
    dt = np.diff(ts_ms) / 1000.0
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 1.0
    return np.degrees(ang) / dt                                 # deg/s, length n-1


def best_window(sig, win):
    """Offset of the `win`-length slice with the highest RMS (most active)."""
    if len(sig) <= win:
        return 0, len(sig)
    csum = np.cumsum(np.concatenate([[0.0], sig ** 2]))
    rms = csum[win:] - csum[:-win]
    i = int(np.argmax(rms))
    return i, i + win


def main():
    ap = argparse.ArgumentParser()
    # each --clip: LABEL DEFAULT_CSV DCR_CSV
    ap.add_argument("--clip", nargs=3, action="append", metavar=("LABEL", "DEFAULT_CSV", "DCR_CSV"),
                    required=True)
    ap.add_argument("--window", type=int, default=600, help="frames shown (most-active slice)")
    ap.add_argument("-o", "--out", default="angular_velocity_compare.png")
    args = ap.parse_args()

    fig, axes = plt.subplots(len(args.clip), 1, figsize=(15, 4.5 * len(args.clip)))
    if len(args.clip) == 1:
        axes = [axes]
    for ax, (label, dflt, dcr) in zip(axes, args.clip):
        o, s_d, ts = load(dflt)
        _o2, s_r, _ts2 = load(dcr)
        raw = ang_speed(o, ts)
        sm_default = ang_speed(s_d, ts)
        sm_dcr = ang_speed(s_r, ts)
        a, b = best_window(raw, args.window)
        x = np.arange(a, b)
        ax.plot(x, raw[a:b], lw=0.8, color="#999999",
                label=f"raw gyro (RMS {np.sqrt(np.mean(raw**2)):.1f}°/s)")
        ax.plot(x, sm_default[a:b], lw=1.1, color="#1f77b4",
                label=f"smoothed: default (RMS {np.sqrt(np.mean(sm_default**2)):.1f}°/s)")
        ax.plot(x, sm_dcr[a:b], lw=1.1, color="#2ca02c",
                label=f"smoothed: DCR (--enhanced) (RMS {np.sqrt(np.mean(sm_dcr**2)):.1f}°/s)")
        ax.set_title(f"{label} — camera angular velocity: raw vs smoothed "
                     f"(most-active {args.window}-frame window; RMS over full clip)")
        ax.set_ylabel("angular speed (deg/s)")
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8, loc="upper right")
    axes[-1].set_xlabel("frame")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
