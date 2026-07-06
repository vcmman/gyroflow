#!/usr/bin/env python3
"""Required zoom vs max_zoom clamp — the direct cause of black borders.

For each config: required zoom = 1/raw_fov (instantaneous zoom needed to cover the
frame with no black border), applied zoom = 1/fov (after temporal smoothing + clamp).
Black borders are forced wherever required zoom > max_zoom (1.30 = 130%).
Data from gyroflow_cpp_validate CSVs (columns ..., fov, raw_fov).
"""
import os, csv, argparse
os.environ.setdefault("MPLBACKEND", "Agg")
import numpy as np
import matplotlib.pyplot as plt


def load(path):
    frame, fov, raw = [], [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            frame.append(int(row["frame"]))
            fov.append(float(row["fov"]))
            raw.append(float(row["raw_fov"]))
    return np.array(frame), 1.0 / np.array(fov), 1.0 / np.array(raw)  # applied_zoom, required_zoom


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--max-zoom", type=float, default=1.30)
    ap.add_argument("-o", "--out", default="zoom_vs_maxzoom.png")
    args = ap.parse_args()

    clips = ["0001", "0002"]
    configs = [
        ("default", "default (offline)", "#7f7f7f"),
        ("dcr",     "DCR (offline)",      "#2ca02c"),
        ("la1",     "DCR off + 1s LA",    "#d62728"),
        ("dcr_la1", "DCR + 1s LA",        "#1f77b4"),
    ]
    mz = args.max_zoom

    data = {}
    print("===== ZOOM vs max_zoom (%.0f%%) =====" % (mz * 100))
    print(f"{'clip':5} {'config':20} {'max req zoom':>12} {'mean req':>9} "
          f"{'#req>clamp':>11} {'%frames':>8}")
    for clip in clips:
        for cfg, label, _c in configs:
            p = os.path.join(args.dir, f"zoom_{clip}_{cfg}.csv")
            if not os.path.exists(p):
                print(f"SKIP {p}"); continue
            fr, appz, reqz = load(p)
            data[(clip, cfg)] = (fr, appz, reqz)
            breach = reqz > mz + 1e-9
            print(f"{clip:5} {label:20} {reqz.max():12.3f} {reqz.mean():9.3f} "
                  f"{int(breach.sum()):11d} {100.0*breach.mean():7.1f}%")

    fig, axes = plt.subplots(2, len(clips), figsize=(16, 9), sharex="col")
    for col, clip in enumerate(clips):
        # Row 0: required zoom + clamp line
        axr = axes[0, col]
        for cfg, label, color in configs:
            if (clip, cfg) not in data:
                continue
            fr, appz, reqz = data[(clip, cfg)]
            axr.plot(fr, reqz, lw=0.5, color=color, alpha=0.85, label=label)
        axr.axhline(mz, color="k", lw=1.4, ls="--", label=f"max_zoom = {mz:.2f}")
        axr.set_title(f"Clip {clip} — REQUIRED zoom (1/raw_fov); above dashed = black border")
        axr.set_ylabel("required zoom  (1 / raw_fov)")
        axr.grid(alpha=0.2); axr.legend(fontsize=7, loc="upper right")

        # Row 1: applied zoom + clamp line
        axa = axes[1, col]
        for cfg, label, color in configs:
            if (clip, cfg) not in data:
                continue
            fr, appz, reqz = data[(clip, cfg)]
            axa.plot(fr, appz, lw=0.5, color=color, alpha=0.85, label=label)
        axa.axhline(mz, color="k", lw=1.4, ls="--", label=f"max_zoom = {mz:.2f}")
        axa.set_title(f"Clip {clip} — APPLIED zoom (1/fov, after smoothing + clamp)")
        axa.set_ylabel("applied zoom  (1 / fov)")
        axa.set_xlabel("frame number")
        axa.grid(alpha=0.2); axa.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
