#!/usr/bin/env python3
"""Simplest image-domain vertical-motion metric.

For each video, take consecutive grayscale frames and estimate the global
sub-pixel translation with cv2.phaseCorrelate; keep the vertical component
(dy = per-frame up/down shift, in pixels of the analysis resolution).
Plot dy vs frame number, overlaying the modes for comparison.
"""
import os, sys, argparse
os.environ.setdefault("MPLBACKEND", "Agg")
import cv2
import numpy as np
import matplotlib.pyplot as plt


def vertical_flow(path, width=640, max_frames=None):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {path}")
    ok, prev = cap.read()
    if not ok:
        raise RuntimeError(f"empty video {path}")
    scale = width / prev.shape[1]
    size = (width, int(round(prev.shape[0] * scale)))
    win = cv2.createHanningWindow(size, cv2.CV_32F)

    def prep(f):
        g = cv2.cvtColor(cv2.resize(f, size), cv2.COLOR_BGR2GRAY)
        return np.float32(g)

    prev_g = prep(prev)
    dys = []
    i = 0
    while True:
        ok, cur = cap.read()
        if not ok:
            break
        cur_g = prep(cur)
        (dx, dy), _resp = cv2.phaseCorrelate(prev_g, cur_g, win)
        dys.append(dy)
        prev_g = cur_g
        i += 1
        if max_frames and i >= max_frames:
            break
        if i % 200 == 0:
            print(f"  {os.path.basename(path)}: {i} frames", flush=True)
    cap.release()
    return np.asarray(dys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="cpp_out directory")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("-o", "--out", default="vertical_flow_compare.png")
    args = ap.parse_args()

    clips = ["0001", "0002"]
    # (suffix, label, color) — one line per smoothing config
    configs = [
        ("",         "default (offline)",   "#7f7f7f"),
        ("_dcr",     "DCR (offline)",        "#2ca02c"),
        ("_l1",      "L1 jerk-limit",        "#9467bd"),
        ("_la1",     "DCR off + 1s LA",      "#d62728"),
        ("_dcr_la1", "DCR + 1s LA",          "#1f77b4"),
    ]

    # Per-clip extra reference clips (absolute paths), e.g. DJI in-camera stabilized.
    REF_DIR = "/media/yc/Seagate Backup Plus Drive/dji6/dji6_R/run"
    refs = {
        "0001": (os.path.join(REF_DIR, "DJI_20260625014752_0002_D.MP4"), "DJI in-camera", "#000000"),
        "0002": (os.path.join(REF_DIR, "DJI_20260625014927_0004_D.MP4"), "DJI in-camera", "#000000"),
    }

    series = {}
    for clip in clips:
        for suffix, label, _c in configs:
            fn = f"{clip}_D_cpp_stabilized{suffix}.mp4"
            p = os.path.join(args.dir, fn)
            if not os.path.exists(p):
                print(f"SKIP missing {p}")
                continue
            print(f"Analyzing {fn} ...", flush=True)
            dy = vertical_flow(p, args.width, args.max_frames)
            series[(clip, label)] = dy
            print(f"  -> {len(dy)} frames, RMS dy = {np.sqrt(np.mean(dy**2)):.3f} px", flush=True)
        if clip in refs:
            p, label, _c = refs[clip]
            if os.path.exists(p):
                print(f"Analyzing REF {os.path.basename(p)} ...", flush=True)
                dy = vertical_flow(p, args.width, args.max_frames)
                series[(clip, label)] = dy
                print(f"  -> {len(dy)} frames, RMS dy = {np.sqrt(np.mean(dy**2)):.3f} px", flush=True)
            else:
                print(f"SKIP missing REF {p}")

    fig, axes = plt.subplots(len(clips), 1, figsize=(15, 9), sharex=False)
    if len(clips) == 1:
        axes = [axes]
    for ax, clip in zip(axes, clips):
        plot_list = list(configs)
        if clip in refs:
            rp, rlabel, rcolor = refs[clip]
            plot_list = plot_list + [(None, rlabel, rcolor)]
        for suffix, label, color in plot_list:
            dy = series.get((clip, label))
            if dy is None:
                continue
            rms = np.sqrt(np.mean(dy**2))
            is_ref = suffix is None
            ax.plot(np.arange(len(dy)), dy, lw=0.9 if is_ref else 0.5, color=color,
                    alpha=0.9 if is_ref else 0.85, ls="--" if is_ref else "-",
                    label=f"{label}  (RMS {rms:.3f} px)")
        ax.axhline(0, color="k", lw=0.4, alpha=0.4)
        ax.set_title(f"Clip {clip} — per-frame vertical optical flow (phaseCorrelate dy)")
        ax.set_ylabel("vertical shift dy (px @ %dpx wide)" % args.width)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(alpha=0.2)
    axes[-1].set_xlabel("frame number")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
