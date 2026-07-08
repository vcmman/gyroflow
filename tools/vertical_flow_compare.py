#!/usr/bin/env python3
"""Simplest image-domain vertical-motion metric.

For each video, take consecutive grayscale frames and estimate the global
sub-pixel translation with cv2.phaseCorrelate; keep the vertical component
(dy = per-frame up/down shift, in pixels of the analysis resolution).
Plot dy vs frame number, overlaying the modes for comparison.
"""
import os, argparse
os.environ.setdefault("MPLBACKEND", "Agg")
import numpy as np
import matplotlib.pyplot as plt

from gyro_analysis.video_metrics import vertical_flow


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="cpp_out directory")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--ref", nargs=2, action="append", metavar=("CLIP", "VIDEO"), default=None,
                    help="optional per-clip reference video (e.g. DJI in-camera), repeatable: "
                         "--ref 0001 /path/DJI_ref.MP4. Omit to plot configs only.")
    ap.add_argument("--series", nargs=3, action="append", metavar=("CLIP", "LABEL", "VIDEO"),
                    default=None,
                    help="explicit extra series, repeatable: --series bike0005 DCR /path/x.mp4. "
                         "Adds subplots for clips outside the --dir naming pattern.")
    ap.add_argument("--cache", default=None, metavar="DIR",
                    help="cache dir for per-video dy arrays (skip re-decoding on plot iterations)")
    ap.add_argument("-o", "--out", default="vertical_flow_compare.png")
    args = ap.parse_args()

    clips = ["0001", "0002"]
    # (suffix, label, color) — one line per smoothing config.
    # default is orange (NOT gray/black) so it stays clearly distinct from the black DJI ref.
    configs = [
        ("",         "default (offline)",   "#ff7f0e"),
        ("_dcr",     "DCR (offline)",        "#2ca02c"),
        ("_l1",      "L1 jerk-limit",        "#9467bd"),
        ("_la1",     "DCR off + 1s LA",      "#d62728"),
        ("_dcr_la1", "DCR + 1s LA",          "#1f77b4"),
    ]

    # Per-clip extra reference clips from --ref (e.g. DJI in-camera stabilized takes).
    refs = {}
    for clip, path in (args.ref or []):
        refs[clip] = (path, "DJI in-camera", "#000000")

    # Explicit extra series (--series) for clips whose files don't follow the --dir pattern.
    # New clips are appended as additional subplots in first-appearance order.
    extra = {}   # clip -> [(label, path), ...]
    for clip, label, path in (args.series or []):
        extra.setdefault(clip, []).append((label, path))
        if clip not in clips:
            clips.append(clip)
    # color per extra label: reuse the config palette by label match, else cycle spares
    label_color = {label: c for _s, label, c in configs}
    spare = ["#17becf", "#bcbd22", "#8c564b", "#e377c2"]

    # Optional dy cache (npy per video, keyed by basename+width) so plot iterations don't
    # re-decode ~10-minute 4K clips.
    def measure(p):
        if args.cache:
            os.makedirs(args.cache, exist_ok=True)
            key = os.path.join(args.cache,
                               f"{os.path.basename(p)}.w{args.width}.dy.npy")
            if os.path.exists(key):
                print(f"  (cached) {os.path.basename(p)}", flush=True)
                return np.load(key)
            dy = vertical_flow(p, args.width, args.max_frames)
            np.save(key, dy)
            return dy
        return vertical_flow(p, args.width, args.max_frames)

    series = {}
    for clip in clips:
        if clip not in extra:   # pattern-based configs only for --dir-style clips
            for suffix, label, _c in configs:
                fn = f"{clip}_D_cpp_stabilized{suffix}.mp4"
                p = os.path.join(args.dir, fn)
                if not os.path.exists(p):
                    print(f"SKIP missing {p}")
                    continue
                print(f"Analyzing {fn} ...", flush=True)
                dy = measure(p)
                series[(clip, label)] = dy
                print(f"  -> {len(dy)} frames, RMS dy = {np.sqrt(np.mean(dy**2)):.3f} px", flush=True)
        for label, p in extra.get(clip, []):
            if not os.path.exists(p):
                print(f"SKIP missing {p}")
                continue
            print(f"Analyzing {os.path.basename(p)} ...", flush=True)
            dy = measure(p)
            series[(clip, label)] = dy
            print(f"  -> {len(dy)} frames, RMS dy = {np.sqrt(np.mean(dy**2)):.3f} px", flush=True)
        if clip in refs:
            p, label, _c = refs[clip]
            if os.path.exists(p):
                print(f"Analyzing REF {os.path.basename(p)} ...", flush=True)
                dy = measure(p)
                series[(clip, label)] = dy
                print(f"  -> {len(dy)} frames, RMS dy = {np.sqrt(np.mean(dy**2)):.3f} px", flush=True)
            else:
                print(f"SKIP missing REF {p}")

    fig, axes = plt.subplots(len(clips), 1, figsize=(15, 9), sharex=False)
    if len(clips) == 1:
        axes = [axes]
    for ax, clip in zip(axes, clips):
        if clip in extra:
            plot_list = []
            for label, _p in extra[clip]:
                c = label_color.get(label) or (spare.pop(0) if spare else "#444444")
                label_color[label] = c
                plot_list.append(("x", label, c))
        else:
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
