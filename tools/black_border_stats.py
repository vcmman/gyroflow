#!/usr/bin/env python3
"""Black-border statistics across stabilization configs.

A "black border" is empty area revealed at the frame edge when the crop/zoom
cannot cover the warped image (e.g. adaptive zoom hits its max_zoom clamp).
Per frame we build a near-black mask and keep only the connected black regions
that TOUCH the frame edge (flood from the border) — this ignores genuinely dark
scene content that is not connected to an edge. Metric = that area / total px.
"""
import os, argparse
os.environ.setdefault("MPLBACKEND", "Agg")
import cv2
import numpy as np
import matplotlib.pyplot as plt


def border_black_series(path, width=480, thresh=8, stride=5, max_frames=None):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {path}")
    fracs, idxs = [], []
    i = -1
    got = 0
    size = None
    while True:
        ok = cap.grab()
        if not ok:
            break
        i += 1
        if i % stride != 0:
            continue
        ok, f = cap.retrieve()
        if not ok:
            break
        if size is None:
            scale = width / f.shape[1]
            size = (width, int(round(f.shape[0] * scale)))
        g = cv2.cvtColor(cv2.resize(f, size), cv2.COLOR_BGR2GRAY)
        mask = (g <= thresh).astype(np.uint8)
        if mask.any():
            num, labels = cv2.connectedComponents(mask, connectivity=8)
            border = np.concatenate([labels[0, :], labels[-1, :],
                                     labels[:, 0], labels[:, -1]])
            border_lbls = np.unique(border)
            border_lbls = border_lbls[border_lbls != 0]
            if border_lbls.size:
                area = np.isin(labels, border_lbls).sum()
            else:
                area = 0
        else:
            area = 0
        fracs.append(100.0 * area / (size[0] * size[1]))
        idxs.append(i)
        got += 1
        if got % 200 == 0:
            print(f"  {os.path.basename(path)}: {got} sampled (frame {i})", flush=True)
        if max_frames and i >= max_frames:
            break
    cap.release()
    return np.asarray(idxs), np.asarray(fracs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--thresh", type=int, default=8)
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("-o", "--out", default="black_border.png")
    args = ap.parse_args()

    clips = ["0001", "0002"]
    configs = [
        ("",         "default (offline)", "#7f7f7f"),
        ("_dcr",     "DCR (offline)",      "#2ca02c"),
        ("_l1",      "L1 jerk-limit",      "#9467bd"),
        ("_la1",     "DCR off + 1s LA",    "#d62728"),
        ("_dcr_la1", "DCR + 1s LA",        "#1f77b4"),
    ]

    stats = {}   # (clip,label) -> dict
    ts = {}      # (clip,label) -> (idx, frac)
    for clip in clips:
        for suffix, label, _c in configs:
            fn = f"{clip}_D_cpp_stabilized{suffix}.mp4"
            p = os.path.join(args.dir, fn)
            if not os.path.exists(p):
                print(f"SKIP missing {p}")
                continue
            print(f"Analyzing {fn} ...", flush=True)
            idx, frac = border_black_series(p, args.width, args.thresh, args.stride, args.max_frames)
            ts[(clip, label)] = (idx, frac)
            stats[(clip, label)] = dict(
                mean=float(np.mean(frac)), mx=float(np.max(frac)),
                p99=float(np.percentile(frac, 99)),
                pct_frames=float(100.0 * np.mean(frac > 0.01)),
                n=len(frac))
            s = stats[(clip, label)]
            print(f"  -> n={s['n']}  mean={s['mean']:.4f}%  p99={s['p99']:.4f}%  "
                  f"max={s['mx']:.4f}%  frames>0.01%={s['pct_frames']:.1f}%", flush=True)

    # ---- print table ----
    print("\n===== BLACK-BORDER SUMMARY (% of frame, edge-connected black) =====")
    hdr = f"{'clip':5} {'config':20} {'mean%':>8} {'p99%':>8} {'max%':>8} {'%frames>0.01':>13}"
    print(hdr); print("-" * len(hdr))
    for clip in clips:
        for _s, label, _c in configs:
            if (clip, label) in stats:
                s = stats[(clip, label)]
                print(f"{clip:5} {label:20} {s['mean']:8.4f} {s['p99']:8.4f} {s['mx']:8.4f} {s['pct_frames']:12.1f}%")

    # ---- figure: top = grouped bar (mean, with max marker); bottom = time series ----
    fig, axes = plt.subplots(2, len(clips), figsize=(15, 9))
    for col, clip in enumerate(clips):
        labels = [l for _s, l, _c in configs if (clip, l) in stats]
        colors = [c for _s, l, c in configs if (clip, l) in stats]
        means = [stats[(clip, l)]["mean"] for l in labels]
        maxes = [stats[(clip, l)]["mx"] for l in labels]
        x = np.arange(len(labels))
        axb = axes[0, col]
        axb.bar(x, means, color=colors, alpha=0.85)
        axb.plot(x, maxes, "kv", ms=7, label="max")
        for xi, mx in zip(x, maxes):
            axb.annotate(f"max {mx:.2f}", (xi, mx), textcoords="offset points",
                         xytext=(0, 4), ha="center", fontsize=7)
        axb.set_xticks(x); axb.set_xticklabels(labels, rotation=20, ha="right", fontsize=8)
        axb.set_ylabel("black border (% of frame)")
        axb.set_title(f"Clip {clip} — mean black border (bar) + max (▼)")
        axb.grid(alpha=0.2, axis="y")
        axb.legend(fontsize=8)

        axt = axes[1, col]
        for suffix, label, color in configs:
            if (clip, label) not in ts:
                continue
            idx, frac = ts[(clip, label)]
            axt.plot(idx, frac, lw=0.6, color=color, alpha=0.85, label=label)
        axt.set_xlabel("frame number")
        axt.set_ylabel("black border (% of frame)")
        axt.set_title(f"Clip {clip} — per-frame black border")
        axt.grid(alpha=0.2)
        axt.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
