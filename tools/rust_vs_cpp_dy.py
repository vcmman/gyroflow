#!/usr/bin/env python3
"""Port-parity check: Rust vs C++ default renders via phaseCorrelate dy.

Compares two stabilized renders of the same clip produced under identical default
parameters — one by Rust Gyroflow (from the exported .gyroflow project), one by the
C++ port — on the per-frame global vertical shift `dy` (see
cpp_core/figures/README.md). Prints RMS dy for each, the per-frame RMS difference and
correlation, and writes an overlay plot. Backs cpp_core/COMPARISON.md §4.

Usage:
  python3 tools/rust_vs_cpp_dy.py \
    --pair 0001 RUST_0001.mp4 CPP_0001.mp4 \
    --pair 0002 RUST_0002.mp4 CPP_0002.mp4 \
    -o cpp_core/figures/rust_vs_cpp_default_dy.png
"""
import os, argparse
os.environ.setdefault("MPLBACKEND", "Agg")
import numpy as np
import matplotlib.pyplot as plt

from gyro_analysis.video_metrics import vertical_flow


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pair", nargs=3, action="append", metavar=("LABEL", "RUST", "CPP"),
                    required=True, help="clip label + Rust render + C++ render (repeatable)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("-o", "--out", default="rust_vs_cpp_default_dy.png")
    args = ap.parse_args()

    fig, axes = plt.subplots(len(args.pair), 1, figsize=(14, 4 * len(args.pair)))
    if len(args.pair) == 1:
        axes = [axes]
    print(f"{'clip':6} {'rust RMS':>9} {'cpp RMS':>9} {'ratio':>6} "
          f"{'RMS(rust-cpp)':>13} {'corr':>6}")
    for ax, (label, rust, cpp) in zip(axes, args.pair):
        print(f"Analyzing {label} ...", flush=True)
        r = vertical_flow(rust, args.width)
        c = vertical_flow(cpp, args.width)
        n = min(len(r), len(c))
        r, c = r[:n], c[:n]
        rmsR = np.sqrt(np.mean(r ** 2))
        rmsC = np.sqrt(np.mean(c ** 2))
        diff = np.sqrt(np.mean((r - c) ** 2))
        corr = np.corrcoef(r, c)[0, 1]
        print(f"{label:6} {rmsR:9.3f} {rmsC:9.3f} {rmsC / rmsR:6.3f} "
              f"{diff:13.3f} {corr:6.3f}")
        ax.plot(r, lw=0.5, color="#1f77b4", alpha=0.85, label=f"Rust default (RMS {rmsR:.3f})")
        ax.plot(c, lw=0.5, color="#d62728", alpha=0.85, label=f"C++ default (RMS {rmsC:.3f})")
        ax.set_title(f"Clip {label} — phaseCorrelate dy: Rust vs C++ default "
                     f"(corr {corr:.3f}, RMS diff {diff:.3f} px)")
        ax.set_ylabel(f"dy (px @{args.width})")
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8, loc="upper right")
    axes[-1].set_xlabel("frame number")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
