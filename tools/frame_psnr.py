#!/usr/bin/env python3
"""Frame-by-frame PSNR / pixel-error between two equally-sized, frame-aligned videos.

Use it to compare the C++ stabilizer's output against Rust Gyroflow's (render both from the
SAME source so the diff reflects the stabilization math, not the decoder). For a like-for-
like number, render Gyroflow with the same interpolation as the C++ kernel (Bilinear).

Usage:
    python3 tools/frame_psnr.py A.mp4 B.mp4 [--step 20] [--max-frames 0]
"""
import argparse
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("needs OpenCV (pip install opencv-python) and numpy")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--step", type=int, default=20, help="compare every Nth frame (default 20)")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0=all)")
    args = ap.parse_args()

    A, B = cv2.VideoCapture(args.a), cv2.VideoCapture(args.b)
    ps, mae, mx = [], [], []
    i = 0
    while True:
        if args.max_frames and i >= args.max_frames:
            break
        okA, fa = A.read()
        okB, fb = B.read()
        if not okA or not okB:
            break
        if i % args.step == 0:
            if fa.shape != fb.shape:
                sys.exit(f"shape mismatch at frame {i}: {fa.shape} vs {fb.shape}")
            d = fa.astype(np.float64) - fb.astype(np.float64)
            mse = float((d * d).mean())
            ps.append(10 * np.log10(255 ** 2 / mse) if mse > 0 else 99.0)
            mae.append(float(np.abs(d).mean()))
            mx.append(int(np.abs(d).max()))
        i += 1
    A.release()
    B.release()
    if not ps:
        sys.exit("no frames compared")
    print(f"compared {len(ps)} frames (every {args.step} of {i})")
    print(f"PSNR          min={min(ps):.2f}  mean={sum(ps)/len(ps):.2f}  max={max(ps):.2f} dB")
    print(f"mean-abs-err  mean={sum(mae)/len(mae):.2f}  worst-frame={max(mae):.2f}  (of 255)")
    print(f"max-pixel-err worst={max(mx)}")


if __name__ == "__main__":
    main()
