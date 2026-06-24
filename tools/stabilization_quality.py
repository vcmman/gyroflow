#!/usr/bin/env python3
"""Measure how *stable* a video is — and compare before/after stabilization.

Unlike PSNR-vs-a-reference (which measures whether two renders match), these are
intra-video metrics: they quantify the residual shake left in a single clip. Run on the
original and the stabilized output to see how much motion the stabilizer removed.

Metrics (all computed on a centre-cropped, downscaled, grayscale frame sequence so the
original 4:3 sensor and the 16:9 stabilized crop are comparable):

  * ITF (Interframe Transformation Fidelity) — mean PSNR between *consecutive* frames.
    Steadier video => consecutive frames look more alike => higher ITF (dB). Higher = better.
  * Phase-correlation shift — mean magnitude of the global translation between consecutive
    frames (the dominant pan/tilt shake), in px and as % of frame width. Lower = steadier.
  * Optical-flow magnitude — mean Farneback flow magnitude between consecutive frames
    (captures rotation / complex residual motion, not just translation). Lower = steadier.

Transient-sensitive metrics (the mean metrics above *average jolts away* — a single severe
jolt barely moves the mean, but it is exactly what a viewer notices). These target the worst
moments instead of the average:

  * shift P95 — 95th-percentile inter-frame shift. The worst pans/jolts, not the average.
  * shift jerk (RMS) — RMS of the frame-to-frame *change* in the shift vector. A smooth pan
    has near-zero jerk; a jolt is a spike in jerk. This is the metric that actually moves when
    a jolt is removed, and the primary score for severe-jolt work. Lower = steadier.
  * ITF P05 — 5th-percentile (worst) consecutive-frame PSNR: the single ugliest transition.

Usage:
  # single clip
  python3 tools/stabilization_quality.py VIDEO.mp4 [--max-frames 300] [--width 640]
  # before/after (prints the improvement)
  python3 tools/stabilization_quality.py --compare ORIGINAL.mp4 STABILIZED.mp4 [--max-frames 300]

Notes: the two clips are aligned frame-0-to-frame-0; pass matching --max-frames. Center-crop
to 16:9 + downscale normalises FOV/scale differences and speeds up the 4K math.
"""
import argparse
import sys

import numpy as np

try:
    import cv2
except ImportError:
    print("This script needs OpenCV (pip install opencv-python) and numpy.", file=sys.stderr)
    raise


def prep(frame, width):
    """Center-crop to 16:9, downscale to `width`, return float32 grayscale."""
    h, w = frame.shape[:2]
    # crop the central 16:9 region
    target_h = int(round(w * 9 / 16))
    if target_h <= h:
        y0 = (h - target_h) // 2
        frame = frame[y0:y0 + target_h, :]
    else:  # already wider than 16:9 — crop width instead
        target_w = int(round(h * 16 / 9))
        x0 = (w - target_w) // 2
        frame = frame[:, x0:x0 + target_w]
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    g = cv2.resize(g, (width, int(round(width * 9 / 16))), interpolation=cv2.INTER_AREA)
    return g.astype(np.float32)


def analyze(path, max_frames, width):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {path}")
    psnrs, flows = [], []
    shift_vecs = []  # (dx, dy) per consecutive pair — kept as vectors for the jerk metric
    prev = None
    n = 0
    while max_frames <= 0 or n < max_frames:
        ok, f = cap.read()
        if not ok:
            break
        g = prep(f, width)
        if prev is not None:
            mse = np.mean((g - prev) ** 2)
            psnrs.append(10 * np.log10(255.0 ** 2 / mse) if mse > 0 else 99.0)
            (dx, dy), _ = cv2.phaseCorrelate(prev, g)
            shift_vecs.append((dx, dy))
            flow = cv2.calcOpticalFlowFarneback(
                prev, g, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            flows.append(float(np.mean(np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2))))
        prev = g
        n += 1
    cap.release()
    if not psnrs:
        raise SystemExit(f"not enough frames in {path}")
    psnrs = np.asarray(psnrs)
    flows = np.asarray(flows)
    sv = np.asarray(shift_vecs)                       # (m, 2)
    shifts = np.sqrt((sv ** 2).sum(axis=1))           # per-pair shift magnitude
    # jerk = magnitude of the change in the shift vector between consecutive pairs (= the
    # second difference of position). A steady pan is ~constant velocity => low jerk; a jolt
    # is a velocity spike => high jerk. RMS so a few big spikes dominate (unlike the mean).
    djerk = np.diff(sv, axis=0)
    jerk = np.sqrt((djerk ** 2).sum(axis=1)) if len(sv) > 1 else np.zeros(0)
    # flow jerk = RMS of the frame-to-frame change in mean optical-flow magnitude. Mean flow
    # barely drops with stabilization (pan + scene motion + zoom magnification dominate); the
    # transient *change* in flow is what a jolt produces, so this is the optical-flow analogue
    # of shift jerk. Lower = steadier.
    flow_jerk = np.diff(flows) if len(flows) > 1 else np.zeros(0)
    return {
        "frames": n,
        "itf_db": float(psnrs.mean()),
        "itf_p05_db": float(np.percentile(psnrs, 5)),
        "shift_px": float(shifts.mean()),
        "shift_pctw": 100.0 * float(shifts.mean()) / width,
        "shift_p95_px": float(np.percentile(shifts, 95)),
        "shift_jerk_px": float(np.sqrt((jerk ** 2).mean())) if len(jerk) else 0.0,
        "flow_px": float(flows.mean()),
        "flow_p95_px": float(np.percentile(flows, 95)),
        "flow_jerk_px": float(np.sqrt((flow_jerk ** 2).mean())) if len(flow_jerk) else 0.0,
    }


def show(label, r):
    print(f"{label} ({r['frames']} frames):")
    print(f"  ITF (consecutive-frame PSNR) : {r['itf_db']:.2f} dB  "
          f"(P05 {r['itf_p05_db']:.2f})    (higher = steadier)")
    print(f"  phase-corr shift             : {r['shift_px']:.3f} px  "
          f"({r['shift_pctw']:.3f}% width)  (lower = steadier)")
    print(f"  shift P95 (worst jolts)      : {r['shift_p95_px']:.3f} px   (lower = steadier)")
    print(f"  shift jerk (RMS, transients) : {r['shift_jerk_px']:.3f} px   (lower = steadier) *")
    print(f"  optical-flow magnitude       : {r['flow_px']:.3f} px  "
          f"(P95 {r['flow_p95_px']:.3f})  (lower = steadier)")
    print(f"  optical-flow jerk (RMS)      : {r['flow_jerk_px']:.3f} px   (lower = steadier) *")


def main():
    ap = argparse.ArgumentParser(description="Stabilization quality (ITF + residual motion).")
    ap.add_argument("videos", nargs="+", help="VIDEO  |  --compare ORIGINAL STABILIZED")
    ap.add_argument("--compare", action="store_true",
                    help="treat the two videos as original vs stabilized and print improvement")
    ap.add_argument("--max-frames", type=int, default=300, help="frames to analyze (0=all)")
    ap.add_argument("--width", type=int, default=640, help="downscaled analysis width")
    a = ap.parse_args()

    results = [(v, analyze(v, a.max_frames, a.width)) for v in a.videos]
    for v, r in results:
        show(v, r)
        print()

    if a.compare and len(results) >= 2:
        (_, o), (_, s) = results[0], results[1]
        print("=== improvement (original -> stabilized) ===")
        print(f"  ITF                : {o['itf_db']:.2f} -> {s['itf_db']:.2f} dB "
              f"(+{s['itf_db'] - o['itf_db']:.2f} dB)")
        print(f"  ITF P05 (worst)    : {o['itf_p05_db']:.2f} -> {s['itf_p05_db']:.2f} dB "
              f"(+{s['itf_p05_db'] - o['itf_p05_db']:.2f} dB)")
        for key, name in (("shift_px", "phase-corr shift"), ("shift_p95_px", "shift P95"),
                          ("shift_jerk_px", "shift jerk (RMS)"), ("flow_px", "optical-flow"),
                          ("flow_p95_px", "optical-flow P95"), ("flow_jerk_px", "optical-flow jerk")):
            red = 100.0 * (1 - s[key] / o[key]) if o[key] else 0.0
            tag = "  <- jolt metric" if key in ("shift_jerk_px", "flow_jerk_px") else ""
            print(f"  {name:18s} : {o[key]:.3f} -> {s[key]:.3f} px  "
                  f"({red:.1f}% less residual motion){tag}")


if __name__ == "__main__":
    main()
