#!/usr/bin/env python3
"""Basic visualization for the autosync-time tool.

Produces a 3-panel diagnostic PNG for a gyro<->video time-offset estimate:

  A  cost vs offset over the whole search window (global minimum marked)
  B  cost zoomed to +/-3 ms around the minimum (shows the sharp, sub-ms valley)
  C  angular-velocity overlay in a high-motion window, gyro aligned by the recovered offset
     (faint grey = gyro at offset 0, i.e. before alignment)

Two input modes (mirroring gyroflow_autosync.py):

  real       --gyro IMU.gcsv --video CAMERA.csv          (two measured signals)
  synthetic  --quat DJI.csv --fps 30 [--inject 8.5]      (camera synthesized from quaternions;
                                                          the injected offset is drawn as truth)

Examples:
  ./visualize.py --gyro imu.gcsv --video cam.csv --interp-parabolic --out sync.png
  ./visualize.py --quat ../data/dji_quaternions_full.csv --fps 30 --inject 8.5 --out demo.png
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")  # headless: render straight to a file
import matplotlib.pyplot as plt

import autosync_time as at


def _magnitude(w: np.ndarray) -> np.ndarray:
    return np.linalg.norm(w, axis=1)


def _high_motion_window(series: at.GyroSeries, half_ms: float = 1000.0):
    """[lo, hi] ms window centred on the peak |omega| sample (for a legible overlay)."""
    if len(series) == 0:
        return series.t[0] if len(series) else 0.0, 0.0
    c = float(series.t[int(np.argmax(_magnitude(series.w)))])
    return c - half_ms, c + half_ms


def _frame_timestamps(a: float, b: float, fps: float) -> np.ndarray:
    dt = 1000.0 / fps
    n = int(np.floor((b - a) / dt + 1e-6)) + 1
    return a + np.arange(n) * dt


def _load_inputs(args):
    """Return (video, gyro, video_rate, gyro_rate, truth_ms or None, title)."""
    if args.quat:  # synthetic: camera is a delayed video-rate resample of the IMU motion
        t_ms, quats = at.load_quaternions(args.quat)
        gyro = at.quaternions_to_angular_velocity(t_ms, quats, swap_xy=args.swap_xy, degrees=True)
        gyro_rate = at.estimate_sample_rate_hz(gyro)
        fts = _frame_timestamps(float(gyro.t[0]), float(gyro.t[-1]), args.fps)
        video = at.resample_angular_velocity(gyro, fts - args.inject)
        video.t = fts.copy()
        return video, gyro, args.fps, gyro_rate, args.inject, f"synthetic (quat, {args.fps:g} fps)"
    gyro = at.load_motion(args.gyro, units=args.units, orientation=args.gyro_orientation)
    video = at.load_motion(args.video, units=args.units, orientation=args.video_orientation)
    return (video, gyro, at.estimate_sample_rate_hz(video), at.estimate_sample_rate_hz(gyro),
            None, "real signals")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Visualize an autosync-time offset estimate")
    p.add_argument("--gyro", help="real mode: IMU log (GCSV or angular-velocity CSV)")
    p.add_argument("--video", help="real mode: camera-motion angular-velocity CSV")
    p.add_argument("--quat", help="synthetic mode: DJI quaternion CSV")
    p.add_argument("--fps", type=float, default=30.0, help="synthetic mode: video frame rate")
    p.add_argument("--inject", type=float, default=8.5, help="synthetic mode: injected offset (ms)")
    p.add_argument("--swap-xy", action="store_true", help="synthetic mode: swap x/y deriving omega")
    p.add_argument("--units", choices=["deg", "rad"], default="deg", help="real mode: input units")
    p.add_argument("--gyro-orientation", default=None, help="real mode: 3-char axis remap (gyro)")
    p.add_argument("--video-orientation", default=None, help="real mode: 3-char axis remap (video)")
    p.add_argument("--search", type=float, default=200.0, help="offset search half-width (ms)")
    p.add_argument("--initial", type=float, default=0.0, help="rough starting offset (ms)")
    p.add_argument("--lpf", type=float, default=20.0, help="low-pass cutoff (Hz)")
    p.add_argument("--interp", action="store_true", help="interpolated IMU lookup")
    p.add_argument("--interp-parabolic", action="store_true", help="interp + parabolic sub-grid")
    p.add_argument("--out", default="autosync_plot.png", help="output PNG path")
    args = p.parse_args(argv)

    if not args.quat and not (args.gyro and args.video):
        p.error("provide either --quat (synthetic) or both --gyro and --video (real)")
    parabolic = args.interp_parabolic
    interp = args.interp or parabolic

    video, gyro, video_rate, gyro_rate, truth, title = _load_inputs(args)
    if video_rate <= 0.0 or gyro_rate <= 0.0:
        print("error: could not estimate sample rates (need >=2 samples per signal)", file=sys.stderr)
        return 1

    r = at.find_offset(video, gyro, args.initial, args.search, video_rate, gyro_rate, args.lpf,
                       interp=interp, parabolic=parabolic)
    if not r.found:
        print("no acceptable offset found; nothing to plot (try a larger --search)", file=sys.stderr)
        return 1
    mode = "interp+parabolic" if parabolic else ("interp" if interp else "nearest")
    print(f"offset_ms={r.offset_ms:.4f} cost={r.cost:.4f} matched={r.matched}/{len(video)} mode={mode}",
          file=sys.stderr)

    # --- cost curves (reuse the exact search cost) ---
    coarse_x = np.arange(args.initial - args.search, args.initial + args.search + 1e-9, 0.5)
    coarse_y = at.cost_curve(video, gyro, coarse_x, video_rate, gyro_rate, args.lpf, interp,
                             args.initial, args.search)
    zoom_x = np.arange(r.offset_ms - 3.0, r.offset_ms + 3.0 + 1e-9, 0.02)
    zoom_y = at.cost_curve(video, gyro, zoom_x, video_rate, gyro_rate, args.lpf, interp,
                           args.initial, args.search)

    # --- alignment overlay in a high-motion window ---
    lo, hi = _high_motion_window(video)
    vmask = (video.t >= lo) & (video.t <= hi)
    vt, vmag = video.t[vmask], _magnitude(video.w[vmask])
    g_aligned = at.resample_angular_velocity(gyro, vt - r.offset_ms)  # gyro at video_t - offset
    g_unaligned = at.resample_angular_velocity(gyro, vt)              # gyro at offset 0

    fig, ax = plt.subplot_mosaic("AA\nBC", figsize=(12, 7), constrained_layout=True)
    fig.suptitle(f"autosync-time — {title} — offset = {r.offset_ms:.3f} ms ({mode})", fontsize=13)

    finite = np.isfinite(coarse_y)
    ax["A"].plot(coarse_x[finite], coarse_y[finite], lw=1.0, color="#1f77b4")
    ax["A"].axvline(r.offset_ms, color="#d62728", lw=1.2, label=f"found {r.offset_ms:.3f} ms")
    if truth is not None:
        ax["A"].axvline(truth, color="#2ca02c", ls="--", lw=1.2, label=f"truth {truth:.3f} ms")
    ax["A"].set(xlabel="candidate offset (ms)", ylabel="cost", title="A · cost vs offset (full search)")
    ax["A"].legend(loc="upper right", fontsize=9)
    ax["A"].grid(alpha=0.3)

    fz = np.isfinite(zoom_y)
    ax["B"].plot(zoom_x[fz], zoom_y[fz], lw=1.0, color="#1f77b4")
    ax["B"].axvline(r.offset_ms, color="#d62728", lw=1.2)
    if truth is not None:
        ax["B"].axvline(truth, color="#2ca02c", ls="--", lw=1.2)
    ax["B"].set(xlabel="offset (ms)", ylabel="cost", title="B · cost zoom (±3 ms)")
    ax["B"].grid(alpha=0.3)

    ax["C"].plot(vt, vmag, color="#1f77b4", lw=1.2, label="video |ω|")
    ax["C"].plot(vt, _magnitude(g_unaligned.w), color="#bbbbbb", lw=1.0, label="gyro |ω| (offset 0)")
    ax["C"].plot(vt, _magnitude(g_aligned.w), color="#d62728", lw=1.0, ls="--",
                 label="gyro |ω| (aligned)")
    ax["C"].set(xlabel="video time (ms)", ylabel="|ω| (deg/s)",
                title="C · alignment in peak-motion window")
    ax["C"].legend(loc="upper right", fontsize=8)
    ax["C"].grid(alpha=0.3)

    fig.savefig(args.out, dpi=120)
    print(f"wrote {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
