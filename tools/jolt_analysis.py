#!/usr/bin/env python3
"""IMU-layer jolt analysis — does a severe jolt survive smoothing, and does the zoom pump?

Reads the per-frame CSV emitted by `gyroflow_cpp_validate`
(frame,ts_ms,ow,ox,oy,oz,sw,sx,sy,sz,fov):

  o* = raw (input) attitude quaternion        s* = smoothed (stabilized) attitude
  fov = adaptive-zoom FOV scale (crop = 1/fov; a *dip* in fov is a zoom-IN)

It quantifies the severe-jolt pathology described in cpp_core/TODO.md #6: Gyroflow's
velocity-adaptive low-pass *loosens* at high angular velocity, so it cannot tell an
intentional fast pan from an unintentional jolt and passes the jolt through; adaptive zoom
then pumps (or clamps at max_zoom => black borders).

Metrics:
  * angular velocity (deg/s): raw vs smoothed — mean / P95 / max.
  * angular jerk (deg/s^2): raw vs smoothed, RMS + P95 (transient-sensitive).
  * jolt pass-through: at the top-K raw-velocity frames, smoothed_vel / raw_vel. ~0 means the
    jolt was rejected; ~1 means it passed straight through to the stabilized path.
  * zoom pumping: fov std / range, and whether the deepest fov dips line up with jolt frames.

Usage:
  ./cpp_core/build/gyroflow_cpp_validate bridge.json --frames N --zoom-method envelope > v.csv
  python3 tools/jolt_analysis.py v.csv [--fps 29.97] [--top 20] [--plot out.png]
"""
import argparse
import sys

import numpy as np


def load(path):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    ts = rows["ts_ms"] / 1000.0
    raw = np.column_stack([rows["ow"], rows["ox"], rows["oy"], rows["oz"]])
    sm = np.column_stack([rows["sw"], rows["sx"], rows["sy"], rows["sz"]])
    fov = rows["fov"]
    return ts, raw, sm, fov


def _norm(q):
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def ang_vel(q, dt):
    """Angular speed (deg/s) between consecutive unit quaternions; len == len(q)-1."""
    q = _norm(q)
    q0, q1 = q[:-1], q[1:]
    # relative quat r = conj(q0) * q1  (conj of unit quat = inverse)
    w0, x0, y0, z0 = q0[:, 0], q0[:, 1], q0[:, 2], q0[:, 3]
    w1, x1, y1, z1 = q1[:, 0], q1[:, 1], q1[:, 2], q1[:, 3]
    # conj(q0) = (w0, -x0, -y0, -z0); Hamilton product with q1
    rw = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1
    rx = w0 * x1 - x0 * w1 - y0 * z1 + z0 * y1
    ry = w0 * y1 + x0 * z1 - y0 * w1 - z0 * x1
    rz = w0 * z1 - x0 * y1 + y0 * x1 - z0 * w1
    vnorm = np.sqrt(rx * rx + ry * ry + rz * rz)
    angle = 2.0 * np.arctan2(vnorm, np.abs(rw))    # [0, pi]
    return np.degrees(angle) / dt


def rms(a):
    return float(np.sqrt(np.mean(a ** 2))) if len(a) else 0.0


def stat_line(name, a, unit):
    return (f"  {name:24s} mean {np.mean(a):8.2f}  P95 {np.percentile(a,95):8.2f}  "
            f"max {np.max(a):8.2f}  {unit}")


def main():
    ap = argparse.ArgumentParser(description="IMU-layer jolt / zoom-pumping analysis.")
    ap.add_argument("csv", help="gyroflow_cpp_validate output CSV")
    ap.add_argument("--fps", type=float, default=0.0,
                    help="override fps; default derives dt from ts_ms")
    ap.add_argument("--top", type=int, default=20, help="# of worst jolt frames to summarise")
    ap.add_argument("--plot", help="write a velocity/jerk/fov plot to this PNG")
    a = ap.parse_args()

    ts, raw, sm, fov = load(a.csv)
    n = len(ts)
    if n < 3:
        raise SystemExit("need >=3 frames")
    if a.fps > 0:
        dt = np.full(n - 1, 1.0 / a.fps)
    else:
        dt = np.diff(ts)
        dt[dt <= 0] = np.median(dt[dt > 0])
    dt_mid = dt[1:]  # dt aligned with the jerk samples

    rv = ang_vel(raw, dt)          # len n-1
    sv = ang_vel(sm, dt)
    rj = np.abs(np.diff(rv)) / dt_mid   # raw jerk deg/s^2, len n-2
    sj = np.abs(np.diff(sv)) / dt_mid   # smoothed jerk

    print(f"== {a.csv} : {n} frames, {ts[-1]-ts[0]:.1f}s, "
          f"{'fps='+format(a.fps,'.2f') if a.fps>0 else 'dt from ts'} ==\n")

    print("Angular velocity (deg/s)")
    print(stat_line("raw", rv, "deg/s"))
    print(stat_line("smoothed", sv, "deg/s"))
    print("\nAngular jerk (deg/s^2)  [transient-sensitive]")
    print(f"  {'raw':24s} RMS {rms(rj):8.2f}  P95 {np.percentile(rj,95):8.2f}  max {np.max(rj):8.2f}")
    print(f"  {'smoothed':24s} RMS {rms(sj):8.2f}  P95 {np.percentile(sj,95):8.2f}  max {np.max(sj):8.2f}")

    # ---- jolt pass-through: how much raw velocity survives into the smoothed path ----
    k = min(a.top, len(rv))
    worst = np.argsort(rv)[-k:][::-1]
    ratio = sv[worst] / np.maximum(rv[worst], 1e-9)
    print(f"\nJolt pass-through (top {k} raw-velocity frames)")
    print(f"  raw vel mean {rv[worst].mean():.1f} deg/s -> smoothed mean {sv[worst].mean():.1f} deg/s")
    print(f"  pass-through ratio: mean {ratio.mean():.3f}  median {np.median(ratio):.3f}  "
          f"max {ratio.max():.3f}   (0=rejected, 1=passed through)")

    # ---- zoom pumping ----
    print("\nZoom (fov scale; crop = 1/fov, dip = zoom-in)")
    print(f"  fov mean {fov.mean():.4f}  std {fov.std():.4f}  min {fov.min():.4f}  max {fov.max():.4f}")
    print(f"  fov range {fov.max()-fov.min():.4f}  ({100*(fov.max()-fov.min())/fov.mean():.2f}% of mean)")
    # do the deepest zoom-ins coincide with jolts? fov index aligns with frame index.
    deep = np.argsort(fov)[:k]                       # frames with the smallest fov (most zoom)
    jolt_frames = set(worst.tolist())
    hits = sum(1 for d in deep if d in jolt_frames or (d - 1) in jolt_frames
               or (d + 1) in jolt_frames)
    print(f"  of the {k} deepest zoom-in frames, {hits} are at/adjacent to a top-{k} jolt "
          f"({100*hits/k:.0f}%)  <- high = zoom is pumping on jolts")

    if a.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n(matplotlib not available; skipping --plot)", file=sys.stderr)
            return
        t = ts[1:]
        fig, ax = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
        ax[0].plot(t, rv, lw=0.6, label="raw vel")
        ax[0].plot(t, sv, lw=0.8, label="smoothed vel")
        ax[0].set_ylabel("deg/s"); ax[0].legend(loc="upper right"); ax[0].set_title("Angular velocity")
        ax[1].plot(t[1:], rj, lw=0.6, label="raw jerk")
        ax[1].plot(t[1:], sj, lw=0.8, label="smoothed jerk")
        ax[1].set_ylabel("deg/s^2"); ax[1].legend(loc="upper right"); ax[1].set_title("Angular jerk")
        ax[2].plot(ts, fov, lw=0.8, color="C2")
        ax[2].set_ylabel("fov scale"); ax[2].set_xlabel("t (s)")
        ax[2].set_title("Adaptive zoom (dip = zoom-in)")
        fig.tight_layout(); fig.savefig(a.plot, dpi=110)
        print(f"\nwrote {a.plot}")


if __name__ == "__main__":
    main()
