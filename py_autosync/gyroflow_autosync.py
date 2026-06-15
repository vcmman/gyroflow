#!/usr/bin/env python3
"""gyroflow_autosync (Python) — UI-free autosync-time tool + timestamp-sync accuracy evaluator.

Python sibling of cpp_core's ``gyroflow_autosync``. Loads DJI fused-quaternion telemetry,
derives an angular-velocity signal from the quaternions, and runs Gyroflow's time-offset finder
to report the timestamp delay (ms) between two motion signals.

Modes:
  selftest  Inject known time offsets into a video-rate copy of the signal and measure recovery
            accuracy -> timestamp-sync precision (ground-truthed).
  compare   Find the offset between the full-rate signal and a video-rate resample (true offset 0)
            -> baseline bias floor.
  omega     Dump the quaternion-derived angular velocity as CSV.

Example:
  ./gyroflow_autosync.py selftest --quat ../data/dji_quaternions_full.csv --fps 30 --noise 1.5
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

import autosync_time as at


def _frame_timestamps(a: float, b: float, fps: float) -> np.ndarray:
    if fps <= 0.0 or b <= a:
        return np.array([])
    dt = 1000.0 / fps
    n = int(np.floor((b - a) / dt + 1e-6)) + 1
    return a + np.arange(n) * dt


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Python autosync-time / timestamp-sync evaluator")
    p.add_argument("mode", choices=["selftest", "compare", "omega"])
    p.add_argument("--quat", required=True, help="DJI quaternion CSV (full or camera_data format)")
    p.add_argument("--fps", type=float, default=30.0, help="video frame rate for the video-side signal")
    p.add_argument("--search", type=float, default=200.0, help="offset search size (ms)")
    p.add_argument("--initial", type=float, default=0.0, help="initial/rough offset (ms)")
    p.add_argument("--lpf", type=float, default=20.0, help="low-pass cutoff (Hz)")
    p.add_argument("--swap-xy", action="store_true", help="swap x/y when deriving angular velocity")
    p.add_argument("--inject", type=str, default="-30,-15,-5,0,5,15,30",
                   help="selftest: comma-separated injected offsets (ms)")
    p.add_argument("--noise", type=float, default=0.0, help="selftest: Gaussian noise stddev (deg/s)")
    p.add_argument("--range", type=str, default="", help="analyse only timestamps in A,B (ms)")
    p.add_argument("--seed", type=int, default=12345, help="selftest noise RNG seed")
    args = p.parse_args(argv)

    t_ms, quats = at.load_quaternions(args.quat)
    omega = at.quaternions_to_angular_velocity(t_ms, quats, swap_xy=args.swap_xy, degrees=True)

    first_ts, last_ts = float(omega.t[0]), float(omega.t[-1])
    imu_rate = (len(omega) - 1) / ((last_ts - first_ts) / 1000.0) if len(omega) > 1 else 0.0

    ra, rb = first_ts, last_ts
    if args.range:
        parts = [float(x) for x in args.range.split(",") if x]
        if len(parts) == 2 and parts[1] > parts[0]:
            ra, rb = parts

    print(f"Loaded {len(quats)} quaternions, span [{first_ts:.3f}, {last_ts:.1f}] ms, "
          f"IMU rate ~{imu_rate:.2f} Hz", file=sys.stderr)

    if args.mode == "omega":
        print("timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s")
        for i in range(len(omega)):
            t = omega.t[i]
            if t < ra or t > rb:
                continue
            w = omega.w[i]
            print(f"{t:.4f},{w[0]:.4f},{w[1]:.4f},{w[2]:.4f}")
        return 0

    if args.mode == "compare":
        fts = _frame_timestamps(ra, rb, args.fps)
        video = at.resample_angular_velocity(omega, fts)
        ma = at.max_angle(video)
        print(f"Video signal: {len(video)} frames @ {args.fps} fps, max |omega| = {ma:.4f} deg/s",
              file=sys.stderr)
        if ma < 3.0:
            print("warning: motion below 3 deg/s gate; result unreliable", file=sys.stderr)
        r = at.find_offset(video, omega, args.initial, args.search, args.fps, imu_rate, args.lpf)
        if r.found:
            print(f"recovered_offset_ms={r.offset_ms:.4f} cost={r.cost:.4f} "
                  f"matched={r.matched}/{len(video)}")
            print("(true offset is 0; recovered value is the algorithm's bias floor)")
            return 0
        print("no acceptable offset found")
        return 1

    # selftest
    fts = _frame_timestamps(ra, rb, args.fps)
    injects = [float(x) for x in args.inject.split(",") if x]
    rng = np.random.default_rng(args.seed)

    print("injected_ms,recovered_ms,error_ms,cost,matched,frames")
    errs = []
    ok = 0
    for inj in injects:
        video = at.resample_angular_velocity(omega, fts - inj)  # camera sees motion delayed by inj
        video.t = fts.copy()                                    # video timestamps on the video clock
        if args.noise > 0.0:
            video.w = video.w + rng.normal(0.0, args.noise, size=video.w.shape)
        r = at.find_offset(video, omega, args.initial, args.search, args.fps, imu_rate, args.lpf)
        if r.found:
            err = r.offset_ms - inj
            errs.append(err)
            ok += 1
            print(f"{inj:.4f},{r.offset_ms:.4f},{err:.4f},{r.cost:.4f},{r.matched},{len(video)}")
        else:
            print(f"{inj:.4f},NA,NA,NA,0,{len(video)}")

    if errs:
        errs = np.array(errs)
        print(f"\n=== Timestamp-sync precision ({ok}/{len(injects)} recovered) ===\n"
              f"mean error  = {errs.mean():.4f} ms\n"
              f"RMS error   = {np.sqrt((errs ** 2).mean()):.4f} ms\n"
              f"max |error| = {np.abs(errs).max():.4f} ms", file=sys.stderr)
        return 0
    print("no offsets recovered", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
