#!/usr/bin/env python3
"""gyroflow_autosync (Python) — UI-free autosync-time tool + timestamp-sync accuracy evaluator.

Python sibling of cpp_core's ``gyroflow_autosync``. Loads DJI fused-quaternion telemetry,
derives an angular-velocity signal from the quaternions, and runs Gyroflow's time-offset finder
to report the timestamp delay (ms) between two motion signals.

Modes:
  sync      Find the timestamp offset between TWO real angular-velocity signals (e.g. an IMU
            GCSV log + a camera-motion CSV). This is the production path on real footage.
  selftest  Inject known time offsets into a video-rate copy of the signal and measure recovery
            accuracy -> timestamp-sync precision (ground-truthed).
  compare   Find the offset between the full-rate signal and a video-rate resample (true offset 0)
            -> baseline bias floor.
  omega     Dump the quaternion-derived angular velocity as CSV.

Examples:
  ./gyroflow_autosync.py sync --gyro imu.gcsv --video camera_motion.csv --interp-parabolic
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


def _run_sync(args, interp: bool, parabolic: bool) -> int:
    """Sync two real angular-velocity signals (production path on real footage)."""
    if not args.gyro or not args.video:
        print("sync mode needs both --gyro and --video", file=sys.stderr)
        return 2

    gyro = at.load_motion(args.gyro, units=args.units, orientation=args.gyro_orientation)
    video = at.load_motion(args.video, units=args.units, orientation=args.video_orientation)
    gyro_rate = at.estimate_sample_rate_hz(gyro)
    video_rate = at.estimate_sample_rate_hz(video)

    g_ma, v_ma = at.max_angle(gyro), at.max_angle(video)
    print(f"gyro : {len(gyro)} samples, ~{gyro_rate:.1f} Hz, span "
          f"[{gyro.t[0]:.1f},{gyro.t[-1]:.1f}] ms, max|omega| {g_ma:.2f} deg/s", file=sys.stderr)
    print(f"video: {len(video)} samples, ~{video_rate:.1f} Hz, span "
          f"[{video.t[0]:.1f},{video.t[-1]:.1f}] ms, max|omega| {v_ma:.2f} deg/s", file=sys.stderr)
    if min(g_ma, v_ma) < 3.0:
        print("warning: motion below the 3 deg/s gate; offset may be unreliable", file=sys.stderr)
    if gyro_rate <= 0.0 or video_rate <= 0.0:
        print("error: could not estimate a sample rate (need >=2 samples per signal)", file=sys.stderr)
        return 1

    r = at.find_offset(video, gyro, args.initial, args.search, video_rate, gyro_rate, args.lpf,
                       interp=interp, parabolic=parabolic)
    if not r.found:
        print("no acceptable offset found (try a larger --search, check motion overlap/units)",
              file=sys.stderr)
        return 1
    mode = "interp+parabolic" if parabolic else ("interp" if interp else "nearest")
    print(f"offset_ms={r.offset_ms:.4f} cost={r.cost:.4f} matched={r.matched}/{len(video)} mode={mode}")
    print("(gyro samples align to video timestamps shifted by -offset_ms)", file=sys.stderr)
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Python autosync-time / timestamp-sync evaluator")
    p.add_argument("mode", choices=["sync", "selftest", "compare", "omega"])
    p.add_argument("--quat", help="DJI quaternion CSV (selftest/compare/omega modes)")
    p.add_argument("--gyro", help="sync mode: IMU log (GCSV or angular-velocity CSV)")
    p.add_argument("--video", help="sync mode: camera-motion angular-velocity CSV")
    p.add_argument("--units", choices=["deg", "rad"], default="deg",
                   help="sync mode: angular-velocity units when not inferable from column names")
    p.add_argument("--gyro-orientation", default=None,
                   help="sync mode: 3-char axis remap for the IMU signal, e.g. xzY")
    p.add_argument("--video-orientation", default=None,
                   help="sync mode: 3-char axis remap for the camera signal")
    p.add_argument("--fps", type=float, default=30.0, help="video frame rate for the video-side signal")
    p.add_argument("--search", type=float, default=200.0, help="offset search size (ms)")
    p.add_argument("--initial", type=float, default=0.0, help="initial/rough offset (ms)")
    p.add_argument("--lpf", type=float, default=20.0, help="low-pass cutoff (Hz)")
    p.add_argument("--swap-xy", action="store_true", help="swap x/y when deriving angular velocity")
    p.add_argument("--interp", action="store_true",
                   help="interpolated IMU lookup (removes the nearest-sample quantization bias; "
                        "breaks bit-for-bit Rust/C++ parity)")
    p.add_argument("--interp-parabolic", action="store_true",
                   help="interp + parabolic sub-grid vertex of the cost curve (us-level; implies --interp)")
    p.add_argument("--inject", type=str, default="-30,-15,-5,0,5,15,30",
                   help="selftest: comma-separated injected offsets (ms)")
    p.add_argument("--noise", type=float, default=0.0, help="selftest: Gaussian noise stddev (deg/s)")
    p.add_argument("--range", type=str, default="", help="analyse only timestamps in A,B (ms)")
    p.add_argument("--seed", type=int, default=12345, help="selftest noise RNG seed")
    args = p.parse_args(argv)

    parabolic = args.interp_parabolic
    interp = args.interp or parabolic

    if args.mode == "sync":
        return _run_sync(args, interp, parabolic)

    if not args.quat:
        p.error(f"--quat is required for the {args.mode} mode")

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
        r = at.find_offset(video, omega, args.initial, args.search, args.fps, imu_rate, args.lpf,
                           interp=interp, parabolic=parabolic)
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
        r = at.find_offset(video, omega, args.initial, args.search, args.fps, imu_rate, args.lpf,
                           interp=interp, parabolic=parabolic)
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
