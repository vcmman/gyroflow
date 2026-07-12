#!/usr/bin/env python3
"""Generate a synthetic bridge.json: a smooth pan + an injected speed-bump jolt.

A controlled testbed for the severe / speed-bump jolt case (cpp_core/JOLT_RND.md). Real footage
mixes the jolt with continuous shake, so it is hard to tell how the stabilizer treats the jolt
itself. Here the ground truth is known: a constant-rate pan (which a good stabilizer should
follow) plus a sharp injected impact at a known time (which it should reject). Feed the output
to gyroflow_cpp_validate and score with tools/jolt_analysis.py.

Two impact profiles (--profile):
  * bump (default) — a *damped oscillation* A·exp(-(t-t0)/tau)·sin(2*pi*f*(t-t0)), the realistic
    model of a bicycle/handheld hitting a speed bump: a sharp deflection that overshoots and
    rings down. --jolt-sigma is the decay time tau (s), --bump-freq the ring frequency (Hz).
  * gaussian — a simple deflect-and-recover A·exp(-((t-t0)/sigma)^2) (no overshoot); --jolt-sigma
    is the Gaussian width sigma (s).
Pass several --jolt-time values (comma list) to model e.g. the front then rear wheel.

The lens profile / dimensions / readout are copied from a real bridge so undistort + adaptive
zoom run unchanged; only the quaternion stream is synthetic. IMU samples are emitted at ~1 kHz
(matching DJI); timestamps are in microseconds (telemetry_io divides by 1000 -> ms).

Usage:
  python3 tools/make_synthetic_jolt.py --template data/dji_bridge.json -o /tmp/bump.json \
      --duration 10 --pan-rate 10 --profile bump --jolt-amp 8 --jolt-sigma 0.08 --bump-freq 6 \
      [--jolt-time 5] [--imu-hz 1000]
  # two-wheel speed bump (front + rear, ~0.2 s apart, rear smaller):
  #   --jolt-time 5,5.2 --jolt-amp 8,5
"""
import argparse
import json

import numpy as np


def axis_angle_quat(axis, angle_rad):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    h = angle_rad / 2.0
    s = np.sin(h)
    return np.array([np.cos(h), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def main():
    ap = argparse.ArgumentParser(description="Synthetic pan+jolt bridge.json generator.")
    ap.add_argument("--template", default="data/dji_bridge.json",
                    help="real bridge to copy lens/dims/readout from")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--duration", type=float, default=10.0, help="seconds")
    ap.add_argument("--imu-hz", type=float, default=1000.0)
    ap.add_argument("--pan-rate", type=float, default=25.0, help="yaw pan rate, deg/s")
    ap.add_argument("--pan-axis", default="0,1,0", help="pan rotation axis x,y,z")
    ap.add_argument("--jolt-axis", default="1,0,0", help="jolt rotation axis x,y,z")
    ap.add_argument("--profile", choices=["bump", "gaussian"], default="bump",
                    help="bump = damped oscillation (speed bump); gaussian = deflect-and-recover")
    ap.add_argument("--jolt-amp", default="8", help="deflection amplitude(s) deg, comma list")
    ap.add_argument("--jolt-sigma", default="0.08", help="bump: decay tau (s); gaussian: width "
                    "sigma (s). comma list")
    ap.add_argument("--bump-freq", type=float, default=6.0, help="ring frequency for --profile "
                    "bump, Hz")
    ap.add_argument("--jolt-time", default="", help="jolt onset time(s) s, comma list; "
                    "default single jolt at mid-clip")
    args = ap.parse_args()

    tmpl = json.load(open(args.template))
    pan_axis = [float(x) for x in args.pan_axis.split(",")]
    jolt_axis = [float(x) for x in args.jolt_axis.split(",")]
    amps = [float(x) for x in str(args.jolt_amp).split(",") if x != ""]
    sigs = [float(x) for x in str(args.jolt_sigma).split(",") if x != ""]
    if args.jolt_time:
        times = [float(x) for x in args.jolt_time.split(",")]
    else:
        times = [args.duration / 2.0]
    # broadcast amp/sigma to the number of jolt times
    if len(amps) == 1:
        amps *= len(times)
    if len(sigs) == 1:
        sigs *= len(times)
    if not (len(amps) == len(sigs) == len(times)):
        raise SystemExit("jolt-amp / jolt-sigma / jolt-time counts must match (or be scalar)")

    n = int(round(args.duration * args.imu_hz))
    t = np.arange(n) / args.imu_hz                  # seconds, starting at 0

    pan_deg = args.pan_rate * t                      # smooth, constant-velocity pan
    jolt_deg = np.zeros(n)
    for amp, sig, t0 in zip(amps, sigs, times):
        if args.profile == "bump":
            # damped oscillation: sharp kick at t0 that overshoots and rings down (speed bump).
            dt = t - t0
            ring = amp * np.exp(-np.maximum(dt, 0.0) / sig) * np.sin(2 * np.pi * args.bump_freq * dt)
            jolt_deg += np.where(dt >= 0.0, ring, 0.0)
        else:
            jolt_deg += amp * np.exp(-0.5 * ((t - t0) / sig) ** 2)   # kick out and recover

    quats = []
    for i in range(n):
        q = quat_mul(axis_angle_quat(pan_axis, np.radians(pan_deg[i])),
                     axis_angle_quat(jolt_axis, np.radians(jolt_deg[i])))
        ts_us = t[i] * 1e6                            # microseconds (telemetry_io /1000 -> ms)
        quats.append([ts_us, float(q[0]), float(q[1]), float(q[2]), float(q[3])])

    out = dict(tmpl)
    out["quaternions"] = quats
    out["detected_source"] = "synthetic-jolt"
    json.dump(out, open(args.out, "w"))
    extra = f" freq={args.bump_freq}Hz" if args.profile == "bump" else ""
    print(f"wrote {args.out}: {n} IMU samples, {args.duration}s @ {args.imu_hz} Hz, "
          f"pan {args.pan_rate} deg/s, {args.profile} jolts amp={amps} "
          f"{'tau' if args.profile=='bump' else 'sigma'}={sigs}{extra} t={times}")


if __name__ == "__main__":
    main()
