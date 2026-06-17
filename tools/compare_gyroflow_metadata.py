#!/usr/bin/env python3
"""Validate the C++ core against Rust Gyroflow's golden per-frame metadata.

This cross-checks the *math* of the C++ port (smoothing + adaptive zoom + quaternion
sampling) independently of any video encoder or 10-bit decode, so it isolates the
stabilization algorithms from rendering noise.

Pipeline:
  1. Export Gyroflow's per-frame metadata (org/stab quaternions + fov_scale):
       gyroflow PROJECT.gyroflow --export-metadata "3:/tmp/gf_meta.json"
  2. Dump the same quantities from the C++ core:
       gyroflow_cpp_validate bridge.json --frames N > /tmp/cpp_validate.csv
  3. Diff them:
       python3 tools/compare_gyroflow_metadata.py /tmp/gf_meta.json /tmp/cpp_validate.csv

Conventions (matched in gyroflow_cpp_validate.cpp):
  * Gyroflow's exported stab_quat = org * smoothed^-1, so the C++ `smoothed` orientation
    series is compared directly against stab_quat (both are the "stabilized orientation").
  * fov_scale == params.fovs[frame] (the smoothed adaptive-zoom value actually applied).
  * Quaternions are sampled at ts = frame*1000/fps + readout/2 (gyro_export middle_ts).
"""
import csv
import json
import math
import sys


def qang_deg(a, b):
    """Angular distance (degrees) between two unit quaternions, sign-agnostic."""
    d = min(1.0, abs(sum(x * y for x, y in zip(a, b))))
    return 2.0 * math.acos(d) * 180.0 / math.pi


def main(argv):
    if len(argv) != 3:
        print(__doc__)
        print(f"usage: {argv[0]} <gf_meta.json> <cpp_validate.csv>", file=sys.stderr)
        return 2
    gf = json.load(open(argv[1]))
    cpp = {}
    with open(argv[2]) as f:
        for r in csv.DictReader(f):
            cpp[int(r["frame"])] = r

    ts_err, org_err, stab_err, fov_err, fov_rel = [], [], [], [], []
    for g in gf:
        fr = g["frame"]
        c = cpp.get(fr)
        if c is None:
            continue
        ts_err.append(abs(g["timestamp_ms"] - float(c["ts_ms"])))
        org_err.append(qang_deg(g["org_quat"],
                                [float(c["ow"]), float(c["ox"]), float(c["oy"]), float(c["oz"])]))
        stab_err.append(qang_deg(g["stab_quat"],
                                 [float(c["sw"]), float(c["sx"]), float(c["sy"]), float(c["sz"])]))
        e = abs(g["fov_scale"] - float(c["fov"]))
        fov_err.append(e)
        if g["fov_scale"] > 0:
            fov_rel.append(e / g["fov_scale"])

    if not org_err:
        print("No overlapping frames between the two files.", file=sys.stderr)
        return 1

    def line(name, v, unit, fmt="{:.6g}"):
        mx, mn = max(v), sum(v) / len(v)
        print(f"  {name:30s} max={fmt.format(mx)}{unit}  mean={fmt.format(mn)}{unit}")

    print(f"=== C++ core vs Rust Gyroflow ({len(org_err)} frames) ===")
    line("timestamp_ms", ts_err, " ms")
    line("org_quat (sampling sanity)", org_err, " deg")
    line("smoothed orient vs stab_quat", stab_err, " deg")
    line("adaptive fov vs fov_scale", fov_err, "")
    line("adaptive fov relative", [r * 100 for r in fov_rel], " %", "{:.4f}")
    # A loose pass/fail gate for CI-style use.
    ok = max(stab_err) < 0.05 and max(fov_rel) < 0.001
    print(f"RESULT: {'PASS' if ok else 'REVIEW'} "
          f"(smoothing < 0.05 deg and fov < 0.1% rel)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
