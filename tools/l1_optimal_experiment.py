#!/usr/bin/env python3
"""Prototype L1-optimal camera path (Grundmann 2011) vs Gyroflow default_algo, same crop budget.

Experiment for cpp_core/JOLT_RND.md: is a crop-aware L1-optimal path better than the
velocity-adaptive low-pass on a speed-bump jolt? L1-optimal finds the path whose 1st/2nd/3rd
derivatives are L1-sparse (=> static / linear / parabolic segments) subject to staying within a
crop box around the original path. The crop box is the "crop-aware" part the smoothing-only gate
lacked.

We solve it per euler channel (1D LP via scipy HiGHS), constrained to the SAME per-axis angular
deviation that default_algo actually used (read from the validate CSV's o*/s* columns). That is
the fair comparison: same crop, who produces the steadier (lower-jerk) path?

Usage:
  ./cpp_core/build/gyroflow_cpp_validate BRIDGE.json --frames N > v.csv
  python3 tools/l1_optimal_experiment.py v.csv [--w1 10 --w2 1 --w3 100] [--budget 1.0]
         [--lo F --hi F]   # restrict to a frame window (e.g. around a jolt)
"""
import argparse
import numpy as np
from scipy import sparse
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation

FPS = 29.97


def deriv(n, k):
    """k-th difference operator (n-k, n) by composing first differences."""
    D = sparse.eye(n, format="csc")
    rows = n
    for _ in range(k):
        Dk = (sparse.eye(rows - 1, rows, k=1) - sparse.eye(rows - 1, rows)).tocsc()
        D = Dk @ D
        rows -= 1
    return D.tocsc()


def l1_path(c, B, w1, w2, w3):
    """Minimize w1|D1 p|+w2|D2 p|+w3|D3 p| s.t. |p-c|<=B.  Returns p."""
    n = len(c)
    D1, D2, D3 = deriv(n, 1), deriv(n, 2), deriv(n, 3)
    n1, n2, n3 = D1.shape[0], D2.shape[0], D3.shape[0]
    # variables: p (n), e1 (n1), e2 (n2), e3 (n3)
    nv = n + n1 + n2 + n3
    cost = np.concatenate([np.zeros(n), w1 * np.ones(n1), w2 * np.ones(n2), w3 * np.ones(n3)])

    def block(Dk, nk, off):
        # Dk p - ek <= 0  and  -Dk p - ek <= 0
        Z = sparse.csc_matrix((nk, nv))
        Ipos = sparse.lil_matrix((nk, nv))
        Ineg = sparse.lil_matrix((nk, nv))
        Ipos[:, :n] = Dk
        Ineg[:, :n] = -Dk
        ek = sparse.eye(nk, format="lil")
        Ipos[:, off:off + nk] = -ek
        Ineg[:, off:off + nk] = -ek
        return sparse.vstack([Ipos, Ineg]).tocsc()

    A = sparse.vstack([
        block(D1, n1, n),
        block(D2, n2, n + n1),
        block(D3, n3, n + n1 + n2),
    ]).tocsc()
    b = np.zeros(A.shape[0])
    # bounds: p in [c-B, c+B]; e >= 0
    bounds = [(c[i] - B, c[i] + B) for i in range(n)] + [(0, None)] * (n1 + n2 + n3)
    res = linprog(cost, A_ub=A, b_ub=b, bounds=bounds, method="highs")
    if not res.success:
        raise RuntimeError("LP failed: " + res.message)
    return res.x[:n]


def quat_xyzw(d, pre):
    return np.column_stack([d[pre + "x"], d[pre + "y"], d[pre + "z"], d[pre + "w"]])


def path_metrics(quat_xyzw_arr, lo, hi):
    """frame-to-frame angular velocity (deg/s) and jerk RMS over [lo,hi)."""
    q = quat_xyzw_arr / np.linalg.norm(quat_xyzw_arr, axis=1, keepdims=True)
    dot = np.abs((q[:-1] * q[1:]).sum(1))
    vel = np.degrees(2 * np.arccos(np.clip(dot, 0, 1))) * FPS
    seg = vel[lo:hi] if hi > lo else vel
    jerk = np.diff(seg) * FPS
    return vel, np.sqrt((jerk ** 2).mean())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv")
    ap.add_argument("--w1", type=float, default=10.0)
    ap.add_argument("--w2", type=float, default=1.0)
    ap.add_argument("--w3", type=float, default=100.0)
    ap.add_argument("--budget", type=float, default=1.0,
                    help="crop budget as a multiple of default_algo's per-axis max deviation")
    ap.add_argument("--lo", type=int, default=0, help="metric window start frame")
    ap.add_argument("--hi", type=int, default=0, help="metric window end frame (0=end)")
    a = ap.parse_args()

    d = np.genfromtxt(a.csv, delimiter=",", names=True)
    n = len(d["ts_ms"])
    lo = a.lo
    hi = a.hi if a.hi > 0 else n - 1

    raw = quat_xyzw(d, "o")
    def_ = quat_xyzw(d, "s")
    e_raw = Rotation.from_quat(raw).as_euler("xyz")     # radians, (n,3)
    e_def = Rotation.from_quat(def_).as_euler("xyz")
    # unwrap each channel so a ±pi wrap doesn't blow up the deviation/crop budget
    e_raw = np.unwrap(e_raw, axis=0)
    e_def = np.unwrap(e_def, axis=0)

    # per-axis crop budget = what default_algo actually used (times --budget)
    Bs = np.abs(e_def - e_raw).max(axis=0) * a.budget
    Bs = np.maximum(Bs, 1e-6)

    e_l1 = np.zeros_like(e_raw)
    for ch in range(3):
        e_l1[:, ch] = l1_path(e_raw[:, ch], Bs[ch], a.w1, a.w2, a.w3)
    q_l1 = Rotation.from_euler("xyz", e_l1).as_quat()

    vraw, jraw = path_metrics(raw, lo, hi)
    vdef, jdef = path_metrics(def_, lo, hi)
    vl1, jl1 = path_metrics(q_l1, lo, hi)

    dev_def = np.degrees(np.abs(e_def - e_raw))
    dev_l1 = np.degrees(np.abs(e_l1 - e_raw))

    print(f"== {a.csv}  frames {n}, metric window [{lo},{hi}), budget x{a.budget}, "
          f"weights w1/w2/w3={a.w1}/{a.w2}/{a.w3} ==")
    print(f"crop budget per axis (deg, = default's max deviation x{a.budget}): "
          f"{np.degrees(Bs).round(2)}")
    print()
    print(f"{'path':<14}{'jerk RMS (deg/s^2)':>20}{'max vel (deg/s)':>18}{'max dev (deg)':>16}")
    print(f"{'raw':<14}{jraw:>20.1f}{vraw[lo:hi].max():>18.1f}{'-':>16}")
    print(f"{'default_algo':<14}{jdef:>20.1f}{vdef[lo:hi].max():>18.1f}"
          f"{dev_def[lo:hi].max():>16.2f}")
    print(f"{'L1-optimal':<14}{jl1:>20.1f}{vl1[lo:hi].max():>18.1f}"
          f"{dev_l1[lo:hi].max():>16.2f}")
    print()
    impr = 100 * (1 - jl1 / jdef) if jdef else 0
    print(f"L1 vs default: output-path jerk {jdef:.1f} -> {jl1:.1f}  ({impr:+.1f}%)   "
          f"(L1 max-dev {dev_l1[lo:hi].max():.2f} <= budget {np.degrees(Bs).max():.2f} deg)")


if __name__ == "__main__":
    main()
