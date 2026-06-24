"""Segmented time-sync consistency study.

Inject ONE fixed time offset across the *whole* clip, then split the video side
into N contiguous segments and estimate the offset independently per segment.
A perfect estimator returns the same injected offset in every segment; the
spread between segments exposes the real per-segment precision (lookup
quantization + motion conditioning), and any monotonic trend vs segment time
would indicate genuine clock drift (offset+skew).

Produces a plot (recovered offset per segment, nearest vs interp, with the
injected truth line) and stores it plus a CSV of the numbers.

Run:  python3 segment_experiment.py [quat_csv] [fps] [n_segments] [inject_ms] [noise]
"""
from __future__ import annotations

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import autosync_time as at


def build_imu(quat_csv: str):
    t_ms, quats = at.load_quaternions(quat_csv)
    omega = at.quaternions_to_angular_velocity(t_ms, quats, degrees=True)
    span = omega.t[-1] - omega.t[0]
    imu_rate = (len(omega) - 1) / (span / 1000.0)
    return omega, imu_rate


def synth_video(omega, fps, inject_ms, sigma, rng):
    """Full-clip video-side omega: IMU sampled at uniform frame times, shifted by inject."""
    vts = np.arange(0.0, omega.t[-1] - omega.t[0], 1000.0 / fps)
    video = at.resample_angular_velocity(omega, vts - inject_ms + omega.t[0])
    video.t = vts.copy() + omega.t[0]
    if sigma > 0.0:
        video.w = video.w + rng.normal(0.0, sigma, size=video.w.shape)
    return video


def segment_offsets(video, omega, imu_rate, fps, n_seg, search, interp):
    """Split the video into n_seg contiguous chunks; estimate offset in each."""
    n = len(video)
    bounds = np.linspace(0, n, n_seg + 1, dtype=int)
    rows = []
    for s in range(n_seg):
        a, b = bounds[s], bounds[s + 1]
        if b - a < 8:
            continue
        seg = at.GyroSeries(video.t[a:b].copy(), video.w[a:b].copy())
        r = at.find_offset(seg, omega, 0.0, search, fps, imu_rate, 20.0, interp=interp)
        t_mid = 0.5 * (seg.t[0] + seg.t[-1])
        rows.append({
            "seg": s,
            "t_mid_s": (t_mid - video.t[0]) / 1000.0,
            "found": r.found,
            "offset": r.offset_ms if r.found else np.nan,
            "cost": r.cost if r.found else np.nan,
            "peak_omega": at.max_angle(seg),
        })
    return rows


def run(quat_csv, fps, n_seg, inject, noise, out_png, out_csv):
    omega, imu_rate = build_imu(quat_csv)
    print(f"IMU: {len(omega)} samples, rate ~{imu_rate:.1f} Hz, "
          f"span {(omega.t[-1]-omega.t[0])/1000:.1f}s, peak |omega| {at.max_angle(omega):.1f} deg/s")
    print(f"Injected fixed offset = {inject:.3f} ms | fps={fps} | segments={n_seg} | noise sigma={noise}\n")

    rng = np.random.default_rng(12345)
    video = synth_video(omega, fps, inject, noise, rng)
    search = 80.0

    res = {}
    for interp in (False, True):
        mode = "interp" if interp else "nearest"
        res[mode] = segment_offsets(video, omega, imu_rate, fps, n_seg, search, interp)

    # ---- report ----
    print(f"{'seg':>3} {'t_mid(s)':>9} {'peak|w|':>8} | "
          f"{'nearest off':>12} {'err':>8} | {'interp off':>11} {'err':>8}")
    print("-" * 70)
    nA = res["nearest"]
    iA = {r["seg"]: r for r in res["interp"]}
    for r in nA:
        ir = iA.get(r["seg"], {})
        n_off, n_err = r["offset"], r["offset"] - inject
        i_off = ir.get("offset", np.nan)
        i_err = i_off - inject
        print(f"{r['seg']:>3} {r['t_mid_s']:>9.2f} {r['peak_omega']:>8.1f} | "
              f"{n_off:>12.4f} {n_err:>+8.4f} | {i_off:>11.4f} {i_err:>+8.4f}")

    def stats(rows):
        offs = np.array([r["offset"] for r in rows if r["found"]])
        if not len(offs):
            return None
        return offs.mean(), offs.std(), offs.min(), offs.max(), offs.max() - offs.min()

    print()
    for mode in ("nearest", "interp"):
        st = stats(res[mode])
        if st:
            mean, std, lo, hi, rng_ = st
            print(f"{mode:>8}: mean={mean:+.4f}  std={std:.4f}  "
                  f"range=[{lo:+.4f},{hi:+.4f}]  spread={rng_:.4f} ms  (truth {inject:+.3f})")

    # linear trend (skew) on the interp estimates: offset = a*t + b
    iv = [r for r in res["interp"] if r["found"]]
    if len(iv) >= 3:
        t = np.array([r["t_mid_s"] for r in iv])
        o = np.array([r["offset"] for r in iv])
        slope, intercept = np.polyfit(t, o, 1)
        print(f"\ninterp linear fit: slope = {slope*1000:+.2f} us/s  "
              f"({slope/1000*1e6:+.2f} ppm clock skew), intercept = {intercept:+.4f} ms")
        print("  (a real fixed offset -> slope ~ 0; nonzero slope would indicate clock drift)")

    # ---- plot ----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1]})
    for mode, color, marker in (("nearest", "tab:orange", "o"), ("interp", "tab:blue", "s")):
        rows = [r for r in res[mode] if r["found"]]
        t = [r["t_mid_s"] for r in rows]
        o = [r["offset"] for r in rows]
        ax1.plot(t, o, marker=marker, color=color, label=f"{mode} lookup", alpha=0.85)
    ax1.axhline(inject, color="k", ls="--", lw=1, label=f"injected truth = {inject:g} ms")
    ax1.set_ylabel("recovered offset (ms)")
    ax1.set_title(f"Per-segment time-sync estimate ({n_seg} segments, fps={fps}, "
                  f"noise sigma={noise} deg/s)\nfixed injected offset across the whole clip")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.3)

    rows = [r for r in res["nearest"] if r["found"]]
    ax2.bar([r["t_mid_s"] for r in rows], [r["peak_omega"] for r in rows],
            width=(rows[1]["t_mid_s"] - rows[0]["t_mid_s"]) * 0.7 if len(rows) > 1 else 1.0,
            color="tab:gray", alpha=0.6)
    ax2.set_ylabel("peak |omega|\n(deg/s)")
    ax2.set_xlabel("segment mid-time (s)")
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print(f"\nsaved plot -> {out_png}")

    # ---- CSV ----
    with open(out_csv, "w") as f:
        f.write("mode,seg,t_mid_s,peak_omega,found,offset_ms,err_ms,cost\n")
        for mode in ("nearest", "interp"):
            for r in res[mode]:
                err = (r["offset"] - inject) if r["found"] else np.nan
                f.write(f"{mode},{r['seg']},{r['t_mid_s']:.4f},{r['peak_omega']:.2f},"
                        f"{int(r['found'])},{r['offset']:.5f},{err:.5f},{r['cost']:.4f}\n")
    print(f"saved data -> {out_csv}")


if __name__ == "__main__":
    quat = sys.argv[1] if len(sys.argv) > 1 else "../../../../data/dji_quaternions_full.csv"
    fps = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    n_seg = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    inject = float(sys.argv[4]) if len(sys.argv) > 4 else 7.3
    noise = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
    run(quat, fps, n_seg, inject, noise,
        out_png="segment_offsets.png", out_csv="segment_offsets.csv")
