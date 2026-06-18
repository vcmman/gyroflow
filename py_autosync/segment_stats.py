"""Statistical study of per-segment time-sync estimates.

Question: the per-segment estimates scatter by ~1 ms, but their *mean* looks
close to the truth. How does that hold up statistically?

We inject ONE fixed offset, split the clip into N segments, and average the
per-segment estimates into a single "clip estimate". Repeating over many noise
realizations we measure, for each segment count N:

  * bias      = E[mean_seg_offset] - truth          (does averaging remove error?)
  * std_mean  = std of the clip estimate over trials (repeatability of the mean)
  * std_seg   = typical std *within* one trial's N segments (raw per-segment scatter)

Two predictions to check:
  1. If per-segment error were pure independent noise, std_mean ~ std_seg / sqrt(N).
  2. nearest lookup carries a deterministic +bias that averaging CANNOT remove;
     interp removes it, so its mean converges to the truth.

Outputs a 4-panel plot + CSV.

Run:  python3 segment_stats.py [quat_csv] [fps] [inject_ms] [noise] [trials]
"""
from __future__ import annotations

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import autosync_time as at
from segment_experiment import build_imu, synth_video, segment_offsets


def run(quat_csv, fps, inject, noise, trials, out_png, out_csv):
    omega, imu_rate = build_imu(quat_csv)
    span_s = (omega.t[-1] - omega.t[0]) / 1000.0
    print(f"IMU: {len(omega)} samples ~{imu_rate:.1f} Hz, span {span_s:.1f}s, "
          f"peak |omega| {at.max_angle(omega):.1f} deg/s")
    print(f"fixed inject={inject:g} ms | fps={fps} | noise sigma={noise} deg/s | trials={trials}\n")

    seg_counts = [1, 2, 4, 8, 16, 32]
    search = 80.0
    modes = ("nearest", "interp")

    # results[mode][n] = dict of arrays over trials
    agg = {m: {} for m in modes}
    print(f"{'N':>3} {'seg_len(s)':>10} | "
          + " | ".join(f"{m+' bias':>11} {m+' sdMean':>10} {m+' sdSeg':>9}" for m in modes))
    print("-" * 92)

    for n_seg in seg_counts:
        seg_len = span_s / n_seg
        row_means = {m: [] for m in modes}      # clip estimate (mean of segs) per trial
        row_segstd = {m: [] for m in modes}     # within-trial per-seg std
        for k in range(trials):
            # one noise realization shared across both modes (fair comparison)
            rng = np.random.default_rng(abs(hash((round(inject, 1), round(noise, 1), n_seg, k))) % (2**32))
            video = synth_video(omega, fps, inject, noise, rng)
            for m in modes:
                rows = segment_offsets(video, omega, imu_rate, fps, n_seg, search, interp=(m == "interp"))
                offs = np.array([r["offset"] for r in rows if r["found"]])
                if len(offs):
                    row_means[m].append(offs.mean())
                    row_segstd[m].append(offs.std() if len(offs) > 1 else 0.0)
        for m in modes:
            cm = np.array(row_means[m])
            agg[m][n_seg] = {
                "bias": cm.mean() - inject,
                "std_mean": cm.std(),
                "std_seg": float(np.mean(row_segstd[m])),
            }
        cells = []
        for m in modes:
            a = agg[m][n_seg]
            cells.append(f"{a['bias']:>+11.4f} {a['std_mean']:>10.4f} {a['std_seg']:>9.4f}")
        print(f"{n_seg:>3} {seg_len:>10.2f} | " + " | ".join(cells))

    # ---- plot ----
    Ns = np.array(seg_counts, dtype=float)
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    colors = {"nearest": "tab:orange", "interp": "tab:blue"}

    # (1) bias of the clip mean vs N
    ax = axes[0, 0]
    for m in modes:
        b = [agg[m][n]["bias"] for n in seg_counts]
        ax.plot(Ns, b, "o-", color=colors[m], label=m)
    ax.axhline(0, color="k", ls="--", lw=1)
    ax.set_xscale("log", base=2)
    ax.set_xlabel("number of segments N")
    ax.set_ylabel("bias of clip mean (ms)")
    ax.set_title("(1) Does averaging remove the error?\nbias = E[mean of segments] - truth")
    ax.legend(); ax.grid(True, alpha=0.3)

    # (2) repeatability of the clip mean vs N
    ax = axes[0, 1]
    for m in modes:
        s = [agg[m][n]["std_mean"] for n in seg_counts]
        ax.plot(Ns, s, "o-", color=colors[m], label=m)
    ax.set_xscale("log", base=2); ax.set_yscale("log")
    ax.set_xlabel("number of segments N")
    ax.set_ylabel("std of clip mean (ms)")
    ax.set_title("(2) Repeatability of the averaged estimate")
    ax.legend(); ax.grid(True, alpha=0.3, which="both")

    # (3) 1/sqrt(N) check: per-seg scatter vs std of the mean
    ax = axes[1, 0]
    for m in modes:
        sseg = np.array([agg[m][n]["std_seg"] for n in seg_counts])
        smean = np.array([agg[m][n]["std_mean"] for n in seg_counts])
        ax.plot(Ns, sseg, "s--", color=colors[m], alpha=0.6, label=f"{m}: per-seg std")
        ax.plot(Ns, smean, "o-", color=colors[m], label=f"{m}: std of mean")
    # ideal 1/sqrt(N) reference anchored at N=1 interp per-seg std
    base = agg["interp"][1]["std_seg"] or agg["interp"][2]["std_seg"]
    ax.plot(Ns, base / np.sqrt(Ns), "k:", lw=1.5, label="1/sqrt(N) reference")
    ax.set_xscale("log", base=2); ax.set_yscale("log")
    ax.set_xlabel("number of segments N")
    ax.set_ylabel("std (ms)")
    ax.set_title("(3) Variance averaging: std of mean vs 1/sqrt(N)")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3, which="both")

    # (4) raw per-segment scatter (shorter segments = noisier)
    ax = axes[1, 1]
    seg_lens = span_s / Ns
    for m in modes:
        sseg = [agg[m][n]["std_seg"] for n in seg_counts]
        ax.plot(seg_lens, sseg, "o-", color=colors[m], label=m)
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("segment length (s)")
    ax.set_ylabel("per-segment std (ms)")
    ax.set_title("(4) Shorter segments -> noisier single estimate")
    ax.legend(); ax.grid(True, alpha=0.3, which="both")

    fig.suptitle(f"Statistics of per-segment time-sync estimates "
                 f"(fixed inject={inject:g} ms, fps={fps}, noise sigma={noise} deg/s, {trials} trials)",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_png, dpi=130)
    print(f"\nsaved plot -> {out_png}")

    with open(out_csv, "w") as f:
        f.write("mode,n_seg,seg_len_s,bias_ms,std_mean_ms,std_seg_ms\n")
        for m in modes:
            for n in seg_counts:
                a = agg[m][n]
                f.write(f"{m},{n},{span_s/n:.4f},{a['bias']:.5f},{a['std_mean']:.5f},{a['std_seg']:.5f}\n")
    print(f"saved data -> {out_csv}")
    return agg


if __name__ == "__main__":
    quat = sys.argv[1] if len(sys.argv) > 1 else "../../../../data/dji_quaternions_full.csv"
    fps = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    inject = float(sys.argv[3]) if len(sys.argv) > 3 else 7.3
    noise = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    trials = int(sys.argv[5]) if len(sys.argv) > 5 else 30
    run(quat, fps, inject, noise, trials,
        out_png="segment_stats.png", out_csv="segment_stats.csv")
