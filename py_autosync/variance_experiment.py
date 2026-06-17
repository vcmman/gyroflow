"""Monte-Carlo precision/variance study for the autosync-time estimator.

Loads the real DJI quaternion stream as the IMU side, synthesises a video-side
angular-velocity signal at a chosen fps with a *known* injected offset plus
Gaussian noise, then runs find_offset() many times to measure:

  * bias   = mean(recovered - injected)        (systematic error)
  * std    = std (recovered - injected)         (repeatability / variance)
  * range  = [min, max] of the error            (worst-case spread)

Run:  python3 variance_experiment.py [quat_csv] [fps] [trials]
"""
from __future__ import annotations

import sys
import numpy as np

import autosync_time as at


def build_imu(quat_csv: str):
    t_ms, quats = at.load_quaternions(quat_csv)
    omega = at.quaternions_to_angular_velocity(t_ms, quats, degrees=True)
    span = omega.t[-1] - omega.t[0]
    imu_rate = (len(omega) - 1) / (span / 1000.0)
    return omega, imu_rate


def synth_video(omega, fps, inject_ms, sigma, rng):
    """Video-side ω: sample the IMU signal at uniform frame times, shifted by inject."""
    vts = np.arange(0.0, omega.t[-1] - omega.t[0], 1000.0 / fps)
    video = at.resample_angular_velocity(omega, vts - inject_ms + omega.t[0])
    video.t = vts.copy() + omega.t[0]
    if sigma > 0.0:
        video.w = video.w + rng.normal(0.0, sigma, size=video.w.shape)
    return video


def run(quat_csv, fps, trials):
    omega, imu_rate = build_imu(quat_csv)
    print(f"IMU: {len(omega)} samples, rate ~{imu_rate:.1f} Hz, "
          f"span {(omega.t[-1]-omega.t[0])/1000:.1f}s, peak |ω| {at.max_angle(omega):.1f} deg/s")
    print(f"Video synth: fps={fps}, trials per cell={trials}\n")

    # ω is in deg/s; typical handheld peaks ~ tens of deg/s. Noise sigma in deg/s.
    sigmas = [0.0, 0.5, 1.0, 2.0, 5.0, 10.0]
    injects = [-30.0, -10.0, 0.0, 7.3, 25.0]
    search = 80.0

    print(f"{'noise σ':>8} | {'bias(ms)':>9} | {'std(ms)':>8} | {'min..max err(ms)':>18} | {'n_fail':>6}")
    print("-" * 64)
    overall = []
    for sigma in sigmas:
        errs = []
        fails = 0
        for inj in injects:
            for k in range(trials):
                rng = np.random.default_rng(abs(hash((round(sigma, 1), round(inj, 1), k))) % (2**32))
                video = synth_video(omega, fps, inj, sigma, rng)
                r = at.find_offset(video, omega, 0.0, search, fps, imu_rate, 20.0)
                if not r.found:
                    fails += 1
                    continue
                errs.append(r.offset_ms - inj)
        errs = np.array(errs)
        overall.append((sigma, errs))
        if len(errs):
            print(f"{sigma:>8.1f} | {errs.mean():>+9.4f} | {errs.std():>8.4f} | "
                  f"{errs.min():>+8.3f}..{errs.max():>+7.3f} | {fails:>6d}")
        else:
            print(f"{sigma:>8.1f} | {'--':>9} | {'--':>8} | {'all failed':>18} | {fails:>6d}")

    # Pure repeatability at one fixed true offset (isolates variance from offset-dependence).
    print("\nRepeatability at fixed true offset = 7.3 ms (noise σ=2.0), 40 trials:")
    rep = []
    for k in range(40):
        rng = np.random.default_rng(50000 + k)
        video = synth_video(omega, fps, 7.3, 2.0, rng)
        r = at.find_offset(video, omega, 0.0, search, fps, imu_rate, 20.0)
        if r.found:
            rep.append(r.offset_ms)
    rep = np.array(rep)
    print(f"  recovered mean={rep.mean():.4f} ms  std={rep.std():.4f} ms  "
          f"range=[{rep.min():.3f}, {rep.max():.3f}]  (true 7.3)")
    return overall


if __name__ == "__main__":
    quat = sys.argv[1] if len(sys.argv) > 1 else "../../../../data/dji_quaternions_full.csv"
    fps = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    trials = int(sys.argv[3]) if len(sys.argv) > 3 else 12
    run(quat, fps, trials)
