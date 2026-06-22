"""Unit tests for the Python autosync-time port.

Run standalone:  python3 test_autosync_time.py
Or with pytest:   pytest test_autosync_time.py
Mirrors cpp_core/tests/test_timesync.cpp (lowpass, omega, recovery, noise robustness).
"""

from __future__ import annotations

import numpy as np

import autosync_time as at


def _random_walk_attitude(n: int, fs: float, seed: int):
    """Band-limited random-walk attitude (AR(1) body rates), like real handheld motion."""
    rng = np.random.default_rng(seed)
    a = np.zeros(3)
    quats = np.zeros((n, 4))
    cur = np.array([1.0, 0.0, 0.0, 0.0])
    for i in range(n):
        a = 0.99 * a + 0.06 * rng.standard_normal(3)
        mag = np.linalg.norm(a)
        cur = at.quat_normalize(at.quat_mul(cur, at.quat_from_axis_angle(a, mag * (1.0 / fs))))
        quats[i] = cur
    return np.arange(n) * 1.0, quats


def _video_signal(omega, span_ms, fps, inject, noise_sigma, rng):
    vts = np.arange(0.0, span_ms + 1e-6, 1000.0 / fps)
    video = at.resample_angular_velocity(omega, vts - inject)
    video.t = vts.copy()
    if noise_sigma > 0.0:
        video.w = video.w + rng.normal(0.0, noise_sigma, size=video.w.shape)
    return video


def test_lowpass():
    fs = 1000.0
    # DC passes through with unit gain.
    x = np.full(500, 1.0)
    y = at.Lowpass.filter_forward_backward(20.0, fs, x)
    assert y is not None and abs(y[250] - 1.0) < 1e-6

    # 150 Hz tone attenuated > 95% at 20 Hz cutoff.
    t = np.arange(1000) / fs
    sig = np.sin(2 * np.pi * 150.0 * t)
    f = at.Lowpass.filter_forward_backward(20.0, fs, sig)
    amp_in = np.max(np.abs(sig[400:600]))
    amp_out = np.max(np.abs(f[400:600]))
    assert amp_out < amp_in * 0.05

    # Nyquist guard: 2*f0 > fs leaves data untouched (returns None).
    assert at.Lowpass.filter_forward_backward(20.0, 30.0, np.array([7.0, 7.0])) is None


def test_omega():
    fs = 1000.0
    rate = 45.0  # deg/s about z
    rate_rad = np.deg2rad(rate)
    t = np.arange(1001) * 1.0
    quats = np.array([at.quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), rate_rad * (ti / 1000.0)) for ti in t])
    omega = at.quaternions_to_angular_velocity(t, quats, swap_xy=False, degrees=True)
    assert len(omega) == len(t)
    assert abs(omega.w[500, 0]) < 1e-6
    assert abs(omega.w[500, 1]) < 1e-6
    assert abs(omega.w[500, 2] - rate) < 1e-3

    # swap_xy maps a pure pitch (y) rate onto x.
    pq = np.array([at.quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), rate_rad * (ti / 1000.0)) for ti in t])
    po = at.quaternions_to_angular_velocity(t, pq, swap_xy=True, degrees=True)
    assert abs(po.w[500, 0] - rate) < 1e-3


def test_recovery():
    fs, n = 1000.0, 6000
    t, quats = _random_walk_attitude(n, fs, 777)
    omega = at.quaternions_to_angular_velocity(t, quats, degrees=True)
    imu_rate = (len(omega) - 1) / ((omega.t[-1] - omega.t[0]) / 1000.0)
    fps, span = 30.0, (n - 1) * 1.0
    rng = np.random.default_rng(0)
    max_err = 0.0
    for inj in (-20.0, -7.3, 0.0, 8.5, 18.0):
        video = _video_signal(omega, span, fps, inj, 0.0, rng)
        r = at.find_offset(video, omega, 0.0, 80.0, fps, imu_rate, 20.0)
        assert r.found
        max_err = max(max_err, abs(r.offset_ms - inj))
    assert max_err < 0.6, f"max recovery error {max_err:.3f} ms"


def test_noise_robustness():
    fs, n = 1000.0, 6000
    t, quats = _random_walk_attitude(n, fs, 777)
    omega = at.quaternions_to_angular_velocity(t, quats, degrees=True)
    imu_rate = (len(omega) - 1) / ((omega.t[-1] - omega.t[0]) / 1000.0)
    fps, span, inj = 30.0, (n - 1) * 1.0, 8.5

    clean = at.find_offset(_video_signal(omega, span, fps, inj, 0.0, np.random.default_rng(99)),
                           omega, 0.0, 80.0, fps, imu_rate, 20.0).offset_ms
    assert abs(clean - inj) < 0.6
    for sigma in (1.0, 3.0, 5.0):
        r = at.find_offset(_video_signal(omega, span, fps, inj, sigma, np.random.default_rng(99)),
                           omega, 0.0, 80.0, fps, imu_rate, 20.0)
        assert r.found
        assert abs(r.offset_ms - clean) < 0.4, f"noise {sigma} drifted recovery to {r.offset_ms:.3f}"


def test_interp_removes_quantization_bias():
    """Coarse IMU + fine video isolates the nearest-upper quantization bias.

    A 200 Hz IMU snaps every query up to the next 5 ms sample, so nearest-lookup carries a
    large systematic offset (~+2 ms). Linear interpolation (interp=True) removes that bias,
    bringing the mean recovery error under 0.05 ms; parabolic adds a sub-grid vertex on top.
    The per-injection scatter (~0.1 ms) is video-sampling/motion-conditioning limited and is
    not what interp targets, so the assertion is on the mean (bias), averaged over injections.
    """
    fs, n = 1000.0, 12000
    t, quats = _random_walk_attitude(n, fs, 777)
    omega = at.quaternions_to_angular_velocity(t, quats, degrees=True)
    span = (n - 1) * 1.0

    imu_hz, fps = 200.0, 120.0  # coarse IMU (5 ms grid), fine video (small video-side error)
    imu = at.resample_angular_velocity(omega, np.arange(0.0, span + 1e-6, 1000.0 / imu_hz))
    vts = np.arange(0.0, span + 1e-6, 1000.0 / fps)
    injects = (-20.0, -15.0, -7.3, -2.0, 0.0, 3.5, 8.5, 12.0, 18.0)

    def bias(**kw):
        errs = []
        for inj in injects:
            video = at.resample_angular_velocity(omega, vts - inj)
            video.t = vts.copy()
            r = at.find_offset(video, imu, 0.0, 80.0, fps, imu_hz, 20.0, **kw)
            assert r.found
            errs.append(r.offset_ms - inj)
        return float(np.mean(errs))

    near = bias()
    interp = bias(interp=True)
    parab = bias(interp=True, parabolic=True)
    assert near > 1.0, f"expected a large nearest-lookup bias, got {near:.4f} ms"
    assert abs(interp) < 0.05, f"interp bias {interp:.4f} ms not under 0.05 ms"
    assert abs(parab) < 0.05, f"parabolic bias {parab:.4f} ms not under 0.05 ms"


def test_load_angular_velocity_csv(tmp_path=None):
    """Generic CSV loader: column auto-detect, unit inference, ms timestamps, re-basing."""
    import os
    import tempfile
    d = tempfile.mkdtemp()
    # deg/s via column suffix; timestamps already ms but offset from 0 (should re-base).
    p = os.path.join(d, "gen.csv")
    with open(p, "w") as f:
        f.write("timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s\n")
        for i in range(5):
            f.write(f"{100.0 + i:.1f},{i*1.0},{i*2.0},{i*3.0}\n")
    s = at.load_angular_velocity_csv(p)
    assert len(s) == 5
    assert abs(s.t[0]) < 1e-9 and abs(s.t[-1] - 4.0) < 1e-9   # re-based to 0
    assert abs(s.w[2, 2] - 6.0) < 1e-9                        # wz at i=2 == 6 deg/s

    # rad/s columns convert to deg/s.
    pr = os.path.join(d, "rad.csv")
    with open(pr, "w") as f:
        f.write("t_s,wx_rad_s,wy_rad_s,wz_rad_s\n0,1.0,0,0\n0.001,1.0,0,0\n")
    sr = at.load_angular_velocity_csv(pr)
    assert abs(sr.t[1] - 1.0) < 1e-9                          # 0.001 s -> 1 ms
    assert abs(sr.w[0, 0] - np.degrees(1.0)) < 1e-6          # 1 rad/s -> 57.2958 deg/s


def test_load_gcsv():
    """GCSV loader honours tscale/gscale and the metadata block, returns deg/s."""
    import os
    import tempfile
    p = os.path.join(tempfile.mkdtemp(), "log.gcsv")
    with open(p, "w") as f:
        f.write("GYROFLOW IMU LOG\nversion,1.3\ntscale,0.001\ngscale,1.0\n")
        f.write("t,gx,gy,gz\n")
        f.write("0,1.0,0,0\n1,1.0,0,0\n2,1.0,0,0\n")
    s = at.load_gcsv(p)
    assert len(s) == 3
    assert abs(s.t[1] - 1.0) < 1e-9                           # tscale 0.001 s * 1000 = 1 ms
    assert abs(s.w[0, 0] - np.degrees(1.0)) < 1e-6          # gscale 1 rad/s -> deg/s
    # load_motion must auto-route this to the GCSV path (metadata before the t header).
    s2 = at.load_motion(p)
    assert len(s2) == 3 and abs(s2.w[0, 0] - np.degrees(1.0)) < 1e-6


def test_sync_recovery_two_signals():
    """End-to-end: write two real CSVs with a known offset, sync them, recover the offset."""
    import os
    import tempfile
    fs, n = 1000.0, 6000
    t, quats = _random_walk_attitude(n, fs, 777)
    omega = at.quaternions_to_angular_velocity(t, quats, degrees=True)
    span = (n - 1) * 1.0
    inj = 8.5
    d = tempfile.mkdtemp()

    # IMU at 200 Hz on its own clock; "video" at 60 fps delayed by inj on the same motion.
    gt = np.arange(0.0, span + 1e-6, 1000.0 / 200.0)
    gyro = at.resample_angular_velocity(omega, gt)
    vts = np.arange(0.0, span + 1e-6, 1000.0 / 60.0)
    video = at.resample_angular_velocity(omega, vts - inj)
    video.t = vts.copy()

    def dump(path, s):
        with open(path, "w") as f:
            f.write("timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s\n")
            for i in range(len(s)):
                f.write(f"{s.t[i]:.4f},{s.w[i,0]:.6f},{s.w[i,1]:.6f},{s.w[i,2]:.6f}\n")

    gp, vp = os.path.join(d, "imu.csv"), os.path.join(d, "video.csv")
    dump(gp, gyro)
    dump(vp, video)

    g = at.load_motion(gp)
    v = at.load_motion(vp)
    gr, vr = at.estimate_sample_rate_hz(g), at.estimate_sample_rate_hz(v)
    assert abs(gr - 200.0) < 1.0 and abs(vr - 60.0) < 1.0
    r = at.find_offset(v, g, 0.0, 80.0, vr, gr, 20.0, interp=True, parabolic=True)
    assert r.found
    assert abs(r.offset_ms - inj) < 0.3, f"sync recovered {r.offset_ms:.3f}, expected {inj}"


def _run_all():
    fns = [test_lowpass, test_omega, test_recovery, test_noise_robustness,
           test_interp_removes_quantization_bias, test_load_angular_velocity_csv,
           test_load_gcsv, test_sync_recovery_two_signals]
    failures = 0
    for fn in fns:
        try:
            fn()
            print(f"{fn.__name__} OK")
        except AssertionError as e:
            failures += 1
            print(f"{fn.__name__} FAIL: {e}")
    if failures == 0:
        print("all python autosync tests passed")
    else:
        print(f"{failures} test(s) FAILED")
    return failures


if __name__ == "__main__":
    raise SystemExit(1 if _run_all() else 0)
