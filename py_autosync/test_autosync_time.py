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


def _run_all():
    fns = [test_lowpass, test_omega, test_recovery, test_noise_robustness]
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
