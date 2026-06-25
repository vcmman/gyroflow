"""Tests for video_omega.py (MP4 -> angular velocity via OpenCV).

Run standalone:  python3 test_video_omega.py
Skips the OpenCV-dependent tests with a notice if cv2 is unavailable.

Strategy:
 - Pure helpers (camera matrix, rotvec->omega swap, path sniffing) tested exactly.
 - Pose geometry tested on exact synthetic 3D point correspondences (well-posed two-view case),
   verifying recoverPose + the X/Y-swapped deg/s conversion the module relies on.
 - Full optical-flow pipeline tested on a rendered 3D dot-cloud scene under a known constant
   camera angular velocity + translation (real parallax), asserting recovery within tolerance.
"""

from __future__ import annotations

import numpy as np

import autosync_time as at
import video_omega as vo

try:
    import cv2
    HAVE_CV2 = True
except Exception:
    HAVE_CV2 = False

_RAD2DEG = 180.0 / np.pi


# ----------------------------- pure helpers -----------------------------

def test_is_video_path():
    assert vo.is_video_path("clip.MP4") and vo.is_video_path("a/b.mov")
    assert not vo.is_video_path("omega.csv") and not vo.is_video_path("imu.gcsv")


def test_camera_matrix():
    K = vo.camera_matrix(1000, 500, fov_deg=90.0)
    assert abs(K[0, 2] - 500.0) < 1e-9 and abs(K[1, 2] - 250.0) < 1e-9   # centre
    assert abs(K[0, 0] - 500.0) < 1e-6                                    # f = (w/2)/tan(45)=w/2
    K2 = vo.camera_matrix(800, 600, focal_px=1234.0)
    assert abs(K2[0, 0] - 1234.0) < 1e-9 and abs(K2[1, 1] - 1234.0) < 1e-9


def test_rotvec_to_omega_swap():
    # rotvec (rad) over dt -> deg/s, with Gyroflow's X/Y swap.
    rv = np.array([0.01, 0.02, 0.03])
    dt = 0.5
    w = vo._rotvec_to_omega(rv, dt, swap_xy=True)
    exp = (rv / dt) * _RAD2DEG
    assert np.allclose(w, exp[[1, 0, 2]])
    w0 = vo._rotvec_to_omega(rv, dt, swap_xy=False)
    assert np.allclose(w0, exp)


# ----------------------------- geometry helpers -----------------------------

def _project(K, R, t, X):
    """Project Nx3 world points through camera (R,t). Returns Nx2 pixels and depth mask."""
    Xc = (R @ X.T + t.reshape(3, 1)).T
    z = Xc[:, 2]
    px = (K @ Xc.T).T
    pix = px[:, :2] / px[:, 2:3]
    return pix, z


def test_pose_recovery_correspondences():
    """recoverPose on exact projected correspondences returns the injected rotation, and the
    module's conversion yields the right swapped deg/s omega."""
    if not HAVE_CV2:
        print("test_pose_recovery_correspondences SKIPPED (no cv2)")
        return
    rng = np.random.default_rng(3)
    W, H, f = 960, 540, 800.0
    K = vo.camera_matrix(W, H, focal_px=f)
    X = np.column_stack([rng.uniform(-3, 3, 400), rng.uniform(-3, 3, 400), rng.uniform(5, 12, 400)])

    omega_cam = np.array([0.04, 0.06, 0.01])        # rad (over this step) — small rotation
    R = cv2.Rodrigues(omega_cam)[0]
    t = np.array([0.6, 0.15, 0.0])                  # translation gives parallax (well-posed)

    p1, z1 = _project(K, np.eye(3), np.zeros(3), X)
    p2, z2 = _project(K, R, t, X)
    good = (z1 > 0) & (z2 > 0)
    pp = np.array([K[0, 2], K[1, 2]]); fxy = np.array([K[0, 0], K[1, 1]])
    n1 = (p1[good] - pp) / fxy
    n2 = (p2[good] - pp) / fxy

    E, _ = cv2.findEssentialMat(n1, n2, np.eye(3), method=cv2.LMEDS, prob=0.999,
                                threshold=1e-5, maxIters=4000)
    E = E[:3, :3]
    inliers, Rrec, _t, _m = cv2.recoverPose(E, n1, n2)
    assert inliers >= 50
    rvec_rec = cv2.Rodrigues(Rrec)[0].reshape(3)
    # recovered rotation vector matches the injected one (radians), within ~0.5 deg
    assert np.max(np.abs(rvec_rec - omega_cam)) * _RAD2DEG < 0.5, f"rvec {rvec_rec} vs {omega_cam}"
    # and the module's deg/s + swap conversion is consistent
    w = vo._rotvec_to_omega(rvec_rec, 1.0, swap_xy=True)
    assert np.allclose(w, (omega_cam * _RAD2DEG)[[1, 0, 2]], atol=0.5)


# ----------------------------- rendered-scene pipeline -----------------------------

def _render(K, R, t, X, bright, W, H):
    img = np.full((H, W), 110, np.uint8)
    pix, z = _project(K, R, t, X)
    for (x, y), zz, b in zip(pix, z, bright):
        if zz > 0 and 0 <= x < W and 0 <= y < H:
            cv2.circle(img, (int(round(x)), int(round(y))), 3, int(b), -1)
    return cv2.GaussianBlur(img, (0, 0), 0.7)


def _make_scene_frames(fps, n, amp=0.18, v0=5.0, seed=2, W=960, H=540, f=800.0):
    """Render n frames of a 3D dot cloud under a TIME-VARYING camera angular velocity (+ a steady
    translation for parallax). Returns (gray_frames, timestamps_ms, K, truth_t_ms, truth_w_degs)
    where truth_w is already in the module's X/Y-swapped deg/s output convention."""
    rng = np.random.default_rng(seed)
    K = vo.camera_matrix(W, H, focal_px=f)
    X = np.column_stack([rng.uniform(-9, 9, 800), rng.uniform(-7, 7, 800), rng.uniform(4, 10, 800)])
    bright = rng.integers(180, 256, 800)
    dt = 1.0 / fps
    R = np.eye(3); t = np.zeros(3)
    frames, ts, truth_t, truth_w = [], [], [], []
    for i in range(n):
        wc = np.array([amp * np.sin(2 * np.pi * 0.7 * i * dt),   # camera-frame omega (rad/s)
                       amp * np.cos(2 * np.pi * 0.5 * i * dt), 0.0])
        frames.append(_render(K, R, t, X, bright, W, H))
        ts.append(i * dt * 1000.0)
        truth_t.append(i * dt * 1000.0)
        truth_w.append((wc * _RAD2DEG)[[1, 0, 2]])              # module swaps X/Y
        R = cv2.Rodrigues(wc * dt)[0] @ R
        t = t + np.array([v0, 0.3 * v0, 0.0]) * dt
    return frames, np.array(ts), K, np.array(truth_t), np.array(truth_w)


def _corr(series, truth_t, truth_w, ax):
    ti = np.interp(series.t, truth_t, truth_w[:, ax])
    return float(np.corrcoef(series.w[:, ax], ti)[0, 1])


def test_pipeline_tracks_timevarying_omega():
    """The full goodFeatures->PyrLK->essential->recoverPose pipeline recovers a time-varying camera
    angular velocity: the estimated series correlates strongly with the injected motion (this is the
    time-shape autosync actually relies on; the essential-matrix amplitude is only approximate)."""
    if not HAVE_CV2:
        print("test_pipeline_tracks_timevarying_omega SKIPPED (no cv2)")
        return
    frames, ts, K, tt, tw = _make_scene_frames(fps=30.0, n=40, seed=2)
    series = vo.omega_from_frames(frames, ts, K, every_nth=1, swap_xy=True)
    assert len(series) >= 30, f"only {len(series)} estimates"
    assert np.all(np.isfinite(series.w))
    assert np.all(np.diff(series.t) > 0)                       # midpoints, strictly increasing
    assert abs(series.t[0] - (1000.0 / 30.0) / 2.0) < 1e-6
    cx, cy = _corr(series, tt, tw, 0), _corr(series, tt, tw, 1)
    assert cx > 0.6 and cy > 0.8, f"correlation too low: x={cx:.2f} y={cy:.2f}"


def test_mp4_roundtrip():
    """Best-effort: encode the rendered scene to a real .mp4 and read it back via video_to_omega,
    checking the dominant-axis motion still correlates after a full encode/decode cycle."""
    if not HAVE_CV2:
        print("test_mp4_roundtrip SKIPPED (no cv2)")
        return
    import os
    import tempfile
    frames, ts, K, tt, tw = _make_scene_frames(fps=30.0, n=40, seed=2)
    H, W = frames[0].shape
    path = os.path.join(tempfile.mkdtemp(), "scene.mp4")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (W, H), isColor=True)
    if not writer.isOpened():
        print("test_mp4_roundtrip SKIPPED (no mp4 encoder)")
        return
    for g in frames:
        writer.write(cv2.cvtColor(g, cv2.COLOR_GRAY2BGR))
    writer.release()
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        print("test_mp4_roundtrip SKIPPED (encoder produced no file)")
        return

    series = vo.video_to_omega(path, focal_px=K[0, 0])
    assert len(series) >= 20 and np.all(np.isfinite(series.w))
    assert _corr(series, tt, tw, 1) > 0.6, "y-axis motion did not survive the mp4 round-trip"


def _run_all():
    fns = [test_is_video_path, test_camera_matrix, test_rotvec_to_omega_swap,
           test_pose_recovery_correspondences, test_pipeline_tracks_timevarying_omega,
           test_mp4_roundtrip]
    failures = 0
    for fn in fns:
        try:
            fn()
            print(f"{fn.__name__} OK")
        except AssertionError as e:
            failures += 1
            print(f"{fn.__name__} FAIL: {e}")
    if failures == 0:
        print("all video_omega tests passed")
    else:
        print(f"{failures} test(s) FAILED")
    return failures


if __name__ == "__main__":
    raise SystemExit(1 if _run_all() else 0)
