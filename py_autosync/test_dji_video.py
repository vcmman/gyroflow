"""End-to-end test on a real DJI MP4 (gated on the footage being present).

Runs the full production path on actual DJI OsmoAction4 footage:
  MP4 -> video_omega (fisheye-undistorted optical flow + essential matrix) -> camera angular
  velocity, synced against the IMU angular velocity derived from the DJI fused-quaternion telemetry.

It is SKIPPED (not failed) when the data is not available, so it is safe to run anywhere. Point it
at footage with env vars, or drop the files in a discoverable `data/` dir:

  DJI_VIDEO   path to the .MP4                     (default: search known data dirs for DJI_*.MP4)
  DJI_QUAT    path to dji_quaternions_full.csv     (default: alongside the video / data dir)
  DJI_LENS    path to a .gyroflow / lens profile   (default: the .gyroflow next to the video)

Run:  python3 test_dji_video.py     (or via pytest)
"""

from __future__ import annotations

import glob
import os

import numpy as np

import autosync_time as at

try:
    import cv2  # noqa: F401
    import video_omega as vo
    HAVE_CV2 = True
except Exception:
    HAVE_CV2 = False

# Camera frame -> DJI IMU frame for this telemetry, found by a signed-permutation correlation
# sweep. The net mapping on the raw axis-angle is [-x, +y, +z]; applied on top of video_omega's
# default X/Y swap (Gyroflow parity) the equivalent orientation string is "yXZ". This is what the
# production CLI uses too: `--video-orientation yXZ`.
_DJI_ORIENTATION = "yXZ"

_DATA_DIRS = [
    os.environ.get("GYROFLOW_DATA", ""),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "data"),
    "/home/yc/src/gyroflow/data",
]


def _find_dji():
    """Locate (video, quat_csv, lens_json) from env vars or known data dirs; None if unavailable."""
    video = os.environ.get("DJI_VIDEO")
    quat = os.environ.get("DJI_QUAT")
    lens = os.environ.get("DJI_LENS")
    for d in _DATA_DIRS:
        if not d or not os.path.isdir(d):
            continue
        if not video:
            vids = sorted(glob.glob(os.path.join(d, "DJI_*.MP4")) + glob.glob(os.path.join(d, "DJI_*.mp4")))
            vids = [v for v in vids if "stabilized" not in os.path.basename(v).lower()]
            if vids:
                video = vids[0]
        if not quat:
            q = os.path.join(d, "dji_quaternions_full.csv")
            if os.path.exists(q):
                quat = q
        if not lens:
            gf = sorted(glob.glob(os.path.join(d, "*.gyroflow")))
            if gf:
                lens = gf[0]
    if video and quat and os.path.exists(video) and os.path.exists(quat):
        return video, quat, (lens if lens and os.path.exists(lens) else None)
    return None


def test_dji_video_sync():
    if not HAVE_CV2:
        print("test_dji_video_sync SKIPPED (no cv2)")
        return
    found = _find_dji()
    if not found:
        print("test_dji_video_sync SKIPPED (no DJI footage; set DJI_VIDEO / DJI_QUAT [/ DJI_LENS])")
        return
    video, quat, lens = found
    print(f"DJI video: {os.path.basename(video)}  lens: {os.path.basename(lens) if lens else '(none)'}")

    # Camera-side omega from a downscaled slice of the real video (fisheye-undistorted if lens
    # given). Uses the production defaults (incl. swap_xy=True) so it matches the `sync` CLI exactly.
    vid = vo.video_to_omega(video, lens=lens, downscale=4.0, max_frames=200,
                            orientation=_DJI_ORIENTATION, progress=False)
    assert len(vid) >= 100, f"only {len(vid)} omega samples from video"
    assert np.all(np.isfinite(vid.w))

    # IMU-side omega from the DJI fused quaternions.
    t, q = at.load_quaternions(quat)
    imu = at.quaternions_to_angular_velocity(t, q, degrees=True)

    # The recovered camera motion must track the IMU motion (magnitude correlation over the window).
    iw = np.empty((len(vid), 3))
    for ax in range(3):
        iw[:, ax] = np.interp(vid.t, imu.t, imu.w[:, ax])
    corr = float(np.corrcoef(np.linalg.norm(vid.w, axis=1), np.linalg.norm(iw, axis=1))[0, 1])
    print(f"video<->IMU |omega| correlation = {corr:.3f}")
    assert corr > 0.7, f"video/IMU motion correlation too low ({corr:.3f}) — bad lens/axes?"

    # And the offset finder locks to a small, sane delay.
    r = at.find_offset(vid, imu, 0.0, 200.0, at.estimate_sample_rate_hz(vid),
                       at.estimate_sample_rate_hz(imu), 20.0, interp=True)
    print(f"offset_ms = {r.offset_ms:.2f}  cost = {r.cost:.0f}  matched = {r.matched}/{len(vid)}")
    assert r.found, "no acceptable offset found on DJI footage"
    assert abs(r.offset_ms) < 50.0, f"implausible offset {r.offset_ms:.2f} ms"


def _run_all():
    failures = 0
    for fn in [test_dji_video_sync]:
        try:
            fn()
            print(f"{fn.__name__} OK")
        except AssertionError as e:
            failures += 1
            print(f"{fn.__name__} FAIL: {e}")
    print("all DJI video tests passed" if failures == 0 else f"{failures} test(s) FAILED")
    return failures


if __name__ == "__main__":
    raise SystemExit(1 if _run_all() else 0)
