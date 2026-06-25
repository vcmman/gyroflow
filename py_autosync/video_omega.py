#!/usr/bin/env python3
"""Estimate camera angular velocity from an MP4 with OpenCV — a port of Gyroflow's autosync
video-motion path (optical flow -> essential matrix -> rotation -> angular velocity).

Pipeline per consecutive (sampled) frame pair, matching
``src/core/synchronization/`` in Gyroflow:

  1. goodFeaturesToTrack on the previous frame  (maxCorners 200, quality 0.01, minDist 10, ...)
  2. calcOpticalFlowPyrLK to the next frame      (win 21x21, 3 levels, minEig 1e-4); keep status==1
  3. normalize points by the camera matrix K, then findEssentialMat (LMEDS) + recoverPose -> R
  4. rotvec = axisAngle(R) / dt;  swap X/Y; convert to deg/s;  timestamp at the frame-pair midpoint

The result is a :class:`autosync_time.GyroSeries` (deg/s, ms) in the *same* convention as
Gyroflow's ``estimated_gyro`` (X/Y swapped), so it feeds straight into ``find_offset`` / the
``sync`` CLI / ``visualize.py`` as the ``--video`` signal.

OpenCV is imported lazily so the rest of the toolkit keeps working without it.

CLI:
  ./video_omega.py clip.mp4 --fov-deg 50 --every-nth 1 > omega.csv
  ./video_omega.py clip.mp4 --focal-px 1800 --downscale 2 --out omega.csv
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional

import numpy as np

import autosync_time as at

# --- Gyroflow's parameters (src/core/synchronization/optical_flow/opencv_pyrlk.rs) ---
_MAX_CORNERS = 200
_QUALITY = 0.01
_MIN_DIST = 10.0
_BLOCK = 3
_HARRIS_K = 0.04
_LK_WIN = (21, 21)
_LK_LEVELS = 3
_LK_MIN_EIG = 1e-4
_MIN_POINTS = 10           # both PyrLK valid points and recoverPose inliers
_RAD2DEG = 180.0 / np.pi
_VIDEO_EXTS = (".mp4", ".mov", ".mkv", ".avi", ".m4v", ".webm", ".mts", ".m2ts")


def is_video_path(path: str) -> bool:
    return path.lower().endswith(_VIDEO_EXTS)


def camera_matrix(width: int, height: int, focal_px: Optional[float] = None,
                  fov_deg: Optional[float] = None) -> np.ndarray:
    """Pinhole intrinsics K. Focal from ``focal_px`` (pixels), else from a horizontal ``fov_deg``,
    else a default ~50° HFOV. Principal point at the image centre.

    For autosync only the *timing* of the motion matters; a focal-length error mostly rescales the
    amplitude, not the offset. Pass the real focal/FOV when you have it for the cleanest signal.
    """
    if focal_px is None:
        fov = fov_deg if fov_deg is not None else 50.0
        focal_px = (width * 0.5) / np.tan(np.radians(fov) * 0.5)
    cx, cy = width * 0.5, height * 0.5
    return np.array([[focal_px, 0.0, cx], [0.0, focal_px, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def _estimate_rotvec(prev_gray, gray, K, cv2, min_points: int = _MIN_POINTS):
    """Inter-frame rotation as an axis-angle vector (radians), or None if not estimable."""
    h, w = prev_gray.shape[:2]
    feats = cv2.goodFeaturesToTrack(prev_gray, _MAX_CORNERS, _QUALITY, _MIN_DIST,
                                    blockSize=_BLOCK, useHarrisDetector=False, k=_HARRIS_K)
    if feats is None or len(feats) < min_points:
        return None
    p1 = feats.reshape(-1, 1, 2).astype(np.float32)
    crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.01)
    p2, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p1, None, winSize=_LK_WIN,
                                             maxLevel=_LK_LEVELS, criteria=crit, flags=0,
                                             minEigThreshold=_LK_MIN_EIG)
    if p2 is None or status is None:
        return None
    a, b = p1.reshape(-1, 2), p2.reshape(-1, 2)
    st = status.reshape(-1) == 1
    inb = ((a[:, 0] >= 0) & (a[:, 0] < w) & (a[:, 1] >= 0) & (a[:, 1] < h) &
           (b[:, 0] >= 0) & (b[:, 0] < w) & (b[:, 1] >= 0) & (b[:, 1] < h))
    keep = st & inb
    if int(np.count_nonzero(keep)) < min_points:
        return None
    pts1, pts2 = a[keep].astype(np.float64), b[keep].astype(np.float64)

    # Undistort to normalized camera coordinates (no distortion model -> pinhole K^-1).
    pp = np.array([K[0, 2], K[1, 2]])
    n1 = (pts1 - pp) / np.array([K[0, 0], K[1, 1]])
    n2 = (pts2 - pp) / np.array([K[0, 0], K[1, 1]])

    I = np.eye(3, dtype=np.float64)
    E, _ = cv2.findEssentialMat(n1, n2, I, method=cv2.LMEDS, prob=0.999,
                                threshold=1e-5, maxIters=4000)
    if E is None or E.shape[0] < 3:
        return None
    E = E[:3, :3]
    # Match Gyroflow's recover_pose_triangulated: identity camera (points are normalized) with a
    # large distance threshold so far/low-parallax points still pass the cheirality check.
    out = cv2.recoverPose(E, n1, n2, I, distanceThresh=100000.0)
    inliers, R = out[0], out[1]
    if inliers < min_points:
        return None
    rvec, _ = cv2.Rodrigues(R)
    return rvec.reshape(3)


def _rotvec_to_omega(rotvec_rad: np.ndarray, dt_s: float, swap_xy: bool) -> np.ndarray:
    """axis-angle (rad) over dt -> angular velocity (deg/s), Gyroflow's X/Y swap applied."""
    w = (rotvec_rad / dt_s) * _RAD2DEG
    return w[[1, 0, 2]] if swap_xy else w


def omega_from_frames(gray_frames, timestamps_ms, K, every_nth: int = 1,
                      swap_xy: bool = True, min_points: int = _MIN_POINTS,
                      progress: bool = False) -> at.GyroSeries:
    """Angular velocity from an iterable/list of grayscale frames + their timestamps (ms).

    ``gray_frames`` may be a list or any sequence of 2-D uint8 arrays. Frames are paired
    ``i -> i+every_nth``; ω for each pair is timestamped at the pair midpoint. cv2 is imported here.
    """
    import cv2  # lazy

    frames = list(gray_frames)
    ts = np.asarray(timestamps_ms, dtype=float)
    t_out, w_out = [], []
    n = len(frames)
    for i in range(0, n - every_nth, every_nth):
        j = i + every_nth
        dt_s = (ts[j] - ts[i]) / 1000.0
        if dt_s <= 1e-9:
            continue
        rotvec = _estimate_rotvec(frames[i], frames[j], K, cv2, min_points)
        if rotvec is None:
            continue
        t_out.append(0.5 * (ts[i] + ts[j]))
        w_out.append(_rotvec_to_omega(rotvec, dt_s, swap_xy))
        if progress and (len(t_out) % 50 == 0):
            print(f"  ... {len(t_out)} frame pairs estimated", file=sys.stderr)
    if not t_out:
        return at.GyroSeries(np.array([]), np.zeros((0, 3)))
    order = np.argsort(t_out, kind="stable")
    return at.GyroSeries(np.asarray(t_out)[order], np.asarray(w_out)[order])


def video_to_omega(path: str, focal_px: Optional[float] = None, fov_deg: Optional[float] = None,
                   every_nth: int = 1, downscale: float = 1.0, max_frames: Optional[int] = None,
                   start_sec: float = 0.0, swap_xy: bool = True, min_points: int = _MIN_POINTS,
                   progress: bool = False) -> at.GyroSeries:
    """Read an MP4 with OpenCV and return camera angular velocity as a :class:`GyroSeries`.

    Real per-frame timestamps come from ``CAP_PROP_POS_MSEC`` (VFR-safe), falling back to
    ``frame_index / fps``. ``downscale`` shrinks frames for speed (K scales with them, so the
    normalized geometry is unchanged). ``every_nth`` / ``max_frames`` / ``start_sec`` bound the work.
    """
    import cv2  # lazy

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise IOError(f"could not open video: {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    if start_sec > 0.0:
        cap.set(cv2.CAP_PROP_POS_MSEC, start_sec * 1000.0)

    grays, ts_ms = [], []
    idx = 0
    while True:
        if max_frames is not None and len(grays) >= max_frames:
            break
        ok, frame = cap.read()
        if not ok:
            break
        pos = cap.get(cv2.CAP_PROP_POS_MSEC)
        t = pos if pos and pos > 0.0 else (idx / fps * 1000.0 if fps > 0.0 else float(idx))
        if downscale and downscale > 1.0:
            frame = cv2.resize(frame, (0, 0), fx=1.0 / downscale, fy=1.0 / downscale,
                               interpolation=cv2.INTER_AREA)
        grays.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        ts_ms.append(t)
        idx += 1
    cap.release()

    if len(grays) < 2:
        raise ValueError(f"video had < 2 readable frames: {path}")
    h, w = grays[0].shape[:2]
    eff_focal = (focal_px / downscale) if (focal_px and downscale > 1.0) else focal_px
    K = camera_matrix(w, h, eff_focal, fov_deg)
    if progress:
        print(f"read {len(grays)} frames @ ~{fps:.2f} fps, {w}x{h}, focal_px~{K[0,0]:.1f}",
              file=sys.stderr)
    return omega_from_frames(grays, ts_ms, K, every_nth, swap_xy, min_points, progress)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Estimate camera angular velocity from an MP4 (OpenCV)")
    p.add_argument("video", help="input video file (mp4/mov/mkv/...)")
    p.add_argument("--focal-px", type=float, default=None, help="focal length in pixels (original res)")
    p.add_argument("--fov-deg", type=float, default=None, help="horizontal FOV in degrees (if no focal)")
    p.add_argument("--every-nth", type=int, default=1, help="process every Nth frame (speed)")
    p.add_argument("--downscale", type=float, default=1.0, help="shrink frames by this factor (speed)")
    p.add_argument("--max-frames", type=int, default=None, help="stop after N frames")
    p.add_argument("--start-sec", type=float, default=0.0, help="seek to this time before reading")
    p.add_argument("--no-swap-xy", action="store_true", help="do not swap X/Y (default swaps, as Gyroflow)")
    p.add_argument("--out", default=None, help="write CSV here (default: stdout)")
    args = p.parse_args(argv)

    series = video_to_omega(args.video, focal_px=args.focal_px, fov_deg=args.fov_deg,
                            every_nth=args.every_nth, downscale=args.downscale,
                            max_frames=args.max_frames, start_sec=args.start_sec,
                            swap_xy=not args.no_swap_xy, progress=True)
    if len(series) == 0:
        print("no angular velocity estimated (too little motion / texture?)", file=sys.stderr)
        return 1
    out = open(args.out, "w") if args.out else sys.stdout
    try:
        out.write("timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s\n")
        for i in range(len(series)):
            t, wv = series.t[i], series.w[i]
            out.write(f"{t:.4f},{wv[0]:.4f},{wv[1]:.4f},{wv[2]:.4f}\n")
    finally:
        if args.out:
            out.close()
    print(f"estimated {len(series)} samples, max|omega| {at.max_angle(series):.2f} deg/s",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
