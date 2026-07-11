"""Shared image-domain video metrics for cpp_core stabilization evaluation.

Used by tools/{vertical_flow_compare,black_border_stats,rust_vs_cpp_dy}.py so the
metric definitions live in one place. See cpp_core/figures/README.md for the write-up.
"""
import os
import cv2
import numpy as np


def translation_flow(path, width=640, max_frames=None):
    """Per-frame global (dx, dy) shift (px) between consecutive frames.

    Estimates the single global sub-pixel translation with cv2.phaseCorrelate (a
    Hanning window suppresses FFT edge effects). Frames are resized to `width` px
    wide first, so dx/dy are in pixels at that scale.
    Returns a (n_frames - 1, 2) np.ndarray with columns [dx, dy].
    """
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {path}")
    ok, prev = cap.read()
    if not ok:
        raise RuntimeError(f"empty video {path}")
    scale = width / prev.shape[1]
    size = (width, int(round(prev.shape[0] * scale)))
    win = cv2.createHanningWindow(size, cv2.CV_32F)

    def prep(f):
        return np.float32(cv2.cvtColor(cv2.resize(f, size), cv2.COLOR_BGR2GRAY))

    prev_g = prep(prev)
    dxys = []
    i = 0
    while True:
        ok, cur = cap.read()
        if not ok:
            break
        cur_g = prep(cur)
        (dx, dy), _resp = cv2.phaseCorrelate(prev_g, cur_g, win)
        dxys.append((dx, dy))
        prev_g = cur_g
        i += 1
        if i % 200 == 0:
            print(f"  {os.path.basename(path)}: {i} frames", flush=True)
        if max_frames and i >= max_frames:
            break
    cap.release()
    return np.asarray(dxys)


def vertical_flow(path, width=640, max_frames=None):
    """Per-frame global vertical shift dy (px); see translation_flow."""
    return translation_flow(path, width, max_frames)[:, 1]


def edge_black_series(path, width=480, thresh=8, stride=5, max_frames=None):
    """Per-(sampled)-frame black-border fraction (% of frame).

    Near-black pixels (<= thresh) that are connected to the frame edge (flood from
    the border via connected components) — this ignores genuinely dark scene content
    not touching an edge. `stride` sub-samples frames (grab-skip) for speed.
    Returns (idxs, fracs) as np.ndarrays.
    """
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {path}")
    fracs, idxs = [], []
    i = -1
    got = 0
    size = None
    while True:
        if not cap.grab():
            break
        i += 1
        if i % stride != 0:
            continue
        ok, f = cap.retrieve()
        if not ok:
            break
        if size is None:
            scale = width / f.shape[1]
            size = (width, int(round(f.shape[0] * scale)))
        g = cv2.cvtColor(cv2.resize(f, size), cv2.COLOR_BGR2GRAY)
        mask = (g <= thresh).astype(np.uint8)
        area = 0
        if mask.any():
            _num, labels = cv2.connectedComponents(mask, connectivity=8)
            border = np.concatenate([labels[0, :], labels[-1, :],
                                     labels[:, 0], labels[:, -1]])
            border_lbls = np.unique(border)
            border_lbls = border_lbls[border_lbls != 0]
            if border_lbls.size:
                area = np.isin(labels, border_lbls).sum()
        fracs.append(100.0 * area / (size[0] * size[1]))
        idxs.append(i)
        got += 1
        if got % 200 == 0:
            print(f"  {os.path.basename(path)}: {got} sampled (frame {i})", flush=True)
        if max_frames and i >= max_frames:
            break
    cap.release()
    return np.asarray(idxs), np.asarray(fracs)
