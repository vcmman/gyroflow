"""UI-free Python port of Gyroflow's gyro<->video time-offset finder ("autosync time").

A faithful re-implementation of the same algorithm ported to C++ in ``cpp_core/`` and used by
the Rust engine:

* ``src/core/synchronization/find_offset/essential_matrix.rs`` -> :func:`find_offset`
* ``src/core/filtering.rs``                                    -> :class:`Lowpass`
* ``src/core/synchronization/mod.rs``                          -> estimated_gyro units/axes

Given two angular-velocity time series (one from the video, one from the IMU), it finds the
constant timestamp delay that best aligns them. The video-side signal is synthesised from DJI
fused quaternions rather than decoded from pixels, which also enables ground-truthed accuracy
evaluation (inject a known offset, measure how precisely it is recovered).

Depends on numpy and scipy (scipy.signal.lfilter is used for the IIR pass; with zero initial
state it produces output identical to the Rust biquad ``DirectForm2Transposed``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from scipy.signal import lfilter

# biquad::Q_BUTTERWORTH_F64
_Q_BUTTERWORTH = 0.7071067811865476


# ---------------------------------------------------------------------------
# Quaternions (w, x, y, z), small helpers — match cpp_core/src/quaternion.cpp.
# ---------------------------------------------------------------------------

def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n <= 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_inverse_unit(q: np.ndarray) -> np.ndarray:
    """Inverse of a (near-)unit quaternion == conjugate after normalisation."""
    q = quat_normalize(q)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_from_axis_angle(axis: np.ndarray, radians: float) -> np.ndarray:
    n = np.linalg.norm(axis)
    if n <= 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    half = radians * 0.5
    s = np.sin(half) / n
    return np.array([np.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])


# ---------------------------------------------------------------------------
# Lowpass — RBJ Butterworth biquad, forward-backward (zero-phase).
# ---------------------------------------------------------------------------

class Lowpass:
    """Butterworth (Q = 1/sqrt(2)) low-pass, matching the Rust ``biquad`` crate.

    ``filter_gyro_forward_backward`` reproduces Gyroflow exactly: filter forward, then filter the
    result backward (no scipy ``filtfilt`` edge padding). Returns the data unchanged when
    ``2*freq > sample_rate`` (the crate's ``OutsideNyquist`` case, which Gyroflow ignores).
    """

    @staticmethod
    def _coeffs(freq_hz: float, sample_rate_hz: float):
        omega = 2.0 * np.pi * freq_hz / sample_rate_hz
        os_, oc = np.sin(omega), np.cos(omega)
        alpha = os_ / (2.0 * _Q_BUTTERWORTH)
        b0 = (1.0 - oc) * 0.5
        b1 = 1.0 - oc
        b2 = (1.0 - oc) * 0.5
        a0 = 1.0 + alpha
        a1 = -2.0 * oc
        a2 = 1.0 - alpha
        b = np.array([b0 / a0, b1 / a0, b2 / a0])
        a = np.array([1.0, a1 / a0, a2 / a0])
        return b, a

    @staticmethod
    def filter_forward_backward(freq_hz: float, sample_rate_hz: float, x: np.ndarray) -> Optional[np.ndarray]:
        """Filter ``x`` (1-D) forward then backward. None if parameters are invalid."""
        if not (sample_rate_hz > 0.0) or not (freq_hz > 0.0) or 2.0 * freq_hz > sample_rate_hz:
            return None
        b, a = Lowpass._coeffs(freq_hz, sample_rate_hz)
        y = lfilter(b, a, x)            # forward (zero initial state == Rust)
        y = lfilter(b, a, y[::-1])[::-1]  # backward over the forward result
        return y

    @staticmethod
    def filter_gyro_forward_backward(freq_hz: float, sample_rate_hz: float, gyro: np.ndarray) -> bool:
        """In-place forward-backward filter of an (N, 3) gyro array. Returns False if skipped."""
        if gyro.size == 0:
            return False
        for c in range(gyro.shape[1]):
            out = Lowpass.filter_forward_backward(freq_hz, sample_rate_hz, gyro[:, c])
            if out is None:
                return False
            gyro[:, c] = out
        return True


# ---------------------------------------------------------------------------
# Angular-velocity time series.
# ---------------------------------------------------------------------------

@dataclass
class GyroSeries:
    """Angular-velocity samples. ``t`` is (N,) ms; ``w`` is (N, 3) deg/s (or rad/s)."""
    t: np.ndarray
    w: np.ndarray

    def __len__(self) -> int:
        return int(self.t.shape[0])


def quaternions_to_angular_velocity(t_ms: np.ndarray, quats: np.ndarray,
                                    swap_xy: bool = False, degrees: bool = True) -> GyroSeries:
    """Per-interval body-frame angular velocity from a quaternion attitude series.

    For consecutive (t0,q0),(t1,q1): dq = q0^-1 * q1, rotation vector = axis*angle of dq,
    omega = rotvec / dt. The last sample copies the previous omega (no forward difference).
    Mirrors Gyroflow's per-frame derivation that produces ``estimated_gyro``.
    """
    n = len(t_ms)
    w = np.zeros((n, 3))
    to_deg = (180.0 / np.pi) if degrees else 1.0
    for i in range(n - 1):
        dt_s = (t_ms[i + 1] - t_ms[i]) / 1000.0
        if dt_s <= 1e-9:
            continue
        dq = quat_normalize(quat_mul(quat_inverse_unit(quats[i]), quats[i + 1]))
        if dq[0] < 0.0:  # shortest arc
            dq = -dq
        v = np.linalg.norm(dq[1:])
        angle = 2.0 * np.arctan2(v, dq[0])
        if v > 1e-12:
            rot = dq[1:] * (angle / v)  # axis*angle
        else:
            rot = np.zeros(3)
        ww = (rot / dt_s) * to_deg
        if swap_xy:
            ww = ww[[1, 0, 2]]
        w[i] = ww
    if n >= 2:
        w[n - 1] = w[n - 2]
    return GyroSeries(np.asarray(t_ms, dtype=float), w)


def resample_angular_velocity(series: GyroSeries, target_t_ms: np.ndarray) -> GyroSeries:
    """Linear interpolation of each component at ``target_t_ms`` (end-clamped)."""
    t = series.t
    w = np.empty((len(target_t_ms), 3))
    for c in range(3):
        w[:, c] = np.interp(target_t_ms, t, series.w[:, c])  # np.interp clamps at the ends
    return GyroSeries(np.asarray(target_t_ms, dtype=float), w)


def max_angle(series: GyroSeries) -> float:
    """Largest absolute gyro component (Gyroflow's get_max_angle / movement gate)."""
    return float(np.max(np.abs(series.w))) if len(series) else 0.0


# ---------------------------------------------------------------------------
# Offset search — port of essential_matrix::find_offsets (single range).
# ---------------------------------------------------------------------------

@dataclass
class OffsetResult:
    found: bool = False
    offset_ms: float = 0.0   # timestamp delay: gyro is sampled at (of_ts - offset_ms)
    cost: float = 0.0
    matched: int = 0


def _build_bintree(gyro: GyroSeries):
    """Sorted microsecond keys + values for nearest-upper lookup (matches the Rust BTreeMap)."""
    keys = (gyro.t * 1000.0).astype(np.int64)  # truncate toward zero (matches C++ (long long) cast)
    order = np.argsort(keys, kind="stable")
    return keys[order], gyro.w[order]


_WEIGHTS = np.array([70.0, 70.0, 100.0])


def _cost(offs: float, of: GyroSeries, keys: np.ndarray, vals: np.ndarray) -> float:
    """Mean weighted squared angular-velocity difference over matched samples (vectorised).

    Nearest-upper IMU lookup on the discrete sample grid (parity with Rust/C++).
    """
    query = ((of.t - offs) * 1000.0).astype(np.int64)  # truncate toward zero (matches C++ cast)
    idx = np.searchsorted(keys, query, side="left")    # first key >= query (nearest upper)
    valid = idx < keys.shape[0]
    matches = int(np.count_nonzero(valid))
    if of.t.shape[0] == 0 or matches <= of.t.shape[0] // 2:
        return float("inf")
    g = vals[idx[valid]]
    o = of.w[valid]
    d = g - o
    s = float(np.sum((d * d) * _WEIGHTS))
    return s / matches


def _cost_interp(offs: float, of: GyroSeries, gt_ms: np.ndarray, gw: np.ndarray) -> float:
    """Same cost, but the IMU value at the exact (continuous) query time is obtained by
    linear interpolation between the bracketing samples instead of snapping to the next
    sample. Removes the one-IMU-sample quantization bias (breaks bit-for-bit Rust parity).
    """
    query = of.t - offs  # exact ms, no grid snap
    valid = query <= gt_ms[-1]  # mirror nearest-upper: a sample exists iff query <= last key
    matches = int(np.count_nonzero(valid))
    if of.t.shape[0] == 0 or matches <= of.t.shape[0] // 2:
        return float("inf")
    q = query[valid]
    g = np.empty((q.shape[0], 3))
    for c in range(3):
        g[:, c] = np.interp(q, gt_ms, gw[:, c])  # end-clamped linear interpolation
    d = g - of.w[valid]
    s = float(np.sum((d * d) * _WEIGHTS))
    return s / matches


def find_offset(of: GyroSeries, gyro: GyroSeries, initial_offset_ms: float, search_size_ms: float,
                of_sample_rate_hz: float, gyro_sample_rate_hz: float, lpf_hz: float = 20.0,
                interp: bool = False, parabolic: bool = False) -> OffsetResult:
    """Find the timestamp delay between ``of`` (video-side) and ``gyro`` (IMU-side).

    20 Hz forward-backward low-pass both signals, sweep [initial +/- search] at 1 ms, refine
    +/-2 ms at 0.01 ms. Cost weights x,y by 70 and z by 100, averaged over matched samples.
    Accept only when the optimum is within 90% of the search size (as in Rust).

    interp=False (default): nearest-upper IMU lookup, bit-for-bit faithful to the Rust/C++ port
    (accuracy ceiling ~= one IMU sample interval). interp=True: linear-interpolated IMU lookup,
    which removes that quantization bias at the cost of exact parity.

    parabolic=True: after the 0.01 ms refine grid, fit a parabola to the three cost samples
    around the minimum and move to its analytic vertex, removing the residual refine-grid step.
    Only meaningful with a continuous cost curve, so it implies/pairs with ``interp``.
    """
    result = OffsetResult()
    if len(of) == 0 or len(gyro) == 0:
        return result

    first_of, last_of = float(of.t[0]), float(of.t[-1])

    # Window the gyro to the relevant span (filter in shifted space, keep original timestamps).
    shifted = gyro.t + initial_offset_ms
    mask = (shifted >= first_of - search_size_ms) & (shifted <= last_of + search_size_ms)
    if not np.any(mask):
        return result
    gyro_item = GyroSeries(gyro.t[mask].copy(), gyro.w[mask].copy())

    # Work on copies so the caller's data is untouched.
    of_f = GyroSeries(of.t.copy(), of.w.copy())
    Lowpass.filter_gyro_forward_backward(lpf_hz, of_sample_rate_hz, of_f.w)
    Lowpass.filter_gyro_forward_backward(lpf_hz, gyro_sample_rate_hz, gyro_item.w)

    keys, vals = _build_bintree(gyro_item)
    # Interpolated path needs sorted float timestamps (ms) of the windowed gyro.
    gt_ms, gw = (keys.astype(np.float64) / 1000.0, vals) if interp else (None, None)
    cost = (lambda o: _cost_interp(o, of_f, gt_ms, gw)) if interp else (lambda o: _cost(o, of_f, keys, vals))

    # Coarse sweep at 1 ms.
    steps = int(search_size_ms) * 2
    best_off, best_cost = 0.0, float("inf")
    for i in range(steps):
        offs = initial_offset_ms - search_size_ms + float(i)
        c = cost(offs)
        if c < best_cost:
            best_cost, best_off = c, offs
    if not np.isfinite(best_cost):
        return result

    # Refine to 0.01 ms around the (fixed) coarse optimum. Centre is fixed while searching
    # (separate accumulator, matching the Rust/C++ loop). The faithful default sweeps the
    # original one-sided window [center-2, center] (200 steps); the high-precision interp path
    # sweeps the full symmetric [center-2, +2] so refine can also climb above the coarse pick.
    center = best_off
    refine_size = 2.0
    step = 0.01
    refine_steps = (int(2.0 * refine_size / step) + 1) if interp else int(refine_size * 100.0)
    for i in range(refine_steps):
        offs = center + (-refine_size + i * step)
        c = cost(offs)
        if c < best_cost:
            best_cost, best_off = c, offs

    # Optional parabolic (sub-grid) vertex of the cost curve around the refine minimum.
    # Fit a parabola through (best_off-step, best_off, best_off+step); its vertex offset is
    # delta = 0.5*(c_minus - c_plus)/(c_minus - 2*c0 + c_plus) in units of `step`. Guard on a
    # convex denominator and |delta| <= 1 (the bracket really contains a minimum).
    if parabolic:
        c_minus = cost(best_off - step)
        c_plus = cost(best_off + step)
        denom = c_minus - 2.0 * best_cost + c_plus
        if np.isfinite(c_minus) and np.isfinite(c_plus) and denom > 0.0:
            delta = 0.5 * (c_minus - c_plus) / denom
            if -1.0 <= delta <= 1.0:
                best_off = best_off + delta * step
                best_cost = cost(best_off)

    # Matches at the optimum (for reporting).
    query = ((of_f.t - best_off) * 1000.0).astype(np.int64)
    idx = np.searchsorted(keys, query, side="left")
    matched = int(np.count_nonzero(idx < keys.shape[0]))

    if abs(best_off - initial_offset_ms) < search_size_ms * 0.9 and np.isfinite(best_cost):
        result.found = True
        result.offset_ms = best_off
        result.cost = best_cost
        result.matched = matched
    return result


# ---------------------------------------------------------------------------
# DJI CSV loading.
# ---------------------------------------------------------------------------

def load_quaternions(path: str):
    """Load a DJI quaternion CSV (dji_quaternions_full.csv or dji_camera_data.csv).

    Returns (t_ms (N,), quats (N,4) w,x,y,z), sorted by timestamp.
    """
    with open(path, "r") as f:
        header = f.readline().strip().split(",")

    def idx(name):
        return header.index(name) if name in header else -1

    ts = idx("quat_timestamp_ms")
    if ts < 0:
        ts = idx("timestamp_ms")
    qw, qx, qy, qz = idx("quat_w"), idx("quat_x"), idx("quat_y"), idx("quat_z")
    if qw < 0:
        qw, qx, qy, qz = idx("org_quat_w"), idx("org_quat_x"), idx("org_quat_y"), idx("org_quat_z")
    if min(ts, qw, qx, qy, qz) < 0:
        raise ValueError("unrecognised CSV header (need timestamp + quaternion columns)")

    cols = (ts, qw, qx, qy, qz)
    data = np.genfromtxt(path, delimiter=",", skip_header=1, usecols=cols)
    if data.ndim == 1:
        data = data[None, :]
    t_ms = data[:, 0]
    quats = data[:, 1:5]
    quats = quats / np.linalg.norm(quats, axis=1, keepdims=True)
    order = np.argsort(t_ms, kind="stable")
    return t_ms[order], quats[order]


# ---------------------------------------------------------------------------
# Real angular-velocity loading (for syncing two independently measured signals).
# `find_offset`'s cost weights and the 3 deg/s motion gate are tuned for deg/s, so every
# loader below returns a GyroSeries with t in ms and w in deg/s.
# ---------------------------------------------------------------------------

_RAD2DEG = 180.0 / np.pi


def orient_vec(w: np.ndarray, orientation: str) -> np.ndarray:
    """Re-map/flip axes of an (N,3) array per a 3-char Gyroflow orientation string.

    Upper-case keeps an axis, lower-case negates it: e.g. ``"xzY"`` -> (-x, -z, +y).
    Matches ``tools/gcsv_simple_gyro_compare.py``'s convention so GCSV ``orientation`` headers
    behave identically here.
    """
    if len(orientation) != 3:
        raise ValueError(f"orientation must be 3 characters, got {orientation!r}")
    src = {"X": w[:, 0], "x": -w[:, 0], "Y": w[:, 1], "y": -w[:, 1], "Z": w[:, 2], "z": -w[:, 2]}
    return np.stack([src[c] for c in orientation], axis=1)


def estimate_sample_rate_hz(series: "GyroSeries") -> float:
    """Robust sample rate from the median timestamp spacing (Hz). 0 if undefined."""
    if len(series) < 2:
        return 0.0
    dt_ms = float(np.median(np.diff(series.t)))
    return 1000.0 / dt_ms if dt_ms > 0.0 else 0.0


def load_gcsv(path: str, orientation: Optional[str] = None) -> "GyroSeries":
    """Load a Gyroflow GCSV IMU log into a deg/s :class:`GyroSeries` (t in ms).

    Honours the ``tscale`` (timestamp scale, default 0.001 -> seconds) and ``gscale`` (gyro scale
    to rad/s, default 1.0) header keys, matching telemetry-parser's GCSV path. Timestamps are
    re-based to start at 0. ``orientation`` (e.g. ``"xzY"``) overrides any header orientation; pass
    ``None`` to leave axes as stored.
    """
    header: dict = {}
    data_header = None
    rows = []
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        import csv as _csv
        for raw in _csv.reader(f):
            row = [c.strip() for c in raw]
            if not row or all(not c for c in row):
                continue
            if data_header is None:
                if row[0] in ("t", "time"):
                    data_header = row
                    continue
                if len(row) >= 2:
                    header[row[0]] = row[1]
                continue
            rows.append(row)
    if data_header is None:
        raise ValueError("not a GCSV file (no data header starting with 't' or 'time')")

    time_scale = float(header.get("tscale", "0.001"))      # raw -> seconds
    gscale = float(header.get("gscale", "1.0"))            # raw -> rad/s
    data = np.array([[float(x) for x in r[:4]] for r in rows if len(r) >= 4], dtype=float)
    if data.size == 0:
        raise ValueError("GCSV has no t,gx,gy,gz samples")
    t_ms = (data[:, 0] * time_scale) * 1000.0
    t_ms = t_ms - t_ms[0]
    w = data[:, 1:4] * gscale * _RAD2DEG                   # rad/s -> deg/s
    orient = orientation if orientation is not None else header.get("orientation")
    if orient:
        w = orient_vec(w, orient)
    order = np.argsort(t_ms, kind="stable")
    return GyroSeries(t_ms[order], w[order])


_TS_NAMES_MS = ("timestamp_ms", "time_ms", "t_ms", "ts_ms")
_TS_NAMES_S = ("timestamp_s", "time_s", "t_s", "ts_s", "seconds")
_TS_NAMES_BARE = ("timestamp", "time", "t", "ts")
_AXIS_SETS = (
    ("wx_deg_s", "wy_deg_s", "wz_deg_s"), ("wx_rad_s", "wy_rad_s", "wz_rad_s"),
    ("wx", "wy", "wz"), ("gx", "gy", "gz"), ("gyro_x", "gyro_y", "gyro_z"),
    ("omega_x", "omega_y", "omega_z"), ("x", "y", "z"),
)


def load_angular_velocity_csv(path: str, units: str = "deg",
                              orientation: Optional[str] = None) -> "GyroSeries":
    """Load a generic angular-velocity CSV into a deg/s :class:`GyroSeries` (t in ms).

    Auto-detects a timestamp column (``timestamp_ms`` / ``t`` / ``timestamp_s`` / ...) and a
    3-axis set (``wx,wy,wz`` / ``gx,gy,gz`` / ``wx_deg_s,...`` / ``x,y,z`` / ...). Reads the
    ``omega`` sub-command's own output directly. Units are inferred from a ``_deg``/``_rad`` axis
    suffix when present, else from ``units`` ('deg' or 'rad'). Re-based to start at 0.
    """
    with open(path, "r") as f:
        header = [h.strip() for h in f.readline().strip().split(",")]
    lower = [h.lower() for h in header]

    def find(name):
        return lower.index(name) if name in lower else -1

    ts = next((find(n) for n in (_TS_NAMES_MS + _TS_NAMES_S + _TS_NAMES_BARE) if find(n) >= 0), -1)
    if ts < 0:
        raise ValueError(f"no timestamp column in {header}")
    ts_name = lower[ts]
    if ts_name in _TS_NAMES_S:
        ts_scale_ms = 1000.0
    elif ts_name in _TS_NAMES_MS:
        ts_scale_ms = 1.0
    else:  # bare name: assume ms (the omega dump and most logs are ms)
        ts_scale_ms = 1.0

    axes = next((cols for cols in _AXIS_SETS if all(c in lower for c in cols)), None)
    if axes is None:
        raise ValueError(f"no recognised angular-velocity columns in {header}")
    ax = [lower.index(c) for c in axes]

    if "deg" in axes[0]:
        to_deg = 1.0
    elif "rad" in axes[0]:
        to_deg = _RAD2DEG
    else:
        to_deg = 1.0 if units == "deg" else _RAD2DEG

    data = np.genfromtxt(path, delimiter=",", skip_header=1, usecols=(ts, *ax))
    if data.ndim == 1:
        data = data[None, :]
    t_ms = (data[:, 0] * ts_scale_ms)
    t_ms = t_ms - t_ms[0]
    w = data[:, 1:4] * to_deg
    if orientation:
        w = orient_vec(w, orientation)
    order = np.argsort(t_ms, kind="stable")
    return GyroSeries(t_ms[order], w[order])


def load_motion(path: str, units: str = "deg", orientation: Optional[str] = None) -> "GyroSeries":
    """Load a real angular-velocity signal, auto-detecting GCSV vs generic CSV.

    GCSV is recognised by a metadata block followed by a ``t``/``time`` data header; anything else
    is treated as a generic angular-velocity CSV. Returns deg/s, t in ms, re-based to 0.
    """
    with open(path, "r", encoding="utf-8-sig") as f:
        head = [f.readline() for _ in range(40)]
    # GCSV signature: a `t`/`time` data header preceded by a metadata block (so not on line 0).
    # A generic CSV puts its column header on line 0, even if that header is bare `t,...`.
    first_data_hdr = next((i for i, line in enumerate(head)
                           if line and line.strip().split(",")[0].strip() in ("t", "time")), -1)
    is_gcsv = first_data_hdr > 0
    return (load_gcsv(path, orientation) if is_gcsv
            else load_angular_velocity_csv(path, units, orientation))
