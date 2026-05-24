from __future__ import annotations

import math

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.where(norm > 0.0, norm, 1.0)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )


def quat_inv(q: np.ndarray) -> np.ndarray:
    ret = np.array(q, dtype=np.float64, copy=True)
    ret[..., 1:] *= -1.0
    norm2 = np.sum(q * q, axis=-1, keepdims=True)
    return ret / np.where(norm2 > 0.0, norm2, 1.0)


def quat_from_scaled_axis(v: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = v / angle
    half = 0.5 * angle
    return np.array([math.cos(half), *(axis * math.sin(half))], dtype=np.float64)


def quat_slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    a = quat_normalize(a)
    b = quat_normalize(b)
    cos_theta = float(np.dot(a, b))
    if cos_theta < 0.0:
        b = -b
        cos_theta = -cos_theta

    t = max(0.0, min(1.0, t))
    if cos_theta > 0.9995:
        return quat_normalize(a + t * (b - a))

    theta = math.acos(max(-1.0, min(1.0, cos_theta)))
    sin_theta = math.sin(theta)
    wa = math.sin((1.0 - t) * theta) / sin_theta
    wb = math.sin(t * theta) / sin_theta
    return quat_normalize(a * wa + b * wb)


def quat_log_to_omega(q_delta: np.ndarray, dt_s: float) -> np.ndarray:
    q_delta = quat_normalize(q_delta)
    if q_delta[0] < 0.0:
        q_delta = -q_delta

    v = q_delta[1:]
    v_norm = float(np.linalg.norm(v))
    if v_norm < 1e-12 or dt_s <= 0.0:
        return np.zeros(3, dtype=np.float64)

    angle = 2.0 * math.atan2(v_norm, float(q_delta[0]))
    return (v / v_norm) * (angle / dt_s)


def orient_vec(values: np.ndarray, orientation: str) -> np.ndarray:
    if len(orientation) != 3:
        raise ValueError(f"orientation must have 3 characters, got {orientation!r}")

    mapping = {
        "X": values[:, 0],
        "x": -values[:, 0],
        "Y": values[:, 1],
        "y": -values[:, 1],
        "Z": values[:, 2],
        "z": -values[:, 2],
    }
    return np.stack([mapping[c] for c in orientation], axis=1)


def integrate_simple_gyro(timestamps_s: np.ndarray, gyro_rad_s: np.ndarray) -> np.ndarray:
    if len(timestamps_s) == 0:
        return np.empty((0, 4), dtype=np.float64)

    # Matches UnitQuaternion::from_euler_angles(FRAC_PI_2, 0, 0).
    orientation = np.array(
        [math.cos(math.pi / 4.0), math.sin(math.pi / 4.0), 0.0, 0.0],
        dtype=np.float64,
    )

    sample_time = (timestamps_s[-1] - timestamps_s[0]) / max(1, len(timestamps_s) - 1)
    prev_time = timestamps_s[0] - sample_time
    quats = np.empty((len(timestamps_s), 4), dtype=np.float64)

    for i, (timestamp, gyro) in enumerate(zip(timestamps_s, gyro_rad_s)):
        # Gyroflow SimpleGyroIntegrator applies this axis shuffle before exp-map integration.
        omega = np.array([-gyro[1], gyro[0], gyro[2]], dtype=np.float64)
        dt = float(timestamp - prev_time)
        delta_q = quat_from_scaled_axis(omega * dt)
        orientation = quat_normalize(quat_mul(orientation, delta_q))
        quats[i] = orientation
        prev_time = timestamp

    return quats


def smooth_quaternions(timestamps_s: np.ndarray, quats: np.ndarray, smooth_ms: float) -> np.ndarray:
    if len(quats) <= 1 or smooth_ms <= 0.0:
        return np.array(quats, copy=True)

    smooth_s = smooth_ms / 1000.0
    out = np.array(quats, copy=True)

    for i in range(1, len(out)):
        dt = max(0.0, timestamps_s[i] - timestamps_s[i - 1])
        alpha = 1.0 - math.exp(-dt / smooth_s)
        out[i] = quat_slerp(out[i - 1], quats[i], alpha)

    for i in range(len(out) - 2, -1, -1):
        dt = max(0.0, timestamps_s[i + 1] - timestamps_s[i])
        alpha = 1.0 - math.exp(-dt / smooth_s)
        out[i] = quat_slerp(out[i + 1], out[i], alpha)

    return out


def angular_velocity_from_quats(timestamps_s: np.ndarray, quats: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if len(quats) < 2:
        return np.empty(0, dtype=np.float64), np.empty((0, 3), dtype=np.float64)

    out_t = np.empty(len(quats) - 1, dtype=np.float64)
    out_omega = np.empty((len(quats) - 1, 3), dtype=np.float64)

    for i in range(1, len(quats)):
        dt = float(timestamps_s[i] - timestamps_s[i - 1])
        q_delta = quat_mul(quat_inv(quats[i - 1]), quats[i])
        out_omega[i - 1] = quat_log_to_omega(q_delta, dt)
        out_t[i - 1] = 0.5 * (timestamps_s[i - 1] + timestamps_s[i])

    return out_t, out_omega


def angular_velocity_from_simple_gyro_quats(timestamps_s: np.ndarray, quats: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    timestamps, omega_integrator_axes = angular_velocity_from_quats(timestamps_s, quats)
    gyro_axes = np.empty_like(omega_integrator_axes)
    # Invert Gyroflow's SimpleGyro axis shuffle:
    # omega_integrator = [-raw_y, raw_x, raw_z]
    gyro_axes[:, 0] = omega_integrator_axes[:, 1]
    gyro_axes[:, 1] = -omega_integrator_axes[:, 0]
    gyro_axes[:, 2] = omega_integrator_axes[:, 2]
    return timestamps, gyro_axes


def quat_to_euler_xyz_deg(quats: np.ndarray) -> np.ndarray:
    q = quat_normalize(quats)
    w, x, y, z = np.moveaxis(q, -1, 0)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1.0, np.sign(sinp) * (math.pi / 2.0), np.arcsin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.rad2deg(np.stack([roll, pitch, yaw], axis=1))

