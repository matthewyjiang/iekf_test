from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


G_WORLD = np.array([0.0, 0.0, -9.81])


@dataclass(frozen=True)
class TruthState:
    t: float
    R: np.ndarray
    p: np.ndarray
    v: np.ndarray
    a: np.ndarray
    omega_body: np.ndarray


@dataclass(frozen=True)
class SimData:
    times: np.ndarray
    R: np.ndarray
    p: np.ndarray
    v: np.ndarray
    a: np.ndarray
    omega_body: np.ndarray
    accel_meas: np.ndarray
    gyro_meas: np.ndarray
    gyro_bias: np.ndarray
    accel_bias: np.ndarray
    gps_times: np.ndarray
    gps_indices: np.ndarray
    gps: np.ndarray


def ground_truth(t: float, radius: float = 10.0, w: float = 0.30, z_amp: float = 2.0, z_w: float = 0.50) -> TruthState:
    p = np.array([radius * np.cos(w * t), radius * np.sin(w * t), z_amp * np.sin(z_w * t)])
    v = np.array([-radius * w * np.sin(w * t), radius * w * np.cos(w * t), z_amp * z_w * np.cos(z_w * t)])
    a = np.array([-radius * w * w * np.cos(w * t), -radius * w * w * np.sin(w * t), -z_amp * z_w * z_w * np.sin(z_w * t)])

    yaw = np.arctan2(v[1], v[0])
    pitch = np.arctan2(-v[2], np.linalg.norm(v[:2]))
    roll = np.arctan2((radius * w * w), 9.81)
    R = Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()

    dt = 1e-4
    yaw2 = np.arctan2(
        radius * w * np.cos(w * (t + dt)),
        -radius * w * np.sin(w * (t + dt)),
    )
    pitch2 = np.arctan2(-z_amp * z_w * np.cos(z_w * (t + dt)), radius * w)
    R2 = Rotation.from_euler("ZYX", [yaw2, pitch2, roll]).as_matrix()
    dR = R.T @ R2
    omega_body = Rotation.from_matrix(dR).as_rotvec() / dt

    return TruthState(t=t, R=R, p=p, v=v, a=a, omega_body=omega_body)


def simulate(
    duration: float = 30.0,
    dt_imu: float = 1.0 / 200.0,
    dt_gps: float = 1.0 / 10.0,
    seed: int = 4,
    gyro_sigma: float = 0.004,
    accel_sigma: float = 0.06,
    gps_sigma: float = 0.45,
) -> SimData:
    rng = np.random.default_rng(seed)
    times = np.arange(0.0, duration + 0.5 * dt_imu, dt_imu)
    n = len(times)

    R = np.zeros((n, 3, 3))
    p = np.zeros((n, 3))
    v = np.zeros((n, 3))
    a = np.zeros((n, 3))
    omega_body = np.zeros((n, 3))
    accel_true = np.zeros((n, 3))

    for i, t in enumerate(times):
        x = ground_truth(float(t))
        R[i] = x.R
        p[i] = x.p
        v[i] = x.v
        a[i] = x.a
        omega_body[i] = x.omega_body
        accel_true[i] = x.R.T @ (x.a - G_WORLD)

    gyro_bias0 = np.array([0.015, -0.010, 0.008])
    accel_bias0 = np.array([0.08, -0.05, 0.12])
    gyro_bias_rw = 0.00008
    accel_bias_rw = 0.0015

    gyro_bias = np.zeros((n, 3))
    accel_bias = np.zeros((n, 3))
    gyro_bias[0] = gyro_bias0
    accel_bias[0] = accel_bias0
    for i in range(1, n):
        gyro_bias[i] = gyro_bias[i - 1] + rng.normal(0.0, gyro_bias_rw * np.sqrt(dt_imu), 3)
        accel_bias[i] = accel_bias[i - 1] + rng.normal(0.0, accel_bias_rw * np.sqrt(dt_imu), 3)

    gyro_meas = omega_body + gyro_bias + rng.normal(0.0, gyro_sigma, (n, 3))
    accel_meas = accel_true + accel_bias + rng.normal(0.0, accel_sigma, (n, 3))

    gps_step = max(1, int(round(dt_gps / dt_imu)))
    gps_indices = np.arange(0, n, gps_step, dtype=int)
    gps_times = times[gps_indices]
    gps = p[gps_indices] + rng.normal(0.0, gps_sigma, (len(gps_indices), 3))

    return SimData(
        times=times,
        R=R,
        p=p,
        v=v,
        a=a,
        omega_body=omega_body,
        accel_meas=accel_meas,
        gyro_meas=gyro_meas,
        gyro_bias=gyro_bias,
        accel_bias=accel_bias,
        gps_times=gps_times,
        gps_indices=gps_indices,
        gps=gps,
    )
