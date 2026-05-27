from __future__ import annotations

from dataclasses import dataclass

import gtsam
import numpy as np

from .trajectory import G_WORLD


def rot3_from_matrix(R: np.ndarray) -> gtsam.Rot3:
    return gtsam.Rot3(R)


def navstate_from_arrays(R: np.ndarray, p: np.ndarray, v: np.ndarray) -> gtsam.NavState:
    return gtsam.NavState(gtsam.Pose3(rot3_from_matrix(R), gtsam.Point3(*p)), v)


def arrays_from_navstate(state: gtsam.NavState) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    R = state.attitude().matrix()
    p = np.array(state.position(), dtype=float)
    v = np.array(state.velocity(), dtype=float)
    return R, p, v


@dataclass
class FilterRecord:
    R: np.ndarray
    p: np.ndarray
    v: np.ndarray
    bg: np.ndarray
    ba: np.ndarray
    P: np.ndarray


class SE23IEKF:
    """Small GTSAM-backed EKF over NavState plus IMU bias.

    `NavState` is GTSAM's SE_2(3)-style state `(R, p, v)` with tangent ordering
    `[rotation, position, velocity]` for `localCoordinates`, while `retract` expects
    the same 9-vector ordering. The covariance here is stored as
    `[rotation, position, velocity, gyro_bias, accel_bias]`.
    """

    def __init__(
        self,
        state: gtsam.NavState,
        bias: gtsam.imuBias.ConstantBias,
        P: np.ndarray,
        gyro_sigma: float,
        accel_sigma: float,
        gyro_bias_rw: float,
        accel_bias_rw: float,
        gps_sigma: float,
    ) -> None:
        self.state = state
        self.bias = bias
        self.P = P.copy()
        self.gps_R = np.eye(3) * gps_sigma**2
        self.Q_bias = np.diag([gyro_bias_rw**2] * 3 + [accel_bias_rw**2] * 3)

        params = gtsam.PreintegrationParams.MakeSharedU(float(np.linalg.norm(G_WORLD)))
        params.setAccelerometerCovariance(np.eye(3) * accel_sigma**2)
        params.setGyroscopeCovariance(np.eye(3) * gyro_sigma**2)
        params.setIntegrationCovariance(np.eye(3) * 1e-8)
        self.params = params

    def predict(self, accel: np.ndarray, gyro: np.ndarray, dt: float) -> None:
        old = self.state
        pim = gtsam.PreintegratedImuMeasurements(self.params, self.bias)
        pim.integrateMeasurement(accel, gyro, dt)
        new = pim.predict(old, self.bias)
        self.state = new

        # Numerical discrete-time Jacobian in the NavState tangent space. This is
        # slower than a hand-derived invariant F but keeps the example compact and
        # tied to GTSAM's exact retract/local-coordinate conventions.
        F = np.eye(15)
        eps = 1e-5
        for j in range(9):
            dx = np.zeros(9)
            dx[j] = eps
            xp = old.retract(dx)
            yp = pim.predict(xp, self.bias)
            F[:9, j] = np.asarray(new.localCoordinates(yp)) / eps

        for j in range(3):
            db = np.zeros(6)
            db[j] = eps
            bp = gtsam.imuBias.ConstantBias(
                np.asarray(self.bias.accelerometer(), dtype=float),
                np.asarray(self.bias.gyroscope(), dtype=float) + db[:3],
            )
            yp = pim.predict(old, bp)
            F[:9, 9 + j] = np.asarray(new.localCoordinates(yp)) / eps

        for j in range(3):
            db = np.zeros(6)
            db[3 + j] = eps
            bp = gtsam.imuBias.ConstantBias(
                np.asarray(self.bias.accelerometer(), dtype=float) + db[3:],
                np.asarray(self.bias.gyroscope(), dtype=float),
            )
            yp = pim.predict(old, bp)
            F[:9, 12 + j] = np.asarray(new.localCoordinates(yp)) / eps

        Q = np.zeros((15, 15))
        Q[:9, :9] = np.asarray(pim.preintMeasCov())
        Q[9:, 9:] = self.Q_bias * dt
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def update_gps(self, z: np.ndarray) -> None:
        H = np.zeros((3, 15))
        p0 = np.array(self.state.position(), dtype=float)
        eps = 1e-6
        for j in range(9):
            dx = np.zeros(9)
            dx[j] = eps
            xp = self.state.retract(dx)
            H[:, j] = (np.array(xp.position(), dtype=float) - p0) / eps

        innov = z - p0
        S = H @ self.P @ H.T + self.gps_R
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ innov

        self.state = self.state.retract(dx[:9])
        ba = np.asarray(self.bias.accelerometer(), dtype=float) + dx[12:15]
        bg = np.asarray(self.bias.gyroscope(), dtype=float) + dx[9:12]
        self.bias = gtsam.imuBias.ConstantBias(ba, bg)

        I = np.eye(15)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ self.gps_R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def record(self) -> FilterRecord:
        R, p, v = arrays_from_navstate(self.state)
        return FilterRecord(
            R=R,
            p=p,
            v=v,
            bg=np.asarray(self.bias.gyroscope(), dtype=float),
            ba=np.asarray(self.bias.accelerometer(), dtype=float),
            P=self.P.copy(),
        )


def propagate_dead_reckoning(
    state: gtsam.NavState,
    bias: gtsam.imuBias.ConstantBias,
    accel: np.ndarray,
    gyro: np.ndarray,
    dt: float,
    gyro_sigma: float,
    accel_sigma: float,
) -> gtsam.NavState:
    params = gtsam.PreintegrationParams.MakeSharedU(float(np.linalg.norm(G_WORLD)))
    params.setAccelerometerCovariance(np.eye(3) * accel_sigma**2)
    params.setGyroscopeCovariance(np.eye(3) * gyro_sigma**2)
    params.setIntegrationCovariance(np.eye(3) * 1e-8)
    pim = gtsam.PreintegratedImuMeasurements(params, bias)
    pim.integrateMeasurement(accel, gyro, dt)
    return pim.predict(state, bias)
