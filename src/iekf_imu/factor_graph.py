"""Factor-graph SLAM back-end using GTSAM iSAM2.

Architecture (matches ARCHITECTURE.md):
  - CombinedImuFactor for IMU preintegration between keyframes.
    Uses PreintegratedCombinedMeasurements (NOT PreintegratedImuMeasurements
    which is used by SE23IEKF in iekf.py — different class, different params).
  - BetweenFactor<Pose3> for camera relative-pose constraints.
  - iSAM2 incremental solver.

GTSAM symbol convention:
  X(k) = Pose3  at keyframe k
  V(k) = Vector3 velocity at keyframe k
  B(k) = imuBias.ConstantBias at keyframe k

Tangent-space ordering:
  Pose3:    [rotation(3), translation(3)]   — rot FIRST
  NavState: [rotation(3), position(3), velocity(3)]  — same rot order
  imuBias:  [gyro(3), accel(3)]             — note: opposite of ConstantBias constructor
                                              which takes (accel, gyro)

Double-counting rule (ARCHITECTURE.md):
  Raw IMU feeds IEKF AND this graph. Raw camera feeds graph only.
  IEKF state is used only as initial-guess values for new graph variables,
  never inserted as a factor itself.
"""
from __future__ import annotations

import numpy as np
import gtsam
from gtsam import symbol_shorthand as S

from .trajectory import G_WORLD


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pose3_from_Rp(R: np.ndarray, p: np.ndarray) -> gtsam.Pose3:
    return gtsam.Pose3(gtsam.Rot3(R), p)


def _make_combined_params(
    accel_sigma: float,
    gyro_sigma: float,
    accel_bias_rw: float,
    gyro_bias_rw: float,
    integration_sigma: float = 1e-4,
) -> gtsam.PreintegrationCombinedParams:
    """Build PreintegrationCombinedParams for CombinedImuFactor.

    Uses U-convention: gravity magnitude passed to MakeSharedU, which places
    gravity along -Z in world frame, matching G_WORLD = [0,0,-9.81].

    setBiasAccCovariance / setBiasOmegaCovariance set the bias random-walk
    process noise baked into CombinedImuFactor — no separate BetweenFactor
    on bias needed.
    """
    g_mag = float(np.linalg.norm(G_WORLD))
    params = gtsam.PreintegrationCombinedParams.MakeSharedU(g_mag)
    params.setAccelerometerCovariance(np.eye(3) * accel_sigma ** 2)
    params.setGyroscopeCovariance(np.eye(3) * gyro_sigma ** 2)
    params.setIntegrationCovariance(np.eye(3) * integration_sigma ** 2)
    # Bias random-walk covariance (per second, discrete approximation)
    params.setBiasAccCovariance(np.eye(3) * accel_bias_rw ** 2)
    params.setBiasOmegaCovariance(np.eye(3) * gyro_bias_rw ** 2)
    params.setBiasAccOmegaInit(np.zeros((6, 6)))
    return params


# ---------------------------------------------------------------------------
# GraphBackend
# ---------------------------------------------------------------------------

class GraphBackend:
    """Incremental factor-graph back-end using iSAM2.

    Usage pattern (called from graph_main.py):
        backend = GraphBackend(...)
        for each IMU sample:
            backend.accumulate_imu(accel, gyro, dt)
            if keyframe:
                backend.add_keyframe(iekf_R, iekf_p, iekf_v)
        for each camera measurement:
            backend.add_camera_between(kf_i, kf_j, T_ij, noise_model)
        # camera between-factors added lazily after both keyframes exist
        backend.optimize()
        R_kf, p_kf, v_kf = backend.get_estimates()
    """

    def __init__(
        self,
        init_R: np.ndarray,
        init_p: np.ndarray,
        init_v: np.ndarray,
        init_bias: gtsam.imuBias.ConstantBias,
        accel_sigma: float,
        gyro_sigma: float,
        accel_bias_rw: float,
        gyro_bias_rw: float,
        # Prior noise sigmas
        prior_rot_sigma: float = 0.1,    # rad
        prior_pos_sigma: float = 0.5,    # m
        prior_vel_sigma: float = 0.3,    # m/s
        prior_bias_acc_sigma: float = 0.3,
        prior_bias_gyro_sigma: float = 0.1,
    ) -> None:
        self._params = _make_combined_params(
            accel_sigma, gyro_sigma, accel_bias_rw, gyro_bias_rw
        )
        self._isam = gtsam.ISAM2()
        self._kf_idx = 0  # next keyframe index

        # PreintegratedCombinedMeasurements accumulates IMU between keyframes
        self._pim = gtsam.PreintegratedCombinedMeasurements(self._params, init_bias)

        # Pending factors / values to push on next optimize()
        self._pending_graph = gtsam.NonlinearFactorGraph()
        self._pending_values = gtsam.Values()

        # Camera between-factors queued before optimization
        self._camera_queue: list[tuple[int, int, gtsam.Pose3, gtsam.noiseModel.Base]] = []

        # Insert prior factors + initial values for keyframe 0
        prior_noise_pose = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([prior_rot_sigma] * 3 + [prior_pos_sigma] * 3)
        )
        prior_noise_vel = gtsam.noiseModel.Isotropic.Sigma(3, prior_vel_sigma)
        prior_noise_bias = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(
                [prior_bias_gyro_sigma] * 3 + [prior_bias_acc_sigma] * 3
            )
        )

        pose0 = _pose3_from_Rp(init_R, init_p)
        vel0 = init_v.copy()
        bias0 = init_bias

        self._pending_graph.push_back(
            gtsam.PriorFactorPose3(S.X(0), pose0, prior_noise_pose)
        )
        self._pending_graph.push_back(
            gtsam.PriorFactorVector(S.V(0), vel0, prior_noise_vel)
        )
        self._pending_graph.push_back(
            gtsam.PriorFactorConstantBias(S.B(0), bias0, prior_noise_bias)
        )

        self._pending_values.insert(S.X(0), pose0)
        self._pending_values.insert(S.V(0), vel0)
        self._pending_values.insert(S.B(0), bias0)

        # Flush priors immediately so iSAM2 has the root
        self._isam.update(self._pending_graph, self._pending_values)
        self._pending_graph = gtsam.NonlinearFactorGraph()
        self._pending_values = gtsam.Values()

        # Keep track of keyframe poses for external use
        self._kf_R: list[np.ndarray] = [init_R.copy()]
        self._kf_p: list[np.ndarray] = [init_p.copy()]
        self._kf_v: list[np.ndarray] = [init_v.copy()]

    # ------------------------------------------------------------------
    # IMU accumulation
    # ------------------------------------------------------------------

    def accumulate_imu(
        self, accel: np.ndarray, gyro: np.ndarray, dt: float
    ) -> None:
        """Integrate one IMU sample into the pending preintegrated measurement."""
        self._pim.integrateMeasurement(accel, gyro, dt)

    # ------------------------------------------------------------------
    # Keyframe management
    # ------------------------------------------------------------------

    def add_keyframe(
        self,
        R_guess: np.ndarray,
        p_guess: np.ndarray,
        v_guess: np.ndarray,
    ) -> int:
        """Add a new keyframe with initial-guess values from the IEKF.

        Inserts a CombinedImuFactor from the previous keyframe to this one,
        then resets preintegration for the next interval.

        Returns the new keyframe index.
        """
        prev_k = self._kf_idx
        new_k = prev_k + 1
        self._kf_idx = new_k

        pose_guess = _pose3_from_Rp(R_guess, p_guess)
        vel_guess = v_guess.copy()

        # Get current best estimate of previous bias from iSAM2
        try:
            est = self._isam.calculateEstimate()
            prev_bias = est.atConstantBias(S.B(prev_k))
        except Exception:
            prev_bias = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))

        # Initial guess for new bias = previous bias (bias walk is slow)
        new_bias_guess = prev_bias

        # Add new variable values
        self._pending_values.insert(S.X(new_k), pose_guess)
        self._pending_values.insert(S.V(new_k), vel_guess)
        self._pending_values.insert(S.B(new_k), new_bias_guess)

        # CombinedImuFactor: (pose_i, vel_i, pose_j, vel_j, bias_i, bias_j, pim)
        imu_factor = gtsam.CombinedImuFactor(
            S.X(prev_k), S.V(prev_k),
            S.X(new_k),  S.V(new_k),
            S.B(prev_k), S.B(new_k),
            self._pim,
        )
        self._pending_graph.push_back(imu_factor)

        # Reset preintegration for next interval using updated bias guess
        self._pim = gtsam.PreintegratedCombinedMeasurements(
            self._params, prev_bias
        )

        # Record guess (will be overwritten by optimized values after optimize())
        self._kf_R.append(R_guess.copy())
        self._kf_p.append(p_guess.copy())
        self._kf_v.append(v_guess.copy())

        return new_k

    # ------------------------------------------------------------------
    # Camera relative-pose factor
    # ------------------------------------------------------------------

    def add_camera_between(
        self,
        kf_i: int,
        kf_j: int,
        T_ij: gtsam.Pose3,
        rot_sigma: float = 0.005,
        trans_sigma: float = 0.05,
    ) -> None:
        """Add a BetweenFactor<Pose3> for a camera relative-pose measurement.

        Args:
            kf_i, kf_j: keyframe indices (kf_j == kf_i + 1 typically).
            T_ij: measured relative pose T_j expressed in frame i.
            rot_sigma: per-component rotation noise sigma [rad].
            trans_sigma: per-component translation noise sigma [m].

        Tangent ordering for Pose3 noise model: [rot(3), trans(3)].
        """
        noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([rot_sigma] * 3 + [trans_sigma] * 3)
        )
        factor = gtsam.BetweenFactorPose3(S.X(kf_i), S.X(kf_j), T_ij, noise)
        self._pending_graph.push_back(factor)

    # ------------------------------------------------------------------
    # GPS absolute-position factor
    # ------------------------------------------------------------------

    def add_gps(
        self,
        kf_idx: int,
        gps_position: np.ndarray,
        gps_sigma: float = 0.45,
    ) -> None:
        """Add a GPSFactor (absolute world-frame position) on keyframe kf_idx.

        GPSFactor is a unary factor on Pose3 that constrains only position
        (x,y,z), not orientation.  It provides the absolute position anchor
        that camera BetweenFactors cannot supply.

        Args:
            kf_idx: keyframe index (the variable X(kf_idx) must already exist).
            gps_position: 3-vector GPS measurement in world frame [m].
            gps_sigma: isotropic position noise sigma [m].
        """
        noise = gtsam.noiseModel.Isotropic.Sigma(3, gps_sigma)
        factor = gtsam.GPSFactor(S.X(kf_idx), gps_position, noise)
        self._pending_graph.push_back(factor)

    # ------------------------------------------------------------------
    # Optimization
    # ------------------------------------------------------------------

    def optimize(self) -> None:
        """Push pending factors + values into iSAM2 and re-linearize."""
        if self._pending_graph.size() > 0 or not self._pending_values.empty():
            self._isam.update(self._pending_graph, self._pending_values)
            self._pending_graph = gtsam.NonlinearFactorGraph()
            self._pending_values = gtsam.Values()
        # Extra pass to improve convergence
        self._isam.update()

    # ------------------------------------------------------------------
    # Result extraction
    # ------------------------------------------------------------------

    def get_estimates(self) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
        """Return optimized (R_list, p_list, v_list) for all keyframes so far.

        R_list[k] is the 3×3 rotation at keyframe k.
        p_list[k] is the 3-vector position.
        v_list[k] is the 3-vector velocity.
        """
        est = self._isam.calculateEstimate()
        n_kf = self._kf_idx + 1
        R_list, p_list, v_list = [], [], []
        for k in range(n_kf):
            pose: gtsam.Pose3 = est.atPose3(S.X(k))
            vel: np.ndarray = np.array(est.atVector(S.V(k)), dtype=float)
            R_list.append(pose.rotation().matrix())
            p_list.append(np.array(pose.translation(), dtype=float))
            v_list.append(vel)
        return R_list, p_list, v_list

    @property
    def num_keyframes(self) -> int:
        return self._kf_idx + 1
