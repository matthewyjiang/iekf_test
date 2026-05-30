"""Simulated camera relative-pose measurements for the factor-graph demo.

For each consecutive keyframe pair (i, j) we compute the true SE(3) relative
pose from ground-truth, then corrupt it with small Gaussian noise in the Lie
algebra tangent space.

GTSAM Pose3 tangent ordering: [rotation(3), translation(3)]  — rot FIRST.
This is different from NavState tangent: [rotation(3), position(3), velocity(3)].
"""
from __future__ import annotations

from dataclasses import dataclass

import gtsam
import numpy as np


@dataclass(frozen=True)
class CameraRelPose:
    """Noisy relative-pose measurement between keyframe indices i and j."""

    kf_i: int  # keyframe index in the keyframe list
    kf_j: int  # keyframe index in the keyframe list
    T_ij: gtsam.Pose3  # measured relative pose: T_j expressed in frame i


def simulate_camera_rel_poses(
    gt_R: np.ndarray,
    gt_p: np.ndarray,
    keyframe_indices: np.ndarray,
    rot_sigma: float = 0.005,    # rad  (~0.3 deg 1-sigma)
    trans_sigma: float = 0.05,   # m
    rng: np.random.Generator | None = None,
) -> list[CameraRelPose]:
    """Return noisy relative-pose measurements for consecutive keyframe pairs.

    Args:
        gt_R: (N,3,3) ground-truth rotation matrices over full trajectory.
        gt_p: (N,3)   ground-truth positions over full trajectory.
        keyframe_indices: IMU-step indices that define keyframes.
        rot_sigma: 1-sigma noise on each rotation tangent component [rad].
        trans_sigma: 1-sigma noise on each translation tangent component [m].
        rng: numpy random generator; created from seed 0 if None.

    Returns:
        List of CameraRelPose, one per consecutive keyframe pair.

    Notes:
        Noise is added in the Pose3 tangent space via Pose3.Expmap(noise6)
        and composed on the right: T_ij_noisy = T_ij_true * Expmap(noise).
        Tangent order: [omega_x, omega_y, omega_z, t_x, t_y, t_z].
    """
    if rng is None:
        rng = np.random.default_rng(0)

    noise_sigmas = np.array([rot_sigma] * 3 + [trans_sigma] * 3)

    measurements: list[CameraRelPose] = []
    for k in range(len(keyframe_indices) - 1):
        idx_i = int(keyframe_indices[k])
        idx_j = int(keyframe_indices[k + 1])

        pose_i = gtsam.Pose3(gtsam.Rot3(gt_R[idx_i]), gt_p[idx_i])
        pose_j = gtsam.Pose3(gtsam.Rot3(gt_R[idx_j]), gt_p[idx_j])

        T_ij_true: gtsam.Pose3 = pose_i.between(pose_j)

        noise6 = rng.normal(0.0, noise_sigmas)
        T_noise = gtsam.Pose3.Expmap(noise6)
        T_ij_meas = T_ij_true.compose(T_noise)

        measurements.append(CameraRelPose(kf_i=k, kf_j=k + 1, T_ij=T_ij_meas))

    return measurements
