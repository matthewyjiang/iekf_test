from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.spatial.transform import Rotation


def _set_equal_3d(ax, pts: np.ndarray) -> None:
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    centers = 0.5 * (mins + maxs)
    radius = 0.55 * float(np.max(maxs - mins))
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def plot_errors(times: np.ndarray, gt_R: np.ndarray, gt_p: np.ndarray, est_R: np.ndarray, est_p: np.ndarray, dr_p: np.ndarray) -> None:
    pos_err = est_p - gt_p
    dr_err = dr_p - gt_p
    att_err = np.array([(Rotation.from_matrix(gt_R[i].T @ est_R[i]).as_rotvec()) for i in range(len(times))])

    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axs[0].plot(times, pos_err[:, 0], label="est x")
    axs[0].plot(times, pos_err[:, 1], label="est y")
    axs[0].plot(times, pos_err[:, 2], label="est z")
    axs[0].plot(times, np.linalg.norm(dr_err, axis=1), "--", label="dead-reckoning norm")
    axs[0].set_ylabel("position error [m]")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(times, np.rad2deg(att_err[:, 0]), label="roll")
    axs[1].plot(times, np.rad2deg(att_err[:, 1]), label="pitch")
    axs[1].plot(times, np.rad2deg(att_err[:, 2]), label="yaw")
    axs[1].set_ylabel("attitude error [deg]")
    axs[1].set_xlabel("time [s]")
    axs[1].grid(True)
    axs[1].legend()
    fig.tight_layout()


def animate_world(
    times: np.ndarray,
    gt_p: np.ndarray,
    est_p: np.ndarray,
    est_R: np.ndarray,
    dr_p: np.ndarray,
    gps: np.ndarray,
    save: str | None = None,
) -> None:
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    all_pts = np.vstack([gt_p, est_p, dr_p, gps])
    _set_equal_3d(ax, all_pts)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("SE_2(3) IEKF: IMU + GPS with gravity")

    ax.plot(gt_p[:, 0], gt_p[:, 1], gt_p[:, 2], color="0.75", lw=2, label="ground truth")
    ax.scatter(gps[:, 0], gps[:, 1], gps[:, 2], c="tab:red", s=8, alpha=0.35, label="GPS")
    est_line, = ax.plot([], [], [], color="tab:blue", lw=2.5, label="IEKF")
    dr_line, = ax.plot([], [], [], color="tab:orange", lw=1.5, label="dead reckoning")
    body_lines = [ax.plot([], [], [], lw=2)[0] for _ in range(3)]
    grav_line, = ax.plot([], [], [], color="black", lw=2, label="gravity")
    ax.legend(loc="upper right")

    stride = max(1, len(times) // 500)
    frames = np.arange(0, len(times), stride)
    axis_colors = ["r", "g", "b"]
    for line, color in zip(body_lines, axis_colors):
        line.set_color(color)

    def update(frame_idx: int):
        i = int(frames[frame_idx])
        est_line.set_data(est_p[: i + 1, 0], est_p[: i + 1, 1])
        est_line.set_3d_properties(est_p[: i + 1, 2])
        dr_line.set_data(dr_p[: i + 1, 0], dr_p[: i + 1, 1])
        dr_line.set_3d_properties(dr_p[: i + 1, 2])

        origin = est_p[i]
        scale = 1.2
        for k, line in enumerate(body_lines):
            tip = origin + scale * est_R[i, :, k]
            line.set_data([origin[0], tip[0]], [origin[1], tip[1]])
            line.set_3d_properties([origin[2], tip[2]])

        g_tip = origin + np.array([0.0, 0.0, -2.0])
        grav_line.set_data([origin[0], g_tip[0]], [origin[1], g_tip[1]])
        grav_line.set_3d_properties([origin[2], g_tip[2]])
        return [est_line, dr_line, grav_line, *body_lines]

    ani = FuncAnimation(fig, update, frames=len(frames), interval=25, blit=False)
    if save:
        out = Path(save)
        if out.suffix.lower() == ".gif":
            ani.save(out, writer=PillowWriter(fps=30))
        else:
            ani.save(out, writer="ffmpeg", fps=30, dpi=130)
    else:
        plt.show()
