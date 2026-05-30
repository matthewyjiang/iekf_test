from __future__ import annotations

from pathlib import Path
from typing import Optional

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


def plot_graph_compare(
    times: np.ndarray,
    gt_p: np.ndarray,
    iekf_p: np.ndarray,
    kf_indices: np.ndarray,
    graph_p: np.ndarray,
    save: Optional[str] = None,
) -> None:
    """3D trajectory + per-axis error plots comparing IEKF vs graph vs GT.

    Args:
        times:      (N,)   full IMU-step time array.
        gt_p:       (N,3)  ground-truth positions.
        iekf_p:     (N,3)  IEKF estimated positions (dense, IMU rate).
        kf_indices: (K,)   IMU-step indices of the K keyframes.
        graph_p:    (K,3)  graph-optimised positions at keyframes.
        save:       if given, save figure instead of showing.
    """
    kf_times = times[kf_indices]
    gt_kf = gt_p[kf_indices]
    iekf_kf = iekf_p[kf_indices]

    graph_err = graph_p - gt_kf       # (K,3)
    iekf_err_kf = iekf_kf - gt_kf    # (K,3)

    fig = plt.figure(figsize=(14, 10))

    # ---- 3D trajectory ----
    ax3d = fig.add_subplot(2, 2, (1, 2), projection="3d")
    all_pts = np.vstack([gt_p, iekf_p, graph_p])
    _set_equal_3d(ax3d, all_pts)
    ax3d.plot(gt_p[:, 0], gt_p[:, 1], gt_p[:, 2],
              color="0.65", lw=2, label="ground truth")
    ax3d.plot(iekf_p[:, 0], iekf_p[:, 1], iekf_p[:, 2],
              color="tab:blue", lw=1.5, alpha=0.7, label="IEKF (dense)")
    ax3d.scatter(graph_p[:, 0], graph_p[:, 1], graph_p[:, 2],
                 c="tab:green", s=18, zorder=5, label="graph keyframes")
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("z [m]")
    ax3d.set_title("IEKF + factor graph: trajectory comparison")
    ax3d.legend(loc="upper right")

    # ---- Position error vs time ----
    ax_pos = fig.add_subplot(2, 2, 3)
    ax_pos.plot(kf_times, iekf_err_kf[:, 0], color="tab:blue",
                alpha=0.6, lw=1, label="IEKF x")
    ax_pos.plot(kf_times, iekf_err_kf[:, 1], color="tab:cyan",
                alpha=0.6, lw=1, label="IEKF y")
    ax_pos.plot(kf_times, iekf_err_kf[:, 2], color="tab:purple",
                alpha=0.6, lw=1, label="IEKF z")
    ax_pos.plot(kf_times, graph_err[:, 0], color="tab:green",
                lw=1.5, label="graph x")
    ax_pos.plot(kf_times, graph_err[:, 1], color="tab:olive",
                lw=1.5, label="graph y")
    ax_pos.plot(kf_times, graph_err[:, 2], color="tab:brown",
                lw=1.5, label="graph z")
    ax_pos.axhline(0, color="k", lw=0.5)
    ax_pos.set_ylabel("position error [m]")
    ax_pos.set_xlabel("time [s]")
    ax_pos.grid(True)
    ax_pos.legend(fontsize=7, ncol=2)
    ax_pos.set_title("Position error (at keyframes)")

    # ---- Position error norm ----
    ax_norm = fig.add_subplot(2, 2, 4)
    ax_norm.plot(kf_times, np.linalg.norm(iekf_err_kf, axis=1),
                 color="tab:blue", lw=1.5, label="IEKF ‖err‖")
    ax_norm.plot(kf_times, np.linalg.norm(graph_err, axis=1),
                 color="tab:green", lw=1.5, label="graph ‖err‖")
    ax_norm.axhline(0, color="k", lw=0.5)
    ax_norm.set_ylabel("‖position error‖ [m]")
    ax_norm.set_xlabel("time [s]")
    ax_norm.grid(True)
    ax_norm.legend()
    ax_norm.set_title("Position error norm (at keyframes)")

    fig.tight_layout()

    if save:
        out = Path(save)
        fig.savefig(out, dpi=130, bbox_inches="tight")
    else:
        plt.show()
