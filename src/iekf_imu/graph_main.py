"""CLI: run IEKF front-end + factor-graph back-end side-by-side and compare.

Architecture (matches ARCHITECTURE.md):
  - SE23IEKF runs at full IMU rate for low-latency navigation.
  - GraphBackend (iSAM2 + CombinedImuFactor + BetweenFactor<Pose3>) runs
    incrementally at keyframe rate.
  - Camera relative-pose measurements (simulated from GT + noise) feed only
    the graph, not the IEKF.
  - Raw IMU feeds both the IEKF and the graph preintegrator — safe because
    the IEKF state is used only as an initial guess for graph variables,
    never as an independent graph factor.

Usage:
    uv run iekf-graph-demo --no-animate
    uv run iekf-graph-demo --duration 15 --seed 7 --no-animate
    uv run iekf-graph-demo --keyframe-stride 20 --no-animate
"""
from __future__ import annotations

import argparse

import gtsam
import numpy as np
from scipy.spatial.transform import Rotation

from .camera import simulate_camera_rel_poses
from .factor_graph import GraphBackend
from .iekf import SE23IEKF, arrays_from_navstate, navstate_from_arrays
from .trajectory import simulate
from .viz import plot_graph_compare


def _attitude_rmse_deg(gt_R: np.ndarray, est_R: np.ndarray) -> float:
    errs = [
        Rotation.from_matrix(gt_R[i].T @ est_R[i]).magnitude()
        for i in range(len(gt_R))
    ]
    return float(np.rad2deg(np.sqrt(np.mean(np.square(errs)))))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="IEKF + factor-graph demo: IMU preintegration + camera relative pose"
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=4)
    parser.add_argument("--no-animate", action="store_true")
    parser.add_argument(
        "--save", type=str, default=None, help="Save animation to .mp4 or .gif"
    )
    parser.add_argument(
        "--keyframe-stride",
        type=int,
        default=20,
        help="IMU steps between keyframes (default=20 → 10 Hz at 200 Hz IMU)",
    )
    parser.add_argument(
        "--cam-rot-sigma",
        type=float,
        default=0.005,
        help="Camera relative-pose rotation noise sigma [rad]",
    )
    parser.add_argument(
        "--cam-trans-sigma",
        type=float,
        default=0.05,
        help="Camera relative-pose translation noise sigma [m]",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Simulation parameters
    # ------------------------------------------------------------------
    dt = 1.0 / 200.0
    gyro_sigma = 0.004
    accel_sigma = 0.06
    gps_sigma = 0.45        # used for IEKF GPS update (front-end only)
    gyro_bias_rw = 0.00008
    accel_bias_rw = 0.0015

    data = simulate(
        duration=args.duration,
        dt_imu=dt,
        seed=args.seed,
        gyro_sigma=gyro_sigma,
        accel_sigma=accel_sigma,
        gps_sigma=gps_sigma,
    )
    n = len(data.times)

    # ------------------------------------------------------------------
    # Initial state (same perturbation as main.py for fair comparison)
    # ------------------------------------------------------------------
    R0 = data.R[0] @ Rotation.from_rotvec(np.deg2rad([3.0, -2.0, 5.0])).as_matrix()
    p0 = data.p[0] + np.array([1.5, -1.0, 0.8])
    v0 = data.v[0] + np.array([0.2, -0.2, 0.1])
    init_nav = navstate_from_arrays(R0, p0, v0)
    init_bias = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))

    P0 = np.diag(
        [np.deg2rad(5.0) ** 2] * 3
        + [2.0 ** 2] * 3
        + [0.8 ** 2] * 3
        + [0.05 ** 2] * 3
        + [0.3 ** 2] * 3
    )

    # ------------------------------------------------------------------
    # Keyframe indices — every `keyframe_stride` IMU steps
    # ------------------------------------------------------------------
    kf_indices = np.arange(0, n, args.keyframe_stride, dtype=int)
    # Ensure last step included for clean end-point comparison
    if kf_indices[-1] != n - 1:
        kf_indices = np.append(kf_indices, n - 1)

    # ------------------------------------------------------------------
    # Simulate camera relative-pose measurements (graph only)
    # ------------------------------------------------------------------
    cam_rng = np.random.default_rng(args.seed + 100)
    cam_measurements = simulate_camera_rel_poses(
        gt_R=data.R,
        gt_p=data.p,
        keyframe_indices=kf_indices,
        rot_sigma=args.cam_rot_sigma,
        trans_sigma=args.cam_trans_sigma,
        rng=cam_rng,
    )
    # Build lookup: kf_j -> measurement (so we can add factor when kf_j arrives)
    cam_lookup: dict[int, object] = {m.kf_j: m for m in cam_measurements}

    # ------------------------------------------------------------------
    # Initialise IEKF (front-end)
    # ------------------------------------------------------------------
    filt = SE23IEKF(
        init_nav, init_bias, P0,
        gyro_sigma, accel_sigma, gyro_bias_rw, accel_bias_rw, gps_sigma,
    )

    # ------------------------------------------------------------------
    # Initialise graph back-end
    # ------------------------------------------------------------------
    graph = GraphBackend(
        init_R=R0,
        init_p=p0,
        init_v=v0,
        init_bias=init_bias,
        accel_sigma=accel_sigma,
        gyro_sigma=gyro_sigma,
        accel_bias_rw=accel_bias_rw,
        gyro_bias_rw=gyro_bias_rw,
    )

    # ------------------------------------------------------------------
    # Storage
    # ------------------------------------------------------------------
    est_R = np.zeros_like(data.R)
    est_p = np.zeros_like(data.p)
    est_v = np.zeros_like(data.v)
    est_R[0], est_p[0], est_v[0] = arrays_from_navstate(filt.state)

    # Per-keyframe results
    kf_R_graph: list[np.ndarray] = []
    kf_p_graph: list[np.ndarray] = []
    kf_v_graph: list[np.ndarray] = []

    gps_lookup = {int(idx): data.gps[k] for k, idx in enumerate(data.gps_indices)}
    kf_set = set(int(x) for x in kf_indices)
    kf_counter = 0  # how many keyframes added so far (keyframe 0 already in graph)

    # GPS lookup for the graph: nearest GPS sample per keyframe IMU-step
    # GPS fires every 20 IMU steps (data.gps_indices); keyframes also every
    # `keyframe_stride` IMU steps.  When they coincide, add GPS to graph too.
    # Raw GPS feeds IEKF (correction) AND graph (GPSFactor) — safe because
    # IEKF state is never a graph factor.
    gps_set = set(int(idx) for idx in data.gps_indices)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    for i in range(1, n):
        accel = data.accel_meas[i - 1]
        gyro = data.gyro_meas[i - 1]

        # IEKF prediction (front-end)
        filt.predict(accel, gyro, dt)
        # IEKF GPS correction when available
        if i in gps_lookup:
            filt.update_gps(gps_lookup[i])

        # Graph IMU accumulation (both get raw IMU — not double-counting
        # as long as IEKF state stays out of the graph as a factor)
        graph.accumulate_imu(accel, gyro, dt)

        est_R[i], est_p[i], est_v[i] = arrays_from_navstate(filt.state)

        # At each keyframe: close IMU factor, optionally add camera + GPS factors, optimize
        if i in kf_set:
            kf_counter += 1
            iekf_R, iekf_p, iekf_v = arrays_from_navstate(filt.state)
            new_kf_idx = graph.add_keyframe(iekf_R, iekf_p, iekf_v)

            # Camera between-factor for this keyframe pair
            if new_kf_idx in cam_lookup:
                m = cam_lookup[new_kf_idx]
                graph.add_camera_between(
                    kf_i=m.kf_i,
                    kf_j=m.kf_j,
                    T_ij=m.T_ij,
                    rot_sigma=args.cam_rot_sigma,
                    trans_sigma=args.cam_trans_sigma,
                )

            # GPS factor when a GPS sample falls on this keyframe step
            if i in gps_set:
                graph.add_gps(
                    kf_idx=new_kf_idx,
                    gps_position=gps_lookup[i],
                    gps_sigma=gps_sigma,
                )

            graph.optimize()

    # ------------------------------------------------------------------
    # Extract graph estimates at keyframes
    # ------------------------------------------------------------------
    g_R_list, g_p_list, g_v_list = graph.get_estimates()
    kf_R_graph = [np.array(R) for R in g_R_list]
    kf_p_graph = [np.array(p) for p in g_p_list]
    kf_v_graph = [np.array(v) for v in g_v_list]

    # Arrays at keyframe time steps for RMSE comparison
    kf_p_graph_arr = np.array(kf_p_graph)
    kf_R_graph_arr = np.array(kf_R_graph)
    gt_p_kf = data.p[kf_indices[: len(kf_p_graph)]]
    gt_R_kf = data.R[kf_indices[: len(kf_R_graph)]]

    iekf_p_kf = est_p[kf_indices[: len(kf_p_graph)]]
    iekf_R_kf = est_R[kf_indices[: len(kf_R_graph)]]

    # ------------------------------------------------------------------
    # RMSE
    # ------------------------------------------------------------------
    iekf_pos_rmse = float(np.sqrt(np.mean(np.sum((iekf_p_kf - gt_p_kf) ** 2, axis=1))))
    graph_pos_rmse = float(np.sqrt(np.mean(np.sum((kf_p_graph_arr - gt_p_kf) ** 2, axis=1))))
    iekf_att_rmse = _attitude_rmse_deg(gt_R_kf, iekf_R_kf)
    graph_att_rmse = _attitude_rmse_deg(gt_R_kf, kf_R_graph_arr)

    print()
    print("=" * 52)
    print("  Comparison at keyframes (IEKF vs graph vs GT)")
    print("=" * 52)
    print(f"  IEKF  position RMSE  : {iekf_pos_rmse:8.3f} m")
    print(f"  Graph position RMSE  : {graph_pos_rmse:8.3f} m")
    print(f"  IEKF  attitude RMSE  : {iekf_att_rmse:8.3f} deg")
    print(f"  Graph attitude RMSE  : {graph_att_rmse:8.3f} deg")
    print(f"  Keyframes            : {len(kf_p_graph)}")
    print(f"  Camera factors       : {len(cam_measurements)}")
    print(f"  IMU factors          : {len(kf_p_graph) - 1}")
    print("=" * 52)
    print()

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------
    if not args.no_animate or args.save:
        plot_graph_compare(
            times=data.times,
            gt_p=data.p,
            iekf_p=est_p,
            kf_indices=kf_indices[: len(kf_p_graph)],
            graph_p=kf_p_graph_arr,
            save=args.save,
        )


if __name__ == "__main__":
    main()
