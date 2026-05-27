from __future__ import annotations

import argparse

import gtsam
import numpy as np
from scipy.spatial.transform import Rotation

from .iekf import SE23IEKF, arrays_from_navstate, navstate_from_arrays, propagate_dead_reckoning
from .trajectory import simulate
from .viz import animate_world, plot_errors


def attitude_rmse_deg(gt_R: np.ndarray, est_R: np.ndarray) -> float:
    errs = [Rotation.from_matrix(gt_R[i].T @ est_R[i]).magnitude() for i in range(len(gt_R))]
    return float(np.rad2deg(np.sqrt(np.mean(np.square(errs)))))


def main() -> None:
    parser = argparse.ArgumentParser(description="GTSAM Python SE_2(3) IEKF IMU/GPS demo")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=4)
    parser.add_argument("--no-animate", action="store_true")
    parser.add_argument("--save", type=str, default=None, help="Save animation to .mp4 or .gif")
    args = parser.parse_args()

    dt = 1.0 / 200.0
    gyro_sigma = 0.004
    accel_sigma = 0.06
    gps_sigma = 0.45
    gyro_bias_rw = 0.00008
    accel_bias_rw = 0.0015
    data = simulate(duration=args.duration, dt_imu=dt, seed=args.seed, gyro_sigma=gyro_sigma, accel_sigma=accel_sigma, gps_sigma=gps_sigma)

    R0 = data.R[0] @ Rotation.from_rotvec(np.deg2rad([3.0, -2.0, 5.0])).as_matrix()
    p0 = data.p[0] + np.array([1.5, -1.0, 0.8])
    v0 = data.v[0] + np.array([0.2, -0.2, 0.1])
    init_state = navstate_from_arrays(R0, p0, v0)
    init_bias = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))
    P0 = np.diag(
        [np.deg2rad(5.0) ** 2] * 3
        + [2.0**2] * 3
        + [0.8**2] * 3
        + [0.05**2] * 3
        + [0.3**2] * 3
    )

    filt = SE23IEKF(init_state, init_bias, P0, gyro_sigma, accel_sigma, gyro_bias_rw, accel_bias_rw, gps_sigma)
    dr_state = init_state
    dr_bias = init_bias

    est_R = np.zeros_like(data.R)
    est_p = np.zeros_like(data.p)
    est_v = np.zeros_like(data.v)
    dr_p = np.zeros_like(data.p)
    gps_lookup = {int(idx): data.gps[k] for k, idx in enumerate(data.gps_indices)}

    est_R[0], est_p[0], est_v[0] = arrays_from_navstate(filt.state)
    dr_p[0] = np.array(dr_state.position(), dtype=float)

    for i in range(1, len(data.times)):
        filt.predict(data.accel_meas[i - 1], data.gyro_meas[i - 1], dt)
        dr_state = propagate_dead_reckoning(dr_state, dr_bias, data.accel_meas[i - 1], data.gyro_meas[i - 1], dt, gyro_sigma, accel_sigma)
        if i in gps_lookup:
            filt.update_gps(gps_lookup[i])

        est_R[i], est_p[i], est_v[i] = arrays_from_navstate(filt.state)
        dr_p[i] = np.array(dr_state.position(), dtype=float)

    est_pos_rmse = float(np.sqrt(np.mean(np.sum((est_p - data.p) ** 2, axis=1))))
    dr_pos_rmse = float(np.sqrt(np.mean(np.sum((dr_p - data.p) ** 2, axis=1))))
    est_att_rmse = attitude_rmse_deg(data.R, est_R)
    print(f"IEKF position RMSE:        {est_pos_rmse:8.3f} m")
    print(f"Dead-reckoning pos RMSE:  {dr_pos_rmse:8.3f} m")
    print(f"IEKF attitude RMSE:       {est_att_rmse:8.3f} deg")

    plot_errors(data.times, data.R, data.p, est_R, est_p, dr_p)
    if not args.no_animate or args.save:
        animate_world(data.times, data.p, est_p, est_R, dr_p, data.gps, save=args.save)


if __name__ == "__main__":
    main()
