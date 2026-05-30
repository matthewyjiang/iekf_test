# IEKF IMU/GPS Demo

This example simulates a 3D IMU trajectory with gravity, noisy gyroscope/accelerometer measurements, and GPS-like position fixes. It estimates pose, velocity, and IMU biases with a Lie-group EKF using GTSAM's `NavState` representation of `SE_2(3)`, then visualizes ground truth, IMU dead reckoning, GPS samples, and the corrected estimate in a 3D world.

## Run

```bash
uv sync
uv run iekf-demo --no-animate
uv run iekf-demo
uv run iekf-demo --save out.mp4
```

Useful options:

```bash
uv run iekf-demo --duration 15 --seed 7 --no-animate
uv run iekf-demo --save out.gif
```

## IEKF + Factor Graph Demo

Runs the IEKF as a low-latency front-end and a GTSAM iSAM2 factor graph as the SLAM back-end side-by-side on the same simulated data, then prints a 3-way RMSE comparison.

```bash
uv run iekf-graph-demo --no-animate
uv run iekf-graph-demo --duration 15 --seed 7 --no-animate
uv run iekf-graph-demo --keyframe-stride 40 --no-animate
```

The factor graph uses:
- `CombinedImuFactor` for IMU preintegration between keyframes (`PreintegratedCombinedMeasurements`).
- `BetweenFactor<Pose3>` from simulated camera relative-pose measurements (ground-truth SE3 + Gaussian noise in the Lie algebra).
- `GPSFactor` for absolute position anchor at keyframes coinciding with GPS samples.

Raw IMU feeds both IEKF and graph. Raw GPS feeds IEKF (correction) and graph (GPSFactor). Camera feeds graph only. IEKF state is used as the initial guess for graph variables but never as a factor — no double-counting.

## Notes

The filter stores the navigation state as `gtsam.NavState` with tangent ordering `[rotation, position, velocity]`. IMU propagation uses GTSAM preintegration. The GPS correction is applied as an absolute world-frame position update with a Kalman correction in the NavState tangent space, followed by `NavState.retract`.

Gravity is represented in the navigation/world frame as `[0, 0, -9.81]` m/s^2. Accelerometer measurements are simulated as body-frame specific force, `R.T @ (a - g)`, which is what a real accelerometer observes.
