# AGENTS.md

## Runtime

- Package manager: **uv** (not pip/poetry). Always use `uv run` or `uv sync`.
- Python: **3.11 exactly** (`.python-version` enforces this; 3.12+ breaks GTSAM wheel).
- Entry points: `uv run iekf-demo` (IEKF only), `uv run iekf-graph-demo` (IEKF + factor graph).

```bash
uv sync                                            # install deps into .venv
uv run iekf-demo --no-animate                      # headless IEKF-only, prints RMSE
uv run iekf-demo --save out.mp4                    # save animation
uv run iekf-demo --duration 15 --seed 7 --no-animate

uv run iekf-graph-demo --no-animate                # headless, prints 3-way RMSE table
uv run iekf-graph-demo --duration 15 --seed 7 --no-animate
uv run iekf-graph-demo --keyframe-stride 40 --no-animate
```

No test suite, no lint config, no CI defined in repo yet.

## Key Dependencies

- **gtsam >= 4.2** (Python wheel): `NavState`, `PreintegratedImuMeasurements`, `imuBias.ConstantBias`, `Rot3`, `Pose3`.
- numpy, scipy, matplotlib, imageio/imageio-ffmpeg.
- No Eigen or GTSAM C++ headers needed; Python wheel only.

## Package Layout

```
src/iekf_imu/
  trajectory.py   # ground-truth sim + noisy IMU/GPS data generation
  iekf.py         # SE23IEKF filter + helpers
  main.py         # CLI: IEKF-only demo (iekf-demo)
  camera.py       # simulated camera relative-pose measurements (BetweenFactor source)
  factor_graph.py # GraphBackend: iSAM2 + CombinedImuFactor + BetweenFactor<Pose3> + GPSFactor
  graph_main.py   # CLI: IEKF + factor-graph compare demo (iekf-graph-demo)
  viz.py          # matplotlib 3D animation + error plots + graph compare plot
```

## GTSAM / State Conventions

- State is `gtsam.NavState` = SE₂(3): `(R, p, v)`.
- `NavState` tangent ordering: `[rotation(3), position(3), velocity(3)]`.
- Full filter covariance `P` is 15×15: `[rot(3), pos(3), vel(3), gyro_bias(3), accel_bias(3)]`.
- Gravity: `G_WORLD = [0, 0, -9.81]` in world/navigation frame.
- Accelerometer model: body-frame specific force = `R.T @ (a_world - G_WORLD)`.
- `PreintegrationParams.MakeSharedU(g_magnitude)` — U-convention (gravity up = negative Z in world).
- Jacobians in `predict()` are computed numerically via finite differences on `NavState.retract` / `localCoordinates` — intentional, keeps code tied to GTSAM's exact manifold ops.
- `imuBias.ConstantBias(accel_vec, gyro_vec)` — constructor order is **accel first, gyro second**.
- Bias update in `update_gps`: `dx[9:12]` = gyro correction, `dx[12:15]` = accel correction.

## Simulation

- Helical ground-truth trajectory: `trajectory.py:ground_truth()`.
- IMU @ 200 Hz, GPS @ 10 Hz (every 20 IMU steps).
- Non-zero initial biases baked into `trajectory.py` (`gyro_bias0`, `accel_bias0`); filter starts with zero bias estimate — intentional for demo realism.
- `simulate()` returns `SimData`; `gps_indices` are IMU-step indices, not time indices.

## Architecture Context

`ARCHITECTURE.md` describes the intended underwater SLAM system (IEKF + factor graph + DVL + sonar). Implemented so far:

- IEKF filter (`iekf.py`) with GPS updates — `iekf-demo`.
- Factor-graph back-end (`factor_graph.py`) with:
  - `CombinedImuFactor` for IMU preintegration between keyframes.
  - `BetweenFactor<Pose3>` from simulated camera relative-pose (`camera.py`).
  - `GPSFactor` for absolute position anchor.
  - iSAM2 incremental optimizer.
- Joint demo (`graph_main.py`) compares IEKF vs graph-optimized vs GT — `iekf-graph-demo`.

Not yet implemented: DVL factors, depth/pressure factors, graph → IEKF state reset.

Double-counting rule: raw measurements feed both IEKF and graph independently; IEKF output is only an initial guess for the graph, never a graph factor.

## Key GTSAM API Gotchas (factor_graph.py)

- `gtsam.symbol_shorthand` names are **uppercase only**: `S.X(k)`, `S.V(k)`, `S.B(k)`. `S.x(k)` raises `AttributeError`.
- `CombinedImuFactor` needs `PreintegratedCombinedMeasurements` + `PreintegrationCombinedParams` — **different classes** from `PreintegratedImuMeasurements` / `PreintegrationParams` used in `iekf.py`.
- `PreintegrationCombinedParams` requires `setBiasAccCovariance` + `setBiasOmegaCovariance` + `setBiasAccOmegaInit` — missing these raises at runtime.
- `Pose3` tangent ordering: `[rotation(3), translation(3)]` — rotation **first**. Noise model sigmas must match this order.
- `imuBias.ConstantBias(accel_vec, gyro_vec)` — accel first, gyro second (constructor). But `setBiasAccCovariance` / `setBiasOmegaCovariance` are accel / omega respectively — consistent with constructor.
- Camera `BetweenFactor<Pose3>` gives relative motion only — no absolute position anchor. Must pair with `GPSFactor` or `PriorFactorPose3` to avoid unobservable global translation.
- `NavState.pose()` returns `Pose3` — use to convert IEKF state to graph initial guess.

## Output Artifacts

`*.mp4`, `*.gif`, `*.png` are gitignored. `--save out.mp4` requires `ffmpeg` on PATH (via `imageio-ffmpeg`).
