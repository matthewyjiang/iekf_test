# IEKF + Factor Graph Architecture for RGB-D Visual-Inertial SLAM

## Recommended Architecture

Use the **IEKF as the low-latency real-time state propagator** and the **factor graph as the authoritative estimator/back-end**.

```text
High-rate IMU ---------------------> IEKF prediction -----------------> current low-latency pose
                                      ^
                                      |
Factor graph optimized state --------+

IMU preintegration -----------------> factor graph
RGB-D / depth constraints ----------> factor graph
Loop closures ----------------------> factor graph
Priors / GPS / wheel odom ----------> factor graph
```

The cleanest design is:

```text
1. IEKF propagates the latest graph state forward using IMU.
2. Factor graph processes IMU + RGB-D/depth + loop closures.
3. Graph periodically resets/corrects the IEKF state.
4. IEKF output is used for real-time control, tracking, rendering, and prediction.
5. Factor graph output is used as the globally consistent trajectory and map estimate.
```

## Why Include an IEKF?

The IEKF is valuable because it is fast, causal, and low-latency.

It provides:

- High-rate pose, velocity, and bias propagation from IMU data.
- A smooth current-time estimate between camera frames or graph updates.
- A good initial guess for RGB-D tracking, ICP, feature alignment, or scan matching.
- Robust short-term motion prediction during blur, dropped frames, or low-texture scenes.
- A simple state interface for control loops, visualization, AR rendering, or downstream autonomy.

The IEKF is not usually the best place to solve global consistency. It does not naturally handle long histories, loop closures, map-wide corrections, or relinearization over old states.

## Why Include a Factor Graph?

The factor graph is valuable because it optimizes a history of states and constraints.

It provides:

- Joint optimization over poses, velocities, biases, landmarks, and calibration parameters.
- IMU preintegration factors between keyframes.
- RGB-D odometry, ICP, feature reprojection, depth, plane, and landmark factors.
- Loop closure constraints that correct accumulated drift.
- Proper relinearization and smoothing over a sliding window or full trajectory.
- A consistent optimized trajectory for mapping.

The factor graph is usually heavier than an IEKF. It may run at keyframe rate or asynchronously, and its result may arrive with latency.

## What They Provide Differently

| Component | Main Strength | Main Weakness | Typical Rate |
| --- | --- | --- | --- |
| IEKF | Low-latency current state | Local consistency only | IMU rate, e.g. 200-1000 Hz |
| Factor graph | Globally consistent optimized state | More compute and latency | Camera/keyframe rate, e.g. 5-30 Hz |

The IEKF answers:

```text
Where am I right now, using the latest IMU data?
```

The factor graph answers:

```text
What trajectory and map best explain all recent and historical measurements?
```

## Why Combine Them?

Combining them gives both low latency and consistency.

The IEKF handles real-time propagation:

```text
latest optimized graph state + new IMU samples -> current pose now
```

The factor graph handles correction and consistency:

```text
IMU + RGB-D + loop closures + priors -> optimized trajectory/map
```

Together, the system can:

- Control or render using a current-time pose instead of waiting for optimization.
- Track depth/RGB frames with a good motion prior from IMU propagation.
- Correct long-term drift using graph optimization and loop closures.
- Keep IMU biases consistent through graph-estimated bias updates.
- Maintain a map that does not accumulate the same drift as pure dead reckoning.

## Depth Camera Usage

In the recommended architecture, depth is usually processed by the factor graph/front-end, not directly fused into the IEKF as raw pixels.

Depth can produce graph constraints such as:

- RGB-D visual odometry between keyframes.
- ICP or point-cloud alignment constraints.
- 3D landmark observations.
- Plane or surface constraints.
- Dense/semi-dense map alignment constraints.

The IEKF may still use the output of local RGB-D tracking as a correction if the graph is too slow or asynchronous. However, if the graph already uses those same depth measurements, avoid treating the IEKF-corrected pose as an independent graph factor.

## Avoid Double Counting

The most important rule is:

```text
Do not feed an IEKF estimate into the factor graph as an independent measurement
if that IEKF estimate was produced from measurements already used by the graph.
```

Bad pattern:

```text
Raw IMU ------------------> graph IMU factors
Raw depth/RGB-D ----------> graph depth/visual factors
Same IMU + depth ---------> IEKF
IEKF pose ----------------> graph pose factor  # double counts information
```

Good pattern:

```text
Raw IMU ------------------> graph IMU factors
Raw depth/RGB-D ----------> graph depth/visual factors
IEKF pose ----------------> graph initial guess only
Graph optimized state ----> IEKF reset/correction
```

The IEKF state is allowed to initialize or warm-start graph optimization. It should not be added as a strong independent measurement unless its covariance and correlations with existing graph factors are modeled correctly.

## Practical Runtime Loop

At each IMU sample:

```text
IEKF.predict(accel, gyro, dt)
graph_preintegrator.integrateMeasurement(accel, gyro, dt)
```

At each RGB-D frame:

```text
1. Use IEKF pose as the initial guess for tracking/alignment.
2. Estimate frame-to-frame or frame-to-map motion from RGB-D/depth.
3. If this is a keyframe, add graph factors:
   - IMU preintegration factor
   - RGB-D odometry or ICP factor
   - optional landmark/depth/plane factors
4. Run iSAM2 or fixed-lag smoothing.
5. If the graph has a new optimized current state, reset/correct the IEKF.
```

At loop closure:

```text
1. Detect a previous place/keyframe.
2. Estimate relative pose between current and old keyframe.
3. Add a loop-closure factor.
4. Optimize the graph.
5. Push the corrected current pose, velocity, and bias into the IEKF.
```

## When to Use Graph-Only

A graph-only estimator can be enough if:

- Fixed-lag smoothing runs fast enough for your control/rendering latency budget.
- You do not need a high-rate pose between graph updates.
- Your platform tolerates delayed state estimates.

In that case, the IEKF may be unnecessary. The graph can own the whole state estimate.

## When to Use IEKF + Graph

Use both when:

- You need high-rate current-time pose estimates.
- Your graph runs at keyframe rate or asynchronously.
- You need robust prediction through camera dropouts or motion blur.
- You need loop closure and map consistency.
- You want a clean separation between real-time tracking and global optimization.

## Short Summary

Use the **IEKF for now** and the **factor graph for consistency**.

```text
IEKF:         fast, local, current-time, IMU-driven
Factor graph: optimized, historical, globally consistent, map-driven
Combined:    low-latency pose plus corrected trajectory and map
```
