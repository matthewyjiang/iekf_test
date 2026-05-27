# IEKF + Factor Graph Architecture for Underwater SLAM

## Recommended Architecture

For an underwater robot with **IMU, DVL, pressure/depth sensor, depth camera, and 360 2D sonar**, use the **IEKF as the low-latency local navigation estimator** and the **factor graph as the authoritative SLAM back-end**.

The best default architecture is:

```text
IMU -----------------------------> IEKF propagation -----------------> current low-latency state
DVL velocity --------------------> IEKF local correction ------------+
pressure/depth ------------------> IEKF depth correction ------------+
                                      ^
                                      |
Factor graph optimized state --------+

IMU preintegration --------------> factor graph
DVL velocity / delta factors ----> factor graph
pressure/depth factors ----------> factor graph
depth-camera constraints --------> factor graph
360 sonar constraints -----------> factor graph
loop closures -------------------> factor graph
surface GPS / acoustic fixes ----> factor graph, if available
```

Use the IEKF for **current-time navigation** and the factor graph for **history, mapping, and global consistency**.

## Main Recommendation

For this underwater setup, the cleanest split is:

```text
IEKF gets:          IMU + DVL + pressure/depth + graph resets
Factor graph gets:  IMU + DVL + pressure/depth + camera/depth + sonar + loop closures
```

The IEKF should usually **not** consume the depth camera or 360 sonar as normal parallel updates. Let the visual/sonar front-end turn those sensors into graph constraints. The IEKF state can be used as the initial guess for camera/sonar alignment.

This avoids double counting while still giving the robot a low-latency navigation estimate.

## Why This Is Different From Aerial/Ground VIO

Underwater robots usually do not have continuous GPS. They also often have degraded optical sensing due to turbidity, lighting, suspended particles, and low texture. The reliable local navigation stack is usually:

```text
IMU + DVL + pressure/depth
```

Those sensors are good enough to keep a local navigation estimate bounded for short-to-medium periods. The camera and sonar are then used mainly for map-relative correction, place recognition, loop closure, terrain/object constraints, and drift reduction.

## What Each Sensor Provides

| Sensor | Best Use In IEKF | Best Use In Factor Graph |
| --- | --- | --- |
| IMU | High-rate attitude, velocity, and position propagation | Preintegrated IMU factors between keyframes |
| DVL | Body-frame or beam-frame velocity correction | Velocity factors, water-track/bottom-track factors, delta-pose constraints |
| Pressure/depth | Direct vertical position correction | Depth prior/factor on z or altitude/depth state |
| Depth camera | Usually initial-guess consumer only | RGB-D odometry, ICP, point-cloud/landmark/plane factors |
| 360 2D sonar | Usually initial-guess consumer only | Scan matching, sonar odometry, loop closures, wall/pipe/structure factors |
| Acoustic positioning | Optional correction if low-rate | Absolute/relative position factors, range factors |
| Surface GPS | Optional reset when surfaced | Absolute pose/position prior factors |

## IEKF Role

The IEKF estimates the current navigation state:

```text
x = {
  R_wb,      body orientation in world
  p_wb,      body position in world
  v_wb,      body velocity in world
  b_g,       gyro bias
  b_a        accelerometer bias
}
```

For an underwater robot, the IEKF should normally use:

```text
IMU prediction
DVL velocity update
pressure/depth update
graph optimized-state reset
```

The DVL is especially important because it directly constrains velocity. Without DVL, the IMU position estimate drifts quickly. With DVL and pressure depth, the IEKF can provide a useful real-time navigation solution even when camera/sonar processing is delayed.

The IEKF answers:

```text
Where is the vehicle right now for control, stabilization, tracking, and planning?
```

## Factor Graph Role

The factor graph owns the best estimate of the trajectory and map.

Typical variables:

```text
X_i = vehicle pose at keyframe i
V_i = vehicle velocity at keyframe i
B_i = IMU bias at keyframe i
L_j = landmarks or map features, optional
C   = calibration parameters, optional
```

Typical factors:

```text
PriorFactor<Pose3>                 initial pose or surface GPS prior
PriorFactor<Vector3>               initial velocity prior
PriorFactor<imuBias>               initial IMU bias prior
ImuFactor / CombinedImuFactor      preintegrated IMU between keyframes
DVL velocity factor                measured body/water/bottom-relative velocity
Depth/pressure factor              z/depth measurement
BetweenFactor<Pose3>               camera or sonar odometry
ICP / scan-matching factor         depth-camera or sonar alignment
RangeFactor                        acoustic ranging, if available
Loop closure factor                revisiting known structure or terrain
```

The factor graph answers:

```text
What trajectory and map best explain all IMU, DVL, pressure, camera, sonar, and loop-closure data?
```

## DVL Handling

DVL measurements are commonly body-frame velocity measurements.

If bottom lock is valid:

```text
z_dvl ~= R_bw * v_wb + noise
```

where `R_bw` rotates world velocity into the body frame.

If the DVL measures water-relative velocity instead of bottom-relative velocity, account for current:

```text
z_dvl ~= R_bw * (v_wb - v_current_world) + noise
```

For water-track operation, the water current may need to be estimated as an additional state or modeled as increased noise. Bottom-track DVL is much easier for localization.

Recommended usage:

```text
IEKF:         DVL velocity update at DVL rate
Factor graph: DVL velocity factors at keyframes or integrated intervals
```

This is not harmful double counting as long as the IEKF output is not added back to the graph as an independent factor. The graph still uses the raw DVL measurements.

## Pressure / Depth Handling

The pressure sensor gives a strong vertical constraint:

```text
z_depth ~= depth(p_wb) + noise
```

Use it in both places:

```text
IEKF:         direct depth correction for real-time vertical stability
Factor graph: depth factor on pose z/depth at timestamps or keyframes
```

As with DVL, this is safe if the IEKF state is only used as a live estimate or graph initial guess, not as an independent graph measurement.

## Depth Camera Handling

The depth camera is usually processed by a front-end that produces graph constraints.

Possible constraints:

```text
RGB-D visual odometry between keyframes
ICP alignment against a local point cloud
3D landmark observations
Plane or seafloor patch constraints
Object/structure constraints, e.g. pipe, hull, wall, dock, cable
```

Recommended usage:

```text
IEKF pose -> initial guess for RGB-D tracking or ICP
Depth camera -> graph factors
Graph optimized state -> IEKF reset/correction
```

Avoid making depth-camera odometry a normal IEKF correction if the same odometry is already being added to the graph. Use it as an IEKF fallback only when the graph is delayed and the controller needs a local correction immediately.

## 360 2D Sonar Handling

A 360 2D sonar is often more reliable than a camera in turbid water, but it has its own geometry and ambiguity problems.

It can provide:

```text
2D scan matching against previous sonar scan
scan-to-submap alignment
loop closure against old sonar submaps
wall, pipe, tunnel, pier, or hull constraints
range-bearing features in the horizontal sonar plane
```

Recommended usage:

```text
IEKF pose -> initial guess for sonar scan matching
360 sonar -> graph factors and loop closures
Graph optimized state -> IEKF reset/correction
```

Do not use sonar scan-matching output as both a strong IEKF update and a strong graph factor by default. If you need sonar to correct the IEKF immediately, treat that correction as temporary and do not feed the resulting IEKF pose back into the graph as a measurement.

## Avoid Double Counting

The most important rule is:

```text
Raw measurements can feed both the IEKF and graph only if the IEKF output is not treated as another independent graph measurement.
```

Safe pattern:

```text
Raw IMU -----------------------> IEKF prediction
Raw DVL -----------------------> IEKF velocity update
Raw pressure ------------------> IEKF depth update

Raw IMU -----------------------> graph IMU factors
Raw DVL -----------------------> graph DVL factors
Raw pressure ------------------> graph depth factors
Raw camera/depth --------------> graph visual/depth factors
Raw 360 sonar -----------------> graph sonar factors

IEKF state --------------------> graph initial guess only
Graph optimized state ---------> IEKF reset/correction
```

Risky pattern:

```text
Raw IMU/DVL/pressure/camera/sonar -> IEKF
Raw IMU/DVL/pressure/camera/sonar -> graph factors
IEKF pose ------------------------> graph pose factor  # double counting
```

## Practical Runtime Loop

At each IMU sample:

```text
IEKF.predict(accel, gyro, dt)
graph_imu_preintegrator.integrateMeasurement(accel, gyro, dt)
```

At each DVL sample:

```text
1. Validate bottom lock, beam quality, altitude, and outliers.
2. IEKF.update_dvl_velocity(z_dvl)
3. Store raw DVL measurement for graph factor creation.
```

At each pressure/depth sample:

```text
1. Convert pressure to depth using water density and calibration.
2. IEKF.update_depth(z_depth)
3. Store depth measurement for graph factor creation.
```

At each depth-camera frame:

```text
1. Use IEKF pose as initial guess.
2. Run RGB-D odometry, ICP, or local map alignment.
3. If keyframe, add visual/depth factors to graph.
4. Do not add the IEKF pose itself as a graph measurement.
```

At each 360 sonar scan:

```text
1. Use IEKF pose as initial guess for scan matching.
2. Match scan to previous scan, local submap, or historical submap.
3. Add sonar odometry, scan-to-map, or loop-closure factor to graph.
4. Reject ambiguous matches with robust gating.
```

At graph update:

```text
1. Run iSAM2 or fixed-lag smoothing.
2. Extract optimized pose, velocity, and IMU bias near the current time.
3. Reset/correct the IEKF state.
4. Continue IEKF propagation with new IMU samples.
```

## When To Feed Camera/Sonar Into The IEKF

Default:

```text
Camera/depth and sonar go to the graph, not directly to IEKF.
```

Use camera/sonar as an IEKF update only when:

```text
graph latency is too high for control
DVL is unavailable or has lost bottom lock
the vehicle needs a temporary local correction
the graph back-end is dropped or overloaded
```

If you do this, keep the accounting clean:

```text
camera/sonar correction -> IEKF live state
same camera/sonar measurement -> graph factor
IEKF live state -> graph initial guess only
```

## Best Architecture In One Sentence

For an underwater robot, the best default is:

```text
IEKF = IMU + DVL + pressure for low-latency navigation
Factor graph = IMU + DVL + pressure + depth camera + 360 sonar + loop closures for SLAM consistency
Graph result = periodic reset/correction for the IEKF
```
