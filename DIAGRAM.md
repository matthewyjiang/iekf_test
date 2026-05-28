# Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          Raw Sensors                                     │
│                                                                          │
│   IMU (400Hz)          DVL (10Hz)         Depth Camera (30Hz)            │
└──────┬─────────────────────┬──────────────────────┬──────────────────────┘
       │                     │                      │
       │         ┌───────────┘                      │
       │         │                                  │
       ▼         ▼                                  │
┌──────────────────────┐                            │
│    IEKF frontend     │                            │
│                      │                            │
│  IMU: propagation    │ <-- pose correction fdbk - │ -----------------------
│  DVL: meas. update   │                            │
└──────┬───────────────┘                            │
       │                                            │
       │  fast pose (continuous)                    │
       │                                            │
       ├──────────────────────────────────────────► scan deskewing
       ├──────────────────────────────────────────► initial guess for ICP
       └──────────────────────────────────────────► control feedback

       │  IMU (400 HZ)       │    DVL (10Hz)        │ Depth Camera (30Hz)  
       │  (same raw streams) │                      │
       ▼                     ▼                      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                        Factor graph backend                              │
│                                                                          │
│   IMU preintegration    DVL velocity        Point cloud processing       │
│   factor                factor              (ICP / feature extraction)   │
│   [x_i --- x_j]         [x_i --- x_j]                                    │
│                                              ├─► pose factor [x_i - x_j] │
│                                              ├─► landmark factors        │
│                                              └─► loop closure factors    │
│                                                                          │
│                     Nonlinear solver (GTSAM)                             │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               ▼
                    Optimized trajectory + map
                               │
                               └──────────────────► pose correction
                                                    feedback to IEKF
```
