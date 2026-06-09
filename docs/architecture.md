# A.E.T.H.E.R architecture

Five independent ROS2 nodes. **Any subset runs** — graceful degradation is proven by the node graph itself, not promised in prose.

## Node graph & topic contract

| Node | package | subscribes | publishes |
|---|---|---|---|
| **sensor_bridge** | `aether_sensor_bridge` | `/camera/{left,right}/image_raw`, `/imu/data` | `/camera/{left,right}/image`, `/imu0`; srv `/kill_camera` |
| **vio_core** (OpenVINS) | external `ov_msckf` | `/camera/{left,right}/image`, `/imu0` | `/ov_msckf/odomimu` (pose+cov), `/ov_msckf/points_msckf` |
| **integrity_monitor** | `aether_integrity_monitor` | `/ov_msckf/odomimu`, `/ov_msckf/points_msckf`, `/aether/ground_truth` | `/nav/integrity_state`, `/nav/integrity_bound`, `/nav/trust`, `/nav/state`, `/nav/nees` |
| **degradation_manager** | `aether_degradation_manager` | `/nav/integrity_state` | `/nav/mode` (latched) |
| **health_cockpit** | `aether_health_cockpit` | `/nav/integrity_bound`, `/nav/trust`, `/ov_msckf/odomimu`, `/aether/ground_truth` | `/viz/integrity_bound`, `/viz/truth` (RViz Markers) |

## The integrity math (in `integrity_monitor/core.py`, mirror of `offline_demo/aether_core.py`)

- **Position covariance** = top-left `[0:3,0:3]` block of the `nav_msgs/Odometry` `pose.covariance` (order `x,y,z,rx,ry,rz`).
- **Protection level** (per axis) = `k · sqrt(diag(P_pos))`; horizontal PL = `hypot(pl_x, pl_y)`.
  - operational `k_op = sqrt(chi2.ppf(0.95, 2)) ≈ 2.4477`
  - DAL-C `k_ffd = sqrt(chi2.ppf(1 − 1.39e-10, 2)) ≈ 6.7374`
- **NEES** = `eᵀ P_pos⁻¹ e`, `e = x_est − x_gt`; consistent if inside `chi2.interval(0.95, 3) ≈ [0.216, 9.348]`.
- **Trust** = `clip(0.6·min(n_feat/120,1) + 0.4·min(0.02/trace(P),1), 0, 1)`.
- **State machine** NOMINAL→DEGRADED→INERTIAL→RE_ACQUIRE, hysteretic; enters INERTIAL only after `n_feat < 10` persists `0.3 s` (the debounce that *is* the “< 0.5 s” detection).

## Launch hierarchy

| launch | brings up | runs without |
|---|---|---|
| `sim_world.launch.py` | Gazebo Harmonic tunnel + `ros_gz` bridge | — (or feed EuRoC) |
| `vio.launch.py` | sensor_bridge + OpenVINS | the integrity layer (= the DP7 floor) |
| `integrity.launch.py` | integrity_monitor + degradation_manager + health_cockpit | live sim (replays a bag) |
| `aether.launch.py` | all of the above; `use_sim:=true/false` | — |

See [`AETHER_BIBLE.pdf`](AETHER_BIBLE.pdf) for the full design and [`AETHER_MANUAL_STEPS.pdf`](AETHER_MANUAL_STEPS.pdf) for the build/run sequence.
