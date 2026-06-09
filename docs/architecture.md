# A.E.T.H.E.R architecture

Independent ROS2 nodes. **Any subset runs** — graceful degradation is proven by the node graph itself, not promised in prose.

## Node graph & topic contract

| Node | package | subscribes | publishes |
|---|---|---|---|
| **sensor_bridge** | `aether_sensor_bridge` | `/camera/{left,right}/image_raw`, `/imu/data` | `/camera/{left,right}/image`, `/imu0`; srv `/kill_camera` |
| **vio_core** (OpenVINS) | external `ov_msckf` | `/camera/{left,right}/image`, `/imu0` | `/ov_msckf/odomimu` (pose+cov), `/ov_msckf/points_msckf` |
| **sim_replay** | `aether_sim_replay` | — (synthetic source) | `/ov_msckf/odomimu`, `/ov_msckf/points_msckf`, `/aether/ground_truth`, `/imu/data` (synthetic, 50 Hz), `/fault/active` (`std_msgs/Bool`, true while ANY injected fault is on); srvs `/kill_camera`, `/inject/imu_bias`, `/inject/feature_starvation`, `/uwb/enable` |
| **integrity_monitor** | `aether_integrity_monitor` | `/ov_msckf/odomimu`, `/ov_msckf/points_msckf`, `/aether/ground_truth`, `/imu/data` | `/nav/integrity_state`, `/nav/integrity_bound`, `/nav/trust`, `/nav/state`, `/nav/nees`, `/nav/solution_separation` (`Float32`, aided-vs-IMU-only RAIM cross-check) |
| **degradation_manager** | `aether_degradation_manager` | `/nav/integrity_state`, `/fault/active` | `/nav/mode` (`aether_msgs/NavMode`, latched), `/nav/detection_latency` (`Float32`, fault-onset → detection, seconds) |
| **health_cockpit** | `aether_health_cockpit` | `/nav/integrity_bound`, `/nav/trust`, `/nav/state`, `/nav/detection_latency`, `/fault/active`, `/ov_msckf/odomimu`, `/aether/ground_truth` | `/viz/integrity_bound`, `/viz/truth`, `/viz/naive_bound`, `/viz/naive_estimate` (fixed-σ "no integrity layer" ghost for side-by-side comparison), `/viz/labels` (verdict/status text) — all RViz Markers, frame `odom`; the naive ghost and labels live inside `cockpit_node`, not a separate executable |
| **mission_hud** | `aether_health_cockpit` | `/nav/state`, `/nav/trust`, `/nav/integrity_bound`, `/nav/detection_latency`, `/nav/solution_separation`, `/fault/active`, `/ov_msckf/odomimu`, `/aether/ground_truth`, `/viz/naive_*` | — (matplotlib live dashboard; publishes no topics) |

### Services

| Service | type | effect |
|---|---|---|
| `/kill_camera` | `aether_msgs/KillCamera` `{bool enable -> bool acknowledged}` | vision outage on/off |
| `/inject/imu_bias` | `std_srvs/SetBool` | inject an IMU accel/gyro bias fault |
| `/inject/feature_starvation` | `std_srvs/SetBool` | starve the tracker below the feature floor |
| `/uwb/enable` | `std_srvs/SetBool` | toggle the UWB aiding source |

### Message notes

- `aether_msgs/ProtectionLevel` carries `ellipse_yaw` (radians) — the principal-axis yaw of the horizontal error ellipse (eigen-decomposed PL, not a circle).
- `aether_msgs/IntegrityState` carries `solution_separation` (also streamed on `/nav/solution_separation`).
- `/ov_msckf/odomimu` `pose.covariance` is the row-major 6×6 `[x,y,z,rx,ry,rz]`; position block = `[0:3,0:3]`.

## The integrity math (in `integrity_monitor/core.py`, mirror of `offline_demo/aether_core.py`)

- **Position covariance** = top-left `[0:3,0:3]` block of the `nav_msgs/Odometry` `pose.covariance` (order `x,y,z,rx,ry,rz`).
- **Protection level** (per axis) = `k · sqrt(diag(P_pos))`; horizontal PL = `k · sqrt(λ_max(P_2x2))` (oriented eigen-ellipse, published with `ellipse_yaw`).
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
| `integrity.launch.py` | integrity_monitor + degradation_manager + health_cockpit (sensor_bridge is owned by `vio.launch.py` — not duplicated here) | live sim (replays a bag) |
| `replay.launch.py` | sim_replay + the full integrity layer (the guaranteed demo) | OpenVINS and Gazebo |
| `aether.launch.py` | sim_world + vio + integrity; `use_sim:=true/false` | — |

See [`AETHER_BIBLE.pdf`](AETHER_BIBLE.pdf) for the full design and [`AETHER_MANUAL_STEPS.pdf`](AETHER_MANUAL_STEPS.pdf) for the build/run sequence.
