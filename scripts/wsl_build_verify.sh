#!/usr/bin/env bash
# Build the A.E.T.H.E.R workspace in WSL and verify the live integrity demo headlessly:
# replay + monitor + manager, then walk the FULL fault suite (camera kill, IMU bias,
# feature starvation) and confirm state/trust/bound/separation/latency all react.
# Run as: bash wsl_build_verify.sh    (after ROS2 is installed; lyrical or jazzy)
for d in lyrical jazzy humble; do
  [ -f /opt/ros/$d/setup.bash ] && source /opt/ros/$d/setup.bash && break
done

WS=~/aether
rm -rf "$WS"; cp -r /mnt/d/Work/AETHER "$WS"; cd "$WS"
# normalize CRLF (files may come from Windows checkouts)
find . -type f \( -name '*.py' -o -name '*.msg' -o -name '*.srv' -o -name '*.xml' \
  -o -name 'CMakeLists.txt' -o -name '*.yaml' -o -name '*.cfg' -o -name '*.sh' \) -exec sed -i 's/\r$//' {} +
rm -rf build install log bags results

echo "=== BUILD: aether_msgs ==="
colcon build --packages-select aether_msgs || { echo "MSGS_BUILD_FAIL"; exit 1; }
source install/setup.bash
echo "=== BUILD: nodes ==="
colcon build --packages-select aether_sim_replay aether_integrity_monitor \
  aether_degradation_manager aether_health_cockpit aether_sensor_bridge aether_flight \
  || { echo "NODE_BUILD_FAIL"; exit 1; }
source install/setup.bash
echo "BUILD_DONE"

probe () {  # probe <label>  — sample the nav topics once each
  echo "--- $1 ---"
  timeout 6 ros2 topic echo /nav/state --once 2>/dev/null | head -1 || echo "(no /nav/state)"
  timeout 6 ros2 topic echo /nav/trust --once 2>/dev/null | head -1 || echo "(no /nav/trust)"
  timeout 6 ros2 topic echo /nav/integrity_bound --once 2>/dev/null | grep -E "horizontal_pl" || echo "(no bound)"
  timeout 6 ros2 topic echo /nav/solution_separation --once 2>/dev/null | head -1 || echo "(no sep)"
}

echo "=== RUN integrity stack (headless) ==="
ros2 run aether_sim_replay replay_node >/tmp/replay.log 2>&1 &  RP=$!
ros2 run aether_integrity_monitor monitor_node >/tmp/mon.log 2>&1 &  MO=$!
ros2 run aether_degradation_manager manager_node >/tmp/mgr.log 2>&1 &  MG=$!
sleep 8

probe "NOMINAL"

echo "=== FAULT 1: camera kill ==="
timeout 12 ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: true}" >/dev/null 2>&1
sleep 3
probe "CAMERA DEAD (expect INERTIAL, low trust, bloomed PL)"
timeout 6 ros2 topic echo /nav/detection_latency --once 2>/dev/null | head -1 || echo "(no latency)"
timeout 12 ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: false}" >/dev/null 2>&1
sleep 4
probe "RESTORED (expect NOMINAL)"

echo "=== FAULT 2: IMU bias (sensor-only; solution separation must catch it) ==="
timeout 12 ros2 service call /inject/imu_bias std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 5
probe "IMU BIAS (expect DEGRADED via separation, features still ~120)"
timeout 12 ros2 service call /inject/imu_bias std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
sleep 4

echo "=== FAULT 3: feature starvation ==="
timeout 12 ros2 service call /inject/feature_starvation std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 2.5
probe "STARVED (expect DEGRADED, modest PL widening)"
timeout 12 ros2 service call /inject/feature_starvation std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
sleep 3
probe "FINAL (expect NOMINAL)"

echo "--- cleanup ---"
kill -9 $RP $MO $MG 2>/dev/null || true
pkill -9 -f replay_node 2>/dev/null; pkill -9 -f monitor_node 2>/dev/null; pkill -9 -f manager_node 2>/dev/null
ros2 daemon stop 2>/dev/null || true
echo "FULL_VERIFY_DONE"
