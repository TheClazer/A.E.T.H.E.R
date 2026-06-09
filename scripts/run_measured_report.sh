#!/usr/bin/env bash
# A.E.T.H.E.R measured-report pipeline (U12): one command -> measured figures.
#
# Launches the guaranteed replay demo, records every integrity topic to CSV
# (eval/record_metrics.py), runs the scripted fault beat (camera kill ->
# IMU bias -> feature starvation), then renders docs/figures/measured_*.png
# with the headline numbers (eval/make_measured_figures.py).
#
# Run:  bash scripts/run_measured_report.sh [results/metrics.csv]
set -u
cd "$(dirname "$0")/.."
CSV="${1:-results/metrics.csv}"

# --- ROS2 environment: prefer Lyrical (host), fall back to Jazzy (container) ---
for distro in lyrical jazzy; do
  if [ -f "/opt/ros/$distro/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$distro/setup.bash"
    break
  fi
done
command -v ros2 >/dev/null 2>&1 || { echo "ERROR: no ROS2 under /opt/ros/{lyrical,jazzy}"; exit 1; }

# --- build the workspace if it has not been built yet ---
if [ ! -f install/setup.bash ]; then
  echo "=== colcon build (first run) ==="
  colcon build --packages-select aether_msgs || { echo "MSGS_BUILD_FAIL"; exit 1; }
  source install/setup.bash
  colcon build --packages-select aether_sim_replay aether_integrity_monitor \
    aether_degradation_manager aether_health_cockpit aether_sensor_bridge \
    aether_bringup || { echo "NODE_BUILD_FAIL"; exit 1; }
fi
source install/setup.bash

LAUNCH_PID=""; REC_PID=""
cleanup() {
  echo "--- cleanup ---"
  [ -n "$REC_PID" ] && kill -INT "$REC_PID" 2>/dev/null
  sleep 1
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 2
  [ -n "$REC_PID" ] && kill -9 "$REC_PID" 2>/dev/null
  [ -n "$LAUNCH_PID" ] && kill -9 "$LAUNCH_PID" 2>/dev/null
  pkill -9 -f record_metrics.py 2>/dev/null
  pkill -9 -f replay_node 2>/dev/null
  pkill -9 -f monitor_node 2>/dev/null
  pkill -9 -f manager_node 2>/dev/null
  pkill -9 -f cockpit_node 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1
  true
}
trap cleanup EXIT INT TERM

# call <service> <type> <request> — a missing service skips the beat, not the run
call() {
  timeout 12 ros2 service call "$1" "$2" "$3" >/dev/null 2>&1 \
    || echo "  ($1 unavailable — beat skipped)"
}

echo "=== launch replay + integrity stack ==="
ros2 launch aether_bringup replay.launch.py >/tmp/aether_measured_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 5

echo "=== start metrics recorder -> $CSV ==="
python3 eval/record_metrics.py "$CSV" >/tmp/aether_measured_record.log 2>&1 &
REC_PID=$!
sleep 2

echo "=== scripted fault beat ==="
echo "--- nominal (8 s)"
sleep 8
echo "--- camera kill (20 s outage)"
call /kill_camera aether_msgs/srv/KillCamera "{enable: true}"
sleep 20
echo "--- camera restore (6 s recovery)"
call /kill_camera aether_msgs/srv/KillCamera "{enable: false}"
sleep 6
echo "--- IMU bias injection (8 s)"
call /inject/imu_bias std_srvs/srv/SetBool "{data: true}"
sleep 8
call /inject/imu_bias std_srvs/srv/SetBool "{data: false}"
sleep 2
echo "--- feature starvation (8 s)"
call /inject/feature_starvation std_srvs/srv/SetBool "{data: true}"
sleep 8
call /inject/feature_starvation std_srvs/srv/SetBool "{data: false}"
sleep 4

echo "=== stop recorder + stack ==="
kill -INT "$REC_PID" 2>/dev/null
sleep 1
kill -INT "$LAUNCH_PID" 2>/dev/null
for _ in 1 2 3 4 5; do
  kill -0 "$LAUNCH_PID" 2>/dev/null || break
  sleep 1
done
cleanup
trap - EXIT INT TERM

echo "=== render measured figures ==="
python3 eval/make_measured_figures.py "$CSV" || exit 1
echo "MEASURED_REPORT_DONE"
