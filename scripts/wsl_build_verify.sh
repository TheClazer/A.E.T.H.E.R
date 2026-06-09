#!/usr/bin/env bash
# Build the A.E.T.H.E.R workspace in WSL and verify the T1 live integrity demo headlessly:
# launch replay + integrity + degradation, kill the camera, confirm the bound blooms and
# trust flips. Run as: bash wsl_build_verify.sh   (after ROS2 is installed)
source /opt/ros/lyrical/setup.bash

WS=~/aether
rm -rf "$WS"; cp -r /mnt/d/Work/AETHER "$WS"; cd "$WS"
# normalize CRLF (files came from Windows)
find . -type f \( -name '*.py' -o -name '*.msg' -o -name '*.srv' -o -name '*.xml' \
  -o -name 'CMakeLists.txt' -o -name '*.yaml' -o -name '*.cfg' \) -exec sed -i 's/\r$//' {} +
rm -rf build install log

echo "=== BUILD: aether_msgs ==="
colcon build --packages-select aether_msgs || { echo "MSGS_BUILD_FAIL"; exit 1; }
source install/setup.bash
echo "=== BUILD: nodes ==="
colcon build --packages-select aether_sim_replay aether_integrity_monitor \
  aether_degradation_manager aether_health_cockpit aether_sensor_bridge \
  || { echo "NODE_BUILD_FAIL"; exit 1; }
source install/setup.bash
echo "BUILD_DONE"

echo "=== RUN T1 (headless) ==="
ros2 run aether_sim_replay replay_node >/tmp/replay.log 2>&1 &  RP=$!
ros2 run aether_integrity_monitor monitor_node >/tmp/mon.log 2>&1 &  MO=$!
ros2 run aether_degradation_manager manager_node >/tmp/mgr.log 2>&1 &  MG=$!
sleep 6

echo "--- NOMINAL ---"
timeout 6 ros2 topic echo /nav/state --once 2>/dev/null || echo "(no /nav/state)"
timeout 6 ros2 topic echo /nav/trust --once 2>/dev/null || echo "(no /nav/trust)"
timeout 6 ros2 topic echo /nav/integrity_bound --once 2>/dev/null | grep -E "horizontal_pl|pl_operational" || echo "(no bound)"

echo "--- KILL CAMERA (inject vision loss) ---"
timeout 12 ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: true}" 2>/dev/null | tail -1
sleep 2.5
timeout 6 ros2 topic echo /nav/state --once 2>/dev/null || echo "(no state)"
timeout 6 ros2 topic echo /nav/trust --once 2>/dev/null || echo "(no trust)"
timeout 6 ros2 topic echo /nav/integrity_bound --once 2>/dev/null | grep -E "horizontal_pl|pl_operational" || echo "(no bound)"

echo "--- RESTORE ---"
timeout 12 ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: false}" 2>/dev/null | tail -1
sleep 2
timeout 6 ros2 topic echo /nav/state --once 2>/dev/null || echo "(no state)"

echo "--- cleanup ---"
kill -9 $RP $MO $MG 2>/dev/null || true
pkill -9 -f replay_node 2>/dev/null; pkill -9 -f monitor_node 2>/dev/null; pkill -9 -f manager_node 2>/dev/null
ros2 daemon stop 2>/dev/null || true
echo "T1_VERIFY_DONE"
