#!/usr/bin/env bash
# P6 verification — headless, scripted check of both judge scenes.
#   A: replay scene — every fault service drives the right /nav/state transition
#   B: live3d scene (headless) — sim-sourced replay derives vision health from the
#      REAL image stream: /kill_camera on sensor_bridge must flip the state.
# Run as the demo user in ~/aether (sync+build first).
source /opt/ros/lyrical/setup.bash
WS=~/aether
cd "$WS"

echo "=== sync + rebuild ==="
for d in src sim eval scripts offline_demo; do rm -rf "$WS/$d"; cp -r "/mnt/d/Work/AETHER/$d" "$WS/$d"; done
find "$WS/src" "$WS/sim" "$WS/eval" "$WS/scripts" -type f -exec sed -i 's/\r$//' {} + 2>/dev/null
colcon build --packages-select aether_msgs > /tmp/v1.log 2>&1 || { tail -3 /tmp/v1.log; exit 1; }
source install/setup.bash
colcon build > /tmp/v2.log 2>&1 || { tail -5 /tmp/v2.log; exit 1; }
source install/setup.bash
echo BUILD_OK

state() { timeout 12 ros2 topic echo /nav/state --once 2>/dev/null | grep -oP '(?<=data: ).*' | head -1; }

echo "=== A: replay scene ==="
ros2 daemon stop > /dev/null 2>&1; ros2 daemon start > /dev/null 2>&1
ros2 launch aether_bringup replay.launch.py > /tmp/replay_scene.log 2>&1 &
LP=$!
sleep 10
ros2 topic list > /dev/null 2>&1   # warm discovery
echo "A1 nominal: $(state)"
ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: true}" > /dev/null 2>&1
sleep 2;  echo "A2 after kill: $(state)"
ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: false}" > /dev/null 2>&1
sleep 3;  echo "A3 after restore: $(state)"
ros2 service call /inject/feature_starvation std_srvs/srv/SetBool "{data: true}" > /dev/null 2>&1
sleep 2;  echo "A4 starved: $(state)"
ros2 service call /inject/feature_starvation std_srvs/srv/SetBool "{data: false}" > /dev/null 2>&1
ros2 service call /inject/imu_bias std_srvs/srv/SetBool "{data: true}" > /dev/null 2>&1
sleep 14; echo "A5 imu-bias (expect DEGRADED via sol-sep): $(state)"
ros2 service call /inject/imu_bias std_srvs/srv/SetBool "{data: false}" > /dev/null 2>&1
kill -9 $LP 2>/dev/null; pkill -9 -f replay_node; pkill -9 -f monitor_node; pkill -9 -f manager_node; pkill -9 -f cockpit_node
sleep 2

echo "=== B: live3d scene, headless ==="
export GZ_SIM_RESOURCE_PATH="$WS/sim/models"
ros2 launch aether_bringup live3d.launch.py gui:=false rviz:=false hud:=false > /tmp/live3d.log 2>&1 &
L3=$!
sleep 40
ros2 topic list > /dev/null 2>&1   # warm discovery
# gz up + takeoff + replay(sim) latched onto GT+images
echo "B1 est publishing: $(timeout 6 ros2 topic hz /ov_msckf/odomimu 2>&1 | grep -m1 average || echo NO_EST)"
echo "B2 nominal: $(state)"
ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: true}" > /dev/null 2>&1
sleep 3;  echo "B3 after REAL image-stream kill: $(state)"
ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: false}" > /dev/null 2>&1
sleep 4;  echo "B4 after restore: $(state)"
kill -9 $L3 2>/dev/null
pkill -9 -f "gz sim"; pkill -9 -f parameter_bridge; pkill -9 -f replay_node; pkill -9 -f monitor_node
pkill -9 -f manager_node; pkill -9 -f cockpit_node; pkill -9 -f flight_director; pkill -9 -f bridge_node
ros2 daemon stop > /dev/null 2>&1
echo "SCENES_VERIFY_DONE"
