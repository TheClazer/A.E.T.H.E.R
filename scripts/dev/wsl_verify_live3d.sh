#!/usr/bin/env bash
# Verify the FINAL live3d architecture exactly as judge_demo.sh runs it:
# standalone gated gz server -> live3d_stack (headless: no rviz/hud).
source /opt/ros/lyrical/setup.bash
WS=/root/aether
for d in src sim eval scripts; do rm -rf "$WS/$d"; cp -r "/mnt/d/Work/AETHER/$d" "$WS/$d"; done
find "$WS/src" "$WS/sim" "$WS/scripts" "$WS/eval" -type f -exec sed -i 's/\r$//' {} + 2>/dev/null
cd "$WS"
source install/setup.bash 2>/dev/null
colcon build --packages-select aether_bringup aether_sim_replay aether_flight > /tmp/rb.log 2>&1 || { tail -3 /tmp/rb.log; exit 1; }
source install/setup.bash
echo BUILD_OK

export GZ_SIM_RESOURCE_PATH="$WS/sim/models"
pkill -9 -f "gz sim" 2>/dev/null; sleep 1
SRV_OK=0
for attempt in 1 2 3; do
  echo "server attempt $attempt"
  ( unset DISPLAY WAYLAND_DISPLAY; gz sim -s -r "$WS/sim/worlds/tunnel.sdf" > /tmp/gz_srv.log 2>&1 ) &
  GZ_PID=$!
  for i in $(seq 1 30); do
    sleep 1
    if gz topic -l 2>/dev/null | grep -q "/camera/left/image_raw"; then
      if timeout 6 gz topic -e -t /camera/left/image_raw -n 1 2>/dev/null | grep -q stamp; then SRV_OK=1; break; fi
    fi
    kill -0 $GZ_PID 2>/dev/null || break
  done
  [ "$SRV_OK" = "1" ] && break
  pkill -9 -f "gz sim" 2>/dev/null; sleep 2
done
echo "SERVER_OK=$SRV_OK (attempt $attempt)"
[ "$SRV_OK" = "1" ] || exit 1

ros2 daemon stop > /dev/null 2>&1; ros2 daemon start > /dev/null 2>&1
ros2 launch aether_bringup live3d_stack.launch.py rviz:=false hud:=false > /tmp/stack.log 2>&1 &
ST=$!
sleep 25
ros2 topic list > /dev/null 2>&1
state() { timeout 12 ros2 topic echo /nav/state --once 2>/dev/null | grep -oP '(?<=data: ).*' | head -1; }
echo "B0 images(ros): $(timeout 8 ros2 topic hz /camera/left/image 2>&1 | grep -m1 average || echo NO_IMAGES)"
echo "B1 est: $(timeout 6 ros2 topic hz /ov_msckf/odomimu 2>&1 | grep -m1 average || echo NO_EST)"
echo "B2 nominal: $(state)"
ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: true}" > /dev/null 2>&1
sleep 3;  echo "B3 after REAL image-stream kill: $(state)"
ros2 service call /kill_camera aether_msgs/srv/KillCamera "{enable: false}" > /dev/null 2>&1
sleep 5;  echo "B4 after restore: $(state)"
kill -9 $ST 2>/dev/null
pkill -9 -f "gz sim"; pkill -9 -f parameter_bridge; pkill -9 -f replay_node; pkill -9 -f monitor_node
pkill -9 -f manager_node; pkill -9 -f cockpit_node; pkill -9 -f flight_director; pkill -9 -f bridge_node
ros2 daemon stop > /dev/null 2>&1
echo "LIVE3D_VERIFY_DONE"
