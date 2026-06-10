#!/usr/bin/env bash
# Record the A.E.T.H.E.R golden bag: headless tunnel sim + flight_director
# profile -> bags/tunnel_run (stereo + IMU + ground truth + clock, ~150 s).
# Run inside WSL (ROS2 Lyrical + ros_gz installed, see wsl_test_gazebo.sh).
#
# Headless rendering: the stereo cameras DO need a render context even with
# `gz sim -s` (server only). WSLg's GPU passthrough usually provides one; if
# the cameras stay silent / ogre2 aborts in ~/gzrec.log, force Mesa software
# rendering by uncommenting the next line (slower than realtime but correct):
# export LIBGL_ALWAYS_SOFTWARE=1
set -o pipefail   # NOTE: no -e/-u — ROS setup scripts use unbound vars; cleanup must run
source /opt/ros/lyrical/setup.bash

RECORD_SECS="${RECORD_SECS:-150}"
WS=~/aether
[ -d "$WS/sim" ] || { rm -rf "$WS"; cp -r /mnt/d/Work/AETHER "$WS"; }
find "$WS/sim" -type f \( -name '*.sdf' -o -name '*.config' -o -name '*.yaml' \) -exec sed -i 's/\r$//' {} +
export GZ_SIM_RESOURCE_PATH="$WS/sim/models"
cd "$WS"

# regenerate the wall textures if they are missing (deterministic, seeded)
[ -f "$WS/sim/models/tunnel_walls/materials/textures/feature_rich.png" ] || \
  python3 "$WS/sim/textures/gen_textures.py"

GZ= BR= FLY= REC=
cleanup() {
  set +e
  echo "[cleanup] stopping recorder / flight / bridge / gz"
  for p in "$REC" "$FLY" "$BR" "$GZ"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  sleep 3
  for p in "$REC" "$FLY" "$BR" "$GZ"; do [ -n "$p" ] && kill -9 "$p" 2>/dev/null; done
  pkill -9 -f "ros2 bag record" 2>/dev/null
  pkill -9 -f flight_director 2>/dev/null
  pkill -9 -f parameter_bridge 2>/dev/null
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f "ruby" 2>/dev/null
}
trap cleanup EXIT

echo "[1/4] launch tunnel world headless (server only, real-time)"
gz sim -s -r "$WS/sim/worlds/tunnel.sdf" > ~/gzrec.log 2>&1 &
GZ=$!
sleep 12

echo "[2/4] ros_gz parameter bridge (clock/stereo/imu/GT + cmd_vel ROS->GZ)"
ros2 run ros_gz_bridge parameter_bridge \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  "/camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU" \
  "/aether/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  --ros-args -p use_sim_time:=true > ~/bridge_rec.log 2>&1 &
BR=$!
sleep 5

echo "[3/4] start recorder (${RECORD_SECS}s) + flight_director"
rm -rf "$WS/bags/tunnel_run"; mkdir -p "$WS/bags"
# (no sim-time flag needed: header stamps carry sim time and /clock is recorded)
# wall-clock cap = 2x the sim-time profile (rotor physics runs at RTF ~0.6 headless)
timeout -k 10 "$((RECORD_SECS * 2 + 20))" ros2 bag record -o "$WS/bags/tunnel_run" --topics \
  /camera/left/image_raw /camera/right/image_raw /imu/data \
  /aether/ground_truth /clock > ~/bag_rec.log 2>&1 &
REC=$!
sleep 2

# prefer the installed entry point; fall back to running the module from source
source "$WS/install/setup.bash" 2>/dev/null || true
if ros2 pkg prefix aether_flight > /dev/null 2>&1; then
  timeout -k 10 "$((RECORD_SECS * 2 + 30))" ros2 run aether_flight flight_director \
    --ros-args -p use_sim_time:=true -p duration:="${RECORD_SECS}.0" > ~/flight_rec.log 2>&1 &
else
  timeout -k 10 "$((RECORD_SECS * 2 + 30))" python3 "$WS/src/aether_flight/aether_flight/flight_director.py" \
    --ros-args -p use_sim_time:=true -p duration:="${RECORD_SECS}.0" > ~/flight_rec.log 2>&1 &
fi
FLY=$!

echo "[4/4] recording ~${RECORD_SECS}s of sim time profile..."
( sleep 12
  echo "--- mid-run diagnostics ---"
  ros2 topic list 2>/dev/null | grep -E "camera|imu|ground_truth" || echo "(no sim topics on ROS side)"
  timeout 6 ros2 topic hz /imu/data 2>/dev/null | head -2 || echo "(no IMU rate)"
  timeout 8 ros2 topic hz /camera/left/image_raw 2>/dev/null | head -2 || echo "(no camera rate — check render context / LIBGL_ALWAYS_SOFTWARE)"
  timeout 6 ros2 topic echo /aether/ground_truth --once --field pose.pose.position 2>/dev/null || echo "(no GT)"
) &
wait "$REC"
cleanup
trap - EXIT

echo "=== bag info ==="
ros2 bag info "$WS/bags/tunnel_run" 2>/dev/null || echo "BAG_MISSING (check ~/gzrec.log ~/bridge_rec.log ~/bag_rec.log ~/flight_rec.log)"
echo "RECORD_BAG_DONE  (bag at $WS/bags/tunnel_run)"
