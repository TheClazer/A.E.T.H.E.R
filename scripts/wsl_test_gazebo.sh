#!/usr/bin/env bash
# Install ros_gz + Gazebo for ROS2 Lyrical and headless-verify the A.E.T.H.E.R tunnel
# sim: world loads, drone spawns, IMU + ground-truth publish to ROS2. (Cameras need a
# render context — verified in the GUI via WSLg; IMU/odometry need no rendering.)
export DEBIAN_FRONTEND=noninteractive
source /opt/ros/lyrical/setup.bash

echo "[1/3] install ros_gz + Gazebo"
apt-get install -y ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge 2>&1 | tail -3
source /opt/ros/lyrical/setup.bash   # re-source so the vendored gz CLI is on PATH

WS=~/aether
[ -d "$WS/sim" ] || { rm -rf "$WS"; cp -r /mnt/d/Work/AETHER "$WS"; }
find "$WS/sim" -type f \( -name '*.sdf' -o -name '*.config' \) -exec sed -i 's/\r$//' {} +
export GZ_SIM_RESOURCE_PATH="$WS/sim/models"
cd "$WS"

echo "[2/3] launch tunnel world headless"
gz sim -s -r "$WS/sim/worlds/tunnel.sdf" > /root/gz.log 2>&1 &
GZ=$!
sleep 12

echo "[3/3] bridge + check IMU / ground-truth"
ros2 run ros_gz_bridge parameter_bridge \
  "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU" \
  "/aether/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  --ros-args -p use_sim_time:=true > /root/bridge.log 2>&1 &
BR=$!
sleep 7
echo "--- gz topics ---"; gz topic -l 2>/dev/null | grep -iE "imu|odom|ground" | head
echo "--- ros2 topics ---"; ros2 topic list 2>/dev/null | grep -iE "imu|ground_truth"
echo "--- IMU sample ---"; timeout 8 ros2 topic echo /imu/data --once 2>/dev/null | head -6 || echo "NO_IMU"
echo "--- GT sample ---"; timeout 8 ros2 topic echo /aether/ground_truth --once 2>/dev/null | head -8 || echo "NO_GT"

kill -9 $GZ $BR 2>/dev/null; pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f "ruby" 2>/dev/null
echo "=== gz.log tail (world load) ==="; tail -8 /root/gz.log
echo "GAZEBO_TEST_DONE"
