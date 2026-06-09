#!/usr/bin/env bash
# Build + launch the A.E.T.H.E.R "floor": Gazebo tunnel + sensor_bridge + (OpenVINS).
# Run from the repo root on Ubuntu after scripts/setup_ubuntu.sh.
set -euo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash
[ -f ~/ws_ov/install/setup.bash ] && source ~/ws_ov/install/setup.bash

colcon build --symlink-install --packages-select aether_msgs
colcon build --symlink-install
source install/setup.bash

echo "Launching sim world + VIO floor (use_sim:=true)."
echo "Enable OpenVINS by uncommenting the ov_msckf node in vio.launch.py once built."
ros2 launch aether_bringup aether.launch.py use_sim:=true
