#!/usr/bin/env bash
# A.E.T.H.E.R GUARANTEED live integrity demo (no OpenVINS / no Gazebo).
# Drives the REAL integrity_monitor + degradation_manager + health_cockpit nodes
# with a representative VIO trajectory, so the kill-camera beat always works on stage.
set -euo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select aether_msgs >/dev/null
colcon build --symlink-install >/dev/null
source install/setup.bash

echo "Launching replay + integrity stack. In other terminals:"
echo "  rviz2 -d src/aether_bringup/config/aether.rviz        # watch /viz/integrity_bound + /viz/truth"
echo "  ros2 topic echo /nav/trust    /nav/state               # trust 0-1 + NOMINAL/INERTIAL/..."
echo "  ./scripts/run_demo.sh kill                              # inject vision loss -> bound blooms, trust RED < 0.5 s"
echo "  ./scripts/run_demo.sh restore                           # recover"
ros2 launch aether_bringup replay.launch.py
