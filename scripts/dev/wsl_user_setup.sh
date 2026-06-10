#!/usr/bin/env bash
# Set up the REGULAR user's demo workspace (GUI apps live best outside root under
# WSLg) and retest rviz2 + the Gazebo GUI with correct paths.
# Run as: the regular user (the rviz2 apt install is done separately as root).
source /opt/ros/lyrical/setup.bash
set -o pipefail

WS=~/aether
echo "=== [1/3] sync workspace from /mnt/d ==="
mkdir -p "$WS"
for d in src sim eval scripts offline_demo bags/tunnel_mini; do
  rm -rf "$WS/${d}"
  mkdir -p "$WS/$(dirname $d)"
  cp -r "/mnt/d/Work/AETHER/$d" "$WS/${d}" 2>/dev/null
done
cp /mnt/d/Work/AETHER/requirements.txt "$WS/" 2>/dev/null
find "$WS" -type f \( -name '*.py' -o -name '*.sh' -o -name '*.yaml' -o -name '*.xml' \
  -o -name '*.msg' -o -name '*.srv' -o -name '*.sdf' -o -name '*.config' -o -name '*.rviz' \
  -o -name 'CMakeLists.txt' \) -exec sed -i 's/\r$//' {} +
cd "$WS"

echo "=== [2/3] build (user-owned) ==="
colcon build --packages-select aether_msgs > /tmp/ub1.log 2>&1 || { tail -3 /tmp/ub1.log; exit 1; }
source install/setup.bash
colcon build > /tmp/ub2.log 2>&1 || { tail -5 /tmp/ub2.log; exit 1; }
source install/setup.bash
echo "USER_WS_BUILT"

echo "=== [3/3] GUI retest ==="
timeout 8 rviz2 -d "$WS/src/aether_bringup/config/aether.rviz" > /tmp/rviz2.log 2>&1
RC=$?
[ "$RC" = "124" ] && echo "RVIZ_OK" || { echo "RVIZ_RC=$RC"; tail -3 /tmp/rviz2.log; }

export GZ_SIM_RESOURCE_PATH="$WS/sim/models"
timeout 15 gz sim -r "$WS/sim/worlds/tunnel.sdf" > /tmp/gzgui.log 2>&1
RC=$?
[ "$RC" = "124" ] && echo "GZ_GUI_OK" || { echo "GZ_GUI_RC=$RC"; grep -iE "error|display|fail" /tmp/gzgui.log | head -5; }
pkill -9 -f "gz sim" 2>/dev/null
echo "USER_SETUP_DONE"
