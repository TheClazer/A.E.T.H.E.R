#!/usr/bin/env bash
# Final demo-day sync: make /root/aether mirror the shipped repo exactly,
# purge stale installed launch files, rebuild, smoke the launcher.
set -e
source /opt/ros/lyrical/setup.bash
WS=/root/aether
SRC=/mnt/d/Work/AETHER

for d in src sim eval scripts offline_demo docs results docker; do
  rm -rf "$WS/$d"
  [ -e "$SRC/$d" ] && cp -r "$SRC/$d" "$WS/$d"
done
for f in docker-compose.yml README.md; do
  [ -f "$SRC/$f" ] && cp "$SRC/$f" "$WS/$f"
done
find "$WS" -path "$WS/install" -prune -o -path "$WS/build" -prune -o -type f \
  \( -name '*.sh' -o -name '*.py' -o -name '*.yaml' -o -name '*.yml' -o -name '*.sdf' \
     -o -name '*.config' -o -name '*.rviz' -o -name '*.md' -o -name '*.txt' -o -name '*.csv' \) \
  -exec sed -i 's/\r$//' {} + 2>/dev/null || true
chmod +x "$WS"/scripts/*.sh "$WS"/scripts/dev/*.sh 2>/dev/null || true

cd "$WS"
# purge the package whose share dir holds the deleted live3d.launch.py
rm -rf install/aether_bringup build/aether_bringup
source install/setup.bash 2>/dev/null || true
colcon build > /tmp/final_build.log 2>&1 || { tail -5 /tmp/final_build.log; exit 1; }
source install/setup.bash
echo SYNC_BUILD_OK

[ -f install/aether_bringup/share/aether_bringup/launch/live3d.launch.py ] \
  && echo "STALE_LAUNCH_PRESENT" || echo "STALE_LAUNCH_PURGED"
[ -f install/aether_bringup/share/aether_bringup/launch/live3d_stack.launch.py ] \
  && echo "STACK_LAUNCH_OK" || echo "STACK_LAUNCH_MISSING"
[ -f "$WS/results/drift_report.txt" ] && echo "RESULTS_OK" || echo "RESULTS_MISSING"
[ -f "$WS/docker-compose.yml" ] && echo "COMPOSE_OK" || echo "COMPOSE_MISSING"
bash scripts/judge_demo.sh help > /tmp/jd_help.txt 2>&1 || true
grep -q "live3d" /tmp/jd_help.txt && echo "LAUNCHER_OK" || echo "LAUNCHER_BROKEN"
echo FINAL_SYNC_DONE
