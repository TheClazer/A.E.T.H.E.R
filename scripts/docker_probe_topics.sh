#!/usr/bin/env bash
# Probe (inside aether/openvins:jazzy): which estimate topics does ov_msckf
# actually publish on this build, and at what rate, while the golden bag plays?
source /opt/ros/jazzy/setup.bash
source /ws_ov/install/setup.bash
CFG=/tmp/ovcfg
mkdir -p "$CFG"
for f in estimator_config kalibr_imu_chain kalibr_imucam_chain; do
  tr -d '\r' < "/aether/sim/openvins_config/$f.yaml" > "$CFG/$f.yaml"
done
ros2 run ov_msckf run_subscribe_msckf "$CFG/estimator_config.yaml" > /tmp/ov.log 2>&1 &
sleep 5
ros2 bag play /aether/bags/tunnel_run --playback-duration 30 > /dev/null 2>&1 &
sleep 22
echo "=== all topics ==="
ros2 topic list -t 2>/dev/null
echo "=== node pubs ==="
ros2 node info /ov_msckf 2>/dev/null | sed -n '/Publishers:/,/Service Servers:/p' | head -20
echo "=== try hz on candidates ==="
for t in /ov_msckf/odomimu /ov_msckf/poseimu /ov_msckf/pathimu; do
  echo "-- $t"; timeout 5 ros2 topic hz "$t" 2>&1 | head -2
done
echo PROBE_DONE
