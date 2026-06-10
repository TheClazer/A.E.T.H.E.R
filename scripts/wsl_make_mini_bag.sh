#!/usr/bin/env bash
# Extract a small, committable "mini" golden bag (ground truth + IMU only, no
# images) from the full tunnel_run bag — feeds the integrity-replay demo rung
# and CI without the 5-10 GB image payload.
set -e
source /opt/ros/lyrical/setup.bash
WS=~/aether
cd "$WS"
rm -rf bags/tunnel_mini
cat > /tmp/mini_convert.yaml <<'EOF'
output_bags:
  - uri: bags/tunnel_mini
    storage_id: mcap
    topics: [/aether/ground_truth, /imu/data]
EOF
ros2 bag convert -i bags/tunnel_run -o /tmp/mini_convert.yaml
ros2 bag info bags/tunnel_mini | grep -E "Bag size|Duration|Count"
rm -rf /mnt/d/Work/AETHER/bags/tunnel_mini
cp -r bags/tunnel_mini /mnt/d/Work/AETHER/bags/
echo MINI_BAG_DONE
