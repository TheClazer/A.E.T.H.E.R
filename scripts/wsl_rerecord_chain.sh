#!/usr/bin/env bash
# Full retry loop after the weave-crash + soft-punch fixes:
#   sync sources -> rebuild flight pkg -> re-record golden bag (150 s profile)
#   -> sanity-check GT (no crash, full corridor) -> copy bag to /mnt/d
#   -> run the OpenVINS chain in docker -> report drift.
set -o pipefail
source /opt/ros/lyrical/setup.bash
WS=~/aether

echo "=== [1/5] sync + rebuild ==="
cp /mnt/d/Work/AETHER/src/aether_flight/aether_flight/flight_director.py "$WS/src/aether_flight/aether_flight/"
mkdir -p "$WS/sim/openvins_config"
cp /mnt/d/Work/AETHER/sim/openvins_config/*.yaml "$WS/sim/openvins_config/"
cp /mnt/d/Work/AETHER/scripts/wsl_record_bag.sh "$WS/scripts/"
sed -i 's/\r$//' "$WS/src/aether_flight/aether_flight/flight_director.py" "$WS/sim/openvins_config/"*.yaml "$WS/scripts/wsl_record_bag.sh"
cd "$WS" && source install/setup.bash
colcon build --packages-select aether_flight > /dev/null 2>&1 && echo REBUILT

echo "=== [2/5] re-record (150 s profile) ==="
RECORD_SECS=150 bash scripts/wsl_record_bag.sh > /tmp/rec3.log 2>&1
grep -E "Duration|image_raw \| .*Count" /tmp/rec3.log | head -3

echo "=== [3/5] GT sanity (crash check) ==="
python3 eval/odom_to_tum.py bags/tunnel_run /aether/ground_truth /tmp/g3.txt > /dev/null 2>&1
python3 - <<'EOF'
import numpy as np, sys
d = np.loadtxt('/tmp/g3.txt'); t = d[:,0]-d[0,0]
x,y,z = d[:,1],d[:,2],d[:,3]
print('GT %.1f s | x %.1f..%.1f | y %.2f..%.2f | z %.2f..%.2f' %
      (t[-1], x.min(), x.max(), y.min(), y.max(), z.min(), z.max()))
# crash heuristics: y must stay inside +-1.2, z must not collapse after climb
i_cl = np.searchsorted(t, 16.0)
ok = (abs(y).max() < 1.2) and (z[i_cl:].min() > 0.25) and (x.max() > 150)
print('FLIGHT_OK' if ok else 'FLIGHT_BAD')
sys.exit(0 if ok else 1)
EOF
[ $? -eq 0 ] || { echo "ABORT: flight failed sanity"; exit 1; }

echo "=== [4/5] copy bag to mount ==="
rm -rf /mnt/d/Work/AETHER/bags/tunnel_run
cp -r "$WS/bags/tunnel_run" /mnt/d/Work/AETHER/bags/ && echo BAG_ON_MOUNT

echo "=== [5/5] docker chain ==="
docker run --rm -v /mnt/d/Work/AETHER:/aether aether/openvins:jazzy \
  bash -c 'tr -d "\r" < /aether/scripts/docker_run_chain.sh > /tmp/chain.sh && PLAY_SECS=165 bash /tmp/chain.sh' 2>&1 | tail -25
echo "RERECORD_CHAIN_DONE"
