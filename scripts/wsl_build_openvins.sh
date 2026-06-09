#!/usr/bin/env bash
# Build OpenVINS (ov_core/ov_init/ov_msckf/ov_eval) in WSL for ROS2 Lyrical.
# Memory-limited (7.6 GB box): one package at a time, 2 compile jobs each.
export DEBIAN_FRONTEND=noninteractive
source /opt/ros/lyrical/setup.bash

echo "[1/3] OpenVINS C++ deps"
apt-get install -y libeigen3-dev libboost-all-dev libopencv-dev libceres-dev \
  python3-matplotlib 2>&1 | tail -2

echo "[2/3] clone OpenVINS"
mkdir -p ~/ws_ov/src && cd ~/ws_ov/src
[ -d open_vins ] || git clone --depth 1 https://github.com/rpng/open_vins.git
cd ~/ws_ov

echo "[3/3] build (Release, parallel-workers=1, -j2)"
MAKEFLAGS="-j2" colcon build --packages-up-to ov_msckf ov_eval \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
rc=$?
echo "OPENVINS_BUILD_EXIT=$rc"
[ -f ~/ws_ov/install/setup.bash ] && ls ~/ws_ov/install | grep -E "ov_msckf|ov_eval" && echo "OPENVINS_INSTALLED"
