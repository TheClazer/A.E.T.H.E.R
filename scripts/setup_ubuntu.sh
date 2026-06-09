#!/usr/bin/env bash
# A.E.T.H.E.R one-shot environment setup for Ubuntu 22.04.
# Installs ROS2 Humble, Gazebo Harmonic + ros_gz, build deps, and OpenVINS.
# Read docs/AETHER_MANUAL_STEPS.pdf first — this automates Phase 1.
set -euo pipefail

echo "[1/5] ROS2 Humble ..."
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions

echo "[2/5] Gazebo Harmonic + ros_gz ..."
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic

echo "[3/5] OpenVINS build deps ..."
sudo apt install -y libeigen3-dev libboost-all-dev libopencv-dev libceres-dev

echo "[4/5] Python deps for the integrity layer + eval ..."
pip3 install -r "$(dirname "$0")/../requirements.txt"

echo "[5/5] OpenVINS (clone + build Release) ..."
mkdir -p ~/ws_ov/src && cd ~/ws_ov/src
[ -d open_vins ] || git clone https://github.com/rpng/open_vins.git
cd ~/ws_ov && source /opt/ros/humble/setup.bash
colcon build --packages-select ov_core ov_init ov_msckf ov_eval \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "DONE. Add to ~/.bashrc:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ws_ov/install/setup.bash"
echo "Then build A.E.T.H.E.R:  cd <repo> && colcon build --symlink-install"
echo "PX4 SITL is optional (Day 3) — see docs/AETHER_MANUAL_STEPS.pdf."
