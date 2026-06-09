#!/usr/bin/env bash
# Install ROS2 (Lyrical, for Ubuntu 26.04 "resolute") + build deps, headless.
# Intended to run as root inside WSL. Idempotent-ish.
set -e
export DEBIAN_FRONTEND=noninteractive

echo "[1/4] base tools + locale"
apt-get update -y
apt-get install -y curl gnupg lsb-release locales ca-certificates software-properties-common
locale-gen en_US.UTF-8 || true
add-apt-repository universe -y || true

echo "[2/4] ROS2 apt repo (resolute)"
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu resolute main" \
  > /etc/apt/sources.list.d/ros2.list
apt-get update -y

echo "[3/4] install ROS2 Lyrical ros-base + build tools + deps"
apt-get install -y \
  ros-lyrical-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  ros-lyrical-sensor-msgs-py \
  python3-numpy \
  build-essential cmake git

echo "[4/4] verify"
. /opt/ros/lyrical/setup.sh
ros2 --help >/dev/null 2>&1 && echo "ros2 CLI OK"
python3 -c "import numpy; print('numpy', numpy.__version__)"
echo "ROS2_LYRICAL_INSTALL_DONE"
