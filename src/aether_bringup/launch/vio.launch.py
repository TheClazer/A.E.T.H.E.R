# Bring up the VIO floor: sensor_bridge -> OpenVINS ov_msckf.
# OpenVINS is an external package built in ~/ws_ov (see docs/AETHER_MANUAL_STEPS.pdf).
# Default config is the SIM-EXACT trio in sim/openvins_config/ (estimator +
# kalibr imu/imucam chains, topics /camera/{left,right}/image_raw + /imu/data);
# pass config_path:=... to run against another calibration (e.g. EuRoC).
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = get_package_share_directory('aether_bringup')
    # workspace root holds sim/ (../../../.. from <ws>/install/aether_bringup/share/aether_bringup)
    repo = os.path.normpath(os.path.join(here, '..', '..', '..', '..'))
    est = os.path.join(repo, 'sim', 'openvins_config', 'estimator_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=est),
        Node(package='aether_sensor_bridge', executable='bridge_node', name='sensor_bridge',
             output='screen'),
        # OpenVINS MSCKF (external; built separately in ~/ws_ov). Uncomment once on the path:
        # Node(package='ov_msckf', executable='run_subscribe_msckf', name='ov_msckf',
        #      parameters=[{'config_path': LaunchConfiguration('config_path')}], output='screen'),
    ])
