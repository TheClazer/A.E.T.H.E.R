# Bring up the VIO floor: sensor_bridge -> OpenVINS ov_msckf.
# OpenVINS is an external package built in ~/ws_ov (see docs/AETHER_MANUAL_STEPS.pdf).
# Topic contract: ov_msckf consumes /camera/left/image, /camera/right/image, /imu0.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    est = os.path.join(get_package_share_directory('aether_bringup'), 'config', 'estimator_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=est),
        Node(package='aether_sensor_bridge', executable='bridge_node', name='sensor_bridge',
             output='screen'),
        # OpenVINS MSCKF (external; built separately in ~/ws_ov). Uncomment once on the path:
        # Node(package='ov_msckf', executable='run_subscribe_msckf', name='ov_msckf',
        #      parameters=[{'config_path': LaunchConfiguration('config_path')}], output='screen'),
    ])
