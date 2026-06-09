# Launch the A.E.T.H.E.R integrity layer (Python proxy path) — runs with or without sim.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('aether_bringup'), 'config', 'integrity.yaml')
    return LaunchDescription([
        Node(package='aether_sensor_bridge', executable='bridge_node', name='sensor_bridge',
             output='screen'),
        Node(package='aether_integrity_monitor', executable='monitor_node', name='integrity_monitor',
             parameters=[cfg], output='screen'),
        Node(package='aether_degradation_manager', executable='manager_node', name='degradation_manager',
             output='screen'),
        Node(package='aether_health_cockpit', executable='cockpit_node', name='health_cockpit',
             output='screen'),
    ])
