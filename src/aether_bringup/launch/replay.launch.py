# A.E.T.H.E.R GUARANTEED live demo: synthetic VIO replay -> the REAL integrity stack.
# No OpenVINS / no Gazebo needed. Launch this, open RViz, call /kill_camera, watch the bound breathe.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('aether_bringup'), 'config', 'integrity.yaml')
    return LaunchDescription([
        Node(package='aether_sim_replay', executable='replay_node', name='aether_replay', output='screen'),
        Node(package='aether_integrity_monitor', executable='monitor_node', name='integrity_monitor',
             parameters=[cfg], output='screen'),
        Node(package='aether_degradation_manager', executable='manager_node', name='degradation_manager',
             output='screen'),
        Node(package='aether_health_cockpit', executable='cockpit_node', name='health_cockpit', output='screen'),
    ])
