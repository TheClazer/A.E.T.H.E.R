# A.E.T.H.E.R GUARANTEED live demo: synthetic VIO replay -> the REAL integrity stack.
# No OpenVINS / no Gazebo needed. Launch this, open RViz, call /kill_camera, watch the bound breathe.
# Optional: hud:=true also starts the matplotlib mission HUD (mission_hud).
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('aether_bringup'), 'config', 'integrity.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('hud', default_value='false',
                              description='Also launch the matplotlib mission HUD'),
        Node(package='aether_sim_replay', executable='replay_node', name='aether_replay', output='screen'),
        Node(package='aether_integrity_monitor', executable='monitor_node', name='integrity_monitor',
             parameters=[cfg], output='screen'),
        Node(package='aether_degradation_manager', executable='manager_node', name='degradation_manager',
             output='screen'),
        Node(package='aether_health_cockpit', executable='cockpit_node', name='health_cockpit', output='screen'),
        Node(package='aether_health_cockpit', executable='mission_hud', name='mission_hud',
             output='screen', condition=IfCondition(LaunchConfiguration('hud'))),
    ])
