# A.E.T.H.E.R live3d — the ROS-side stack ONLY (no Gazebo in here).
#
# Empirical finding on this rig: the gz server's ogre-next texture loader
# (TextureGpuManager) heap-crashes whenever gz runs as a `ros2 launch` child,
# while the IDENTICAL standalone invocation is solid (it recorded the golden
# bag). So scripts/judge_demo.sh starts the gz server itself — with a camera
# readiness gate and retries — and then launches THIS for everything else:
#
#   ros_gz bridge · sensor_bridge (/kill_camera gates the REAL images) ·
#   replay_node(source:=sim) riding the live ground truth + image stream ·
#   integrity stack · RViz · MISSION CONTROL
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('aether_bringup')
    cfg = os.path.join(share, 'config', 'integrity.yaml')
    rviz_cfg = os.path.join(share, 'config', 'aether.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('hud', default_value='true'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
             arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                 '/aether/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
             ],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='aether_flight', executable='flight_director', name='flight_director',
             parameters=[{'use_sim_time': True, 'patrol': True}], output='screen'),
        Node(package='aether_sensor_bridge', executable='bridge_node',
             name='sensor_bridge', output='screen'),
        Node(package='aether_sim_replay', executable='replay_node', name='aether_replay',
             parameters=[{'source': 'sim'}], output='screen'),
        Node(package='aether_integrity_monitor', executable='monitor_node',
             name='integrity_monitor', parameters=[cfg], output='screen'),
        Node(package='aether_degradation_manager', executable='manager_node',
             name='degradation_manager', output='screen'),
        Node(package='aether_health_cockpit', executable='cockpit_node',
             name='health_cockpit', output='screen'),
        Node(package='aether_health_cockpit', executable='mission_hud', name='mission_hud',
             parameters=[{'source_label': 'LIVE GAZEBO SIM · VIO-CLASS ERROR MODEL '
                                          '(real-OpenVINS numbers: results/drift_report.txt)'}],
             output='screen', condition=IfCondition(LaunchConfiguration('hud'))),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
