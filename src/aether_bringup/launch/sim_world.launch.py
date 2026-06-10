# Launch the Gazebo Harmonic tunnel world + the ros_gz parameter bridge +
# the flight_director (open-loop velocity profile -> VelocityControl on the
# kinematic sensor rig). Ground truth is bridged to /aether/ground_truth via
# the OdometryPublisher plugin on the drone model.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    here = get_package_share_directory('aether_bringup')
    # workspace root holds sim/ (../../../.. from <ws>/install/aether_bringup/share/aether_bringup)
    repo = os.path.normpath(os.path.join(here, '..', '..', '..', '..'))
    world = os.path.join(repo, 'sim', 'worlds', 'tunnel.sdf')
    models = os.path.join(repo, 'sim', 'models')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items())

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/aether/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # ROS -> GZ (note ']' direction): velocity commands for the rig
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        parameters=[{'use_sim_time': True}], output='screen')

    flight = Node(
        package='aether_flight', executable='flight_director', name='flight_director',
        parameters=[{'use_sim_time': True}], output='screen')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
        gz, bridge, flight,
    ])
