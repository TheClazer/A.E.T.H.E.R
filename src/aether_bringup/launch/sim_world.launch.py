# Launch the Gazebo Harmonic tunnel world + the ros_gz parameter bridge +
# the flight_director (closed-loop guidance -> the X3 multicopter stack).
# Ground truth is bridged to /aether/ground_truth via the OdometryPublisher
# plugin on the drone model.
#
# args:
#   gui:=true|false      Gazebo GUI client (true for the judge demo; false = -s headless)
#   patrol:=true|false   flight_director shuttles x=[5..90] forever instead of the
#                        one-way 150 s mission (true for the judge demo)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = get_package_share_directory('aether_bringup')
    # workspace root holds sim/ (../../../.. from <ws>/install/aether_bringup/share/aether_bringup)
    repo = os.path.normpath(os.path.join(here, '..', '..', '..', '..'))
    world = os.path.join(repo, 'sim', 'worlds', 'tunnel.sdf')
    models = os.path.join(repo, 'sim', 'models')

    gui = LaunchConfiguration('gui')
    patrol = LaunchConfiguration('patrol')

    # SERVER (physics + SENSOR RENDERING) always runs WITHOUT a display: under
    # WSLg, ogre-next's texture-upload thread crashes on the d3d12 GLX/EGL
    # device (TextureGpuManager::_waitFor) which silently kills the cameras —
    # while the display-less EGL-surfaceless path renders the textured world
    # reliably (it produced the 6,382-frame golden bag). The GUI viewer is a
    # separate client process that attaches to the running server.
    gz_server = ExecuteProcess(
        cmd=['bash', '-c',
             'unset DISPLAY WAYLAND_DISPLAY; exec gz sim -s -r "%s"' % world],
        name='gz_server', output='screen')
    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'], name='gz_gui', output='screen',
        condition=IfCondition(gui))

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/aether/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # ROS -> GZ (note ']' direction): velocity commands for the multicopter
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        parameters=[{'use_sim_time': True}], output='screen')

    flight = Node(
        package='aether_flight', executable='flight_director', name='flight_director',
        parameters=[{'use_sim_time': True, 'patrol': patrol}], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Gazebo GUI client (false = headless -s)'),
        DeclareLaunchArgument('patrol', default_value='false',
                              description='Shuttle the tunnel forever instead of one 150 s mission'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
        gz_server, gz_client, bridge, flight,
    ])
