# Top-level A.E.T.H.E.R launch. use_sim:=true -> live Gazebo; false -> rosbag/EuRoC replay.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('aether_bringup')
    use_sim = LaunchConfiguration('use_sim')

    def inc(name, cond=None):
        kw = {'condition': cond} if cond is not None else {}
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)), **kw)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        inc('sim_world.launch.py', IfCondition(use_sim)),
        inc('vio.launch.py'),
        inc('integrity.launch.py'),
    ])
