import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    pkg = get_package_share_directory('rover_sim')
    world = os.path.join(pkg, 'worlds', LaunchConfiguration('world').perform(context) + '.sdf')
    headless = LaunchConfiguration('headless').perform(context).lower() in ('1', 'true', 'yes')
    gz_args = f'-r -s --headless-rendering {world}' if headless else f'-r {world}'
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'), 'use_sim_time': True}])
    return [gz, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='gap_1p20', description='gap_0p70 | gap_0p80 | gap_0p90 | gap_1p20 | corridor'),
        DeclareLaunchArgument('headless', default_value='false', description='true = no GUI (server only)'),
        OpaqueFunction(function=_setup)])
