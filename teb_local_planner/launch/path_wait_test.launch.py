from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('teb_local_planner')
    rviz_config = os.path.join(pkg_share, 'cfg', 'path_wait_test.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('path_wait_timeout', default_value='4.0'),
        DeclareLaunchArgument('num_dynamic', default_value='3'),
        DeclareLaunchArgument('num_static', default_value='2'),
        DeclareLaunchArgument('seed', default_value='1'),
        Node(
            package='teb_local_planner',
            executable='path_wait_test_node',
            name='path_wait_test_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_wait_timeout': ParameterValue(
                    LaunchConfiguration('path_wait_timeout'), value_type=float),
                'num_dynamic': ParameterValue(
                    LaunchConfiguration('num_dynamic'), value_type=int),
                'num_static': ParameterValue(
                    LaunchConfiguration('num_static'), value_type=int),
                'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
                'map_frame': 'map',
                'base_frame': 'base_link',
                'footprint_radius': 0.17,
                'min_obstacle_dist': 0.27,
                'path_wait_lookahead_dist': 6.0,
                'path_wait_corridor_scale': 1.0,
                'path_wait_clear_time': 0.5,
                'path_wait_timeout': 6.0,
                'spawn_x_min': 1.2,
                'spawn_x_max': 6.0,
                'spawn_y_min': -1.6,
                'spawn_y_max': 1.6,
                'speed_min': 0.05,
                'speed_max': 0.45,
                'resample_period': 8.0,
                'pulled_dev_threshold': 0.25,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
