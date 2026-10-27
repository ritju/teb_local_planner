from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('teb_local_planner')
    rviz_config = os.path.join(pkg_share, 'cfg', 'front_steering_viz.rviz')
    return LaunchDescription([
        Node(
            package='teb_local_planner',
            executable='front_steering_viz_node',
            name='front_steering_viz',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='front_steering_rqt_plot',
            output='screen',
            arguments=[
                '/front_steering/steering_angular_velocity/data',
                '/front_steering/steering_angular_acceleration/data',
            ],
        ),
    ])
