from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('teb_local_planner')
    rviz_config = os.path.join(pkg_share, 'cfg', 'teb_main_flow_viz.rviz')
    return LaunchDescription([
        Node(
            package='teb_local_planner',
            executable='teb_main_flow_viz_node',
            name='teb_main_flow_viz',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
