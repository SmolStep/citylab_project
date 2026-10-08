from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_patrol')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'citylab.rviz')

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='patrol_executable',
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
    