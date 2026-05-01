"""Launch bbs3d_ros2_node alongside RViz2 with the demo config."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bbs3d_ros2')
    default_config = os.path.join(pkg_share, 'config', 'bbs3d_ros2.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'bbs3d.rviz')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the bbs3d_ros2 yaml config file.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Path to the rviz2 .rviz config file.',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    bbs3d_node = Node(
        package='bbs3d_ros2',
        executable='bbs3d_ros2_node',
        output='screen',
        parameters=[{'config': LaunchConfiguration('config_file')}],
    )

    return LaunchDescription([
        config_file_arg,
        rviz_config_arg,
        rviz_node,
        bbs3d_node,
    ])
