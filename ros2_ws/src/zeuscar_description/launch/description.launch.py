"""
ZeusCar description launchファイル

robot_state_publisherを起動し、URDFからTFをパブリッシュする。

起動方法:
    ros2 launch zeuscar_description description.launch.py

パブリッシュされるTF:
    - base_footprint → base_link
    - base_link → laser_frame
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launchファイルのエントリポイント."""
    pkg_share = get_package_share_directory('zeuscar_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'zeuscar.urdf.xacro')

    # Launch引数
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # robot_state_publisher: URDFをパースしてTFをパブリッシュ
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
    ])
