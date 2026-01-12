"""
ZeusCar LiDAR Launch File

RPLIDAR A1M8を起動するlaunchファイル。
/scanトピックにLaserScanメッセージをパブリッシュする。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージのパス
    pkg_zeuscar_lidar = get_package_share_directory('zeuscar_lidar')

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='Serial port for RPLIDAR'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for laser scan'
    )

    # RPLIDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 115200,
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        rplidar_node,
    ])
