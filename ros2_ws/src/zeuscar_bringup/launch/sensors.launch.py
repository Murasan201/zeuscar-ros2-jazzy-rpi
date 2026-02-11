"""ZeusCar センサー系 launchファイル.

LiDAR（RPLIDAR A1M8）とIMU（ICM-42688）を起動する。
センサー動作確認やキャリブレーションに使用する。

起動方法:
    ros2 launch zeuscar_bringup sensors.launch.py
    ros2 launch zeuscar_bringup sensors.launch.py use_imu:=false

パブリッシュされるトピック:
    - /scan (sensor_msgs/LaserScan) - LiDAR有効時
    - /imu/data_raw (sensor_msgs/Imu) - IMU有効時
    - /imu/data (sensor_msgs/Imu) - IMU有効時（madgwickフィルタ済み）
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launchファイルのエントリポイント."""
    # パッケージパス取得
    lidar_pkg = get_package_share_directory('zeuscar_lidar')
    imu_pkg = get_package_share_directory('zeuscar_imu')
    bringup_pkg = get_package_share_directory('zeuscar_bringup')

    # IMUフィルタ設定ファイルパス
    imu_filter_params_file = os.path.join(
        bringup_pkg, 'config', 'imu_filter_params.yaml'
    )

    # Launch引数の宣言
    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='LiDAR起動フラグ'
    )

    declare_use_imu = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='IMU起動フラグ'
    )

    declare_serial_port_lidar = DeclareLaunchArgument(
        'serial_port_lidar',
        default_value='/dev/rplidar',
        description='LiDARシリアルポート'
    )

    # LiDARノード（条件付き起動）
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port_lidar'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    # IMUノード（条件付き起動）
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg, 'launch', 'imu.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_imu')),
    )

    # IMUフィルタノード（madgwick）
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[imu_filter_params_file],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data', '/imu/data'),
        ],
        condition=IfCondition(LaunchConfiguration('use_imu')),
    )

    return LaunchDescription([
        declare_use_lidar,
        declare_use_imu,
        declare_serial_port_lidar,
        lidar_launch,
        imu_launch,
        imu_filter_node,
    ])
