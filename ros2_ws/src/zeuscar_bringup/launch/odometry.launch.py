"""ZeusCar オドメトリ launchファイル.

robot_localizationパッケージのEKFノードを起動し、
IMUデータからオドメトリ（odom → base_footprint TF）を生成する。

起動方法:
    ros2 launch zeuscar_bringup odometry.launch.py

パブリッシュされるトピック:
    - /odometry/filtered (nav_msgs/Odometry) - フィルタリング済みオドメトリ
    - /tf (odom → base_footprint) - オドメトリTF変換

前提条件:
    - IMUノードが起動済み（/imu/data_rawが配信されている）
    - TFツリーが構成済み（base_footprint → base_link → imu_link）

STORY-011: オドメトリ統合・SLAM動作確認
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Launchファイルのエントリポイント."""
    # パッケージパス取得
    bringup_pkg = get_package_share_directory('zeuscar_bringup')

    # EKF設定ファイルのパス
    ekf_params_file = os.path.join(bringup_pkg, 'config', 'ekf_params.yaml')

    # Launch引数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='シミュレーション時刻の使用'
    )

    # EKFノード（robot_localization）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node,
    ])
