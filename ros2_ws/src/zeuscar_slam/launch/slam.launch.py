"""ZeusCar SLAM launchファイル.

slam_toolboxを使用したSLAM（マッピングモード）を起動する。

使用方法:
    ros2 launch zeuscar_slam slam.launch.py

引数:
    use_sim_time: シミュレーション時刻を使用するかどうか（デフォルト: false）

前提条件:
    - LiDARノードが起動済み（/scanトピックがパブリッシュされている）
    - TFツリーが正しく構成されている（base_footprint -> base_link -> laser_frame）
    - オドメトリが利用可能（/odomトピックまたはTF odom -> base_footprint）

注意:
    現時点ではオドメトリソースがないため、SLAMは動作しません。
    IMU（ICM42688）とホイールオドメトリの統合後に動作確認を行ってください。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launchファイルのエントリポイント."""
    # パッケージのパスを取得
    pkg_share = get_package_share_directory('zeuscar_slam')

    # 設定ファイルのパス
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # Launch引数
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # slam_toolboxノード（非同期オンラインモード）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_node,
    ])
