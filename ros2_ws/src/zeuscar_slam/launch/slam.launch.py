"""ZeusCar SLAM launchファイル.

slam_toolboxを使用したSLAM（マッピングモード）を起動する。
slam_toolbox（ROS 2 Jazzy版）はライフサイクルノードのため、
起動後に configure → activate の遷移を行う。

使用方法:
    ros2 launch zeuscar_slam slam.launch.py

引数:
    use_sim_time: シミュレーション時刻を使用するかどうか（デフォルト: false）

前提条件:
    - LiDARノードが起動済み（/scanトピックがパブリッシュされている）
    - TFツリーが正しく構成されている（base_footprint -> base_link -> laser_frame）
    - オドメトリが利用可能（TF odom -> base_footprint）
"""

import os

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


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

    # slam_toolboxノード（ライフサイクルノード）
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # unconfigured → configure 遷移イベント
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # inactive → activate 遷移（configure成功後に自動実行）
    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: True,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_node,
        configure_event,
        activate_event_handler,
    ])
