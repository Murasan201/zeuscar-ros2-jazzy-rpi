"""ZeusCar ベースシステム launchファイル.

robot_state_publisher（TF）とmotor_controller_node（駆動制御）を起動する。
最小構成での動作確認やモーター単独テストに使用する。

起動方法:
    ros2 launch zeuscar_bringup robot_base.launch.py
    ros2 launch zeuscar_bringup robot_base.launch.py use_motor:=false

パブリッシュされるTF:
    - base_footprint -> base_link
    - base_link -> laser_frame
    - base_link -> imu_link
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launchファイルのエントリポイント."""
    # パッケージパス取得
    description_pkg = get_package_share_directory('zeuscar_description')
    motor_pkg = get_package_share_directory('zeuscar_motor')

    # Launch引数の宣言
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='シミュレーション時刻の使用'
    )

    declare_use_motor = DeclareLaunchArgument(
        'use_motor',
        default_value='true',
        description='モーター起動フラグ'
    )

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arduinoシリアルポート'
    )

    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='シリアル通信速度'
    )

    # robot_state_publisher（TF基盤）
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # motor_controller_node（条件付き起動）
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motor_pkg, 'launch', 'motor.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_motor')),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_motor,
        declare_serial_port,
        declare_baud_rate,
        description_launch,
        motor_launch,
    ])
