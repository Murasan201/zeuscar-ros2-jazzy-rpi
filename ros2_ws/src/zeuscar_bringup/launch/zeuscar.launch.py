"""ZeusCar 統合メイン launchファイル.

全機能を統合起動するエントリポイント。
robot_base（TF + Motor）、sensors（LiDAR + IMU）、SLAM、RVizを
Launch Argumentsで柔軟に制御する。

起動方法:
    ros2 launch zeuscar_bringup zeuscar.launch.py
    ros2 launch zeuscar_bringup zeuscar.launch.py use_slam:=true
    ros2 launch zeuscar_bringup zeuscar.launch.py use_motor:=false use_slam:=false

Launch Arguments:
    use_sim_time: シミュレーション時刻の使用 (default: false)
    use_lidar: LiDAR起動フラグ (default: true)
    use_imu: IMU起動フラグ (default: true)
    use_motor: モーター起動フラグ (default: true)
    use_slam: SLAM起動フラグ (default: false)
    use_rviz: RViz自動起動フラグ (default: false)
    serial_port_motor: モーターシリアルポート (default: /dev/ttyACM0)
    serial_port_lidar: LiDARシリアルポート (default: /dev/rplidar)
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
    bringup_pkg = get_package_share_directory('zeuscar_bringup')
    slam_pkg = get_package_share_directory('zeuscar_slam')

    # Launch引数の宣言
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='シミュレーション時刻の使用'
    )

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

    declare_use_motor = DeclareLaunchArgument(
        'use_motor',
        default_value='true',
        description='モーター起動フラグ'
    )

    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='SLAM起動フラグ'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='RViz自動起動フラグ'
    )

    declare_serial_port_motor = DeclareLaunchArgument(
        'serial_port_motor',
        default_value='/dev/ttyACM0',
        description='モーターシリアルポート'
    )

    declare_serial_port_lidar = DeclareLaunchArgument(
        'serial_port_lidar',
        default_value='/dev/rplidar',
        description='LiDARシリアルポート'
    )

    # ロボットベース（TF + Motor）
    robot_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'robot_base.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_motor': LaunchConfiguration('use_motor'),
            'serial_port': LaunchConfiguration('serial_port_motor'),
        }.items(),
    )

    # センサー系（LiDAR + IMU）
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_imu': LaunchConfiguration('use_imu'),
            'serial_port_lidar': LaunchConfiguration('serial_port_lidar'),
        }.items(),
    )

    # SLAM（条件付き起動）
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam')),
    )

    # RViz（条件付き起動）
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'zeuscar.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_lidar,
        declare_use_imu,
        declare_use_motor,
        declare_use_slam,
        declare_use_rviz,
        declare_serial_port_motor,
        declare_serial_port_lidar,
        robot_base_launch,
        sensors_launch,
        slam_launch,
        rviz_node,
    ])
