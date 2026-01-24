"""
ZeusCar モーターコントローラ launchファイル

motor_controller_nodeを起動し、Arduinoとの通信を開始する。

起動方法:
    ros2 launch zeuscar_motor motor.launch.py
    ros2 launch zeuscar_motor motor.launch.py serial_port:=/dev/arduino
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launchファイルのエントリポイント."""
    pkg_share = get_package_share_directory('zeuscar_motor')

    # パラメータファイル
    config_file = os.path.join(pkg_share, 'config', 'motor_params.yaml')

    # Launch引数の宣言
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Serial baud rate'
    )

    # motor_controller_node
    motor_controller_node = Node(
        package='zeuscar_motor',
        executable='motor_controller_node',
        name='motor_controller_node',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        motor_controller_node,
    ])
