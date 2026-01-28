"""IMUテストノード用launchファイル."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """LaunchDescriptionを生成する."""
    return LaunchDescription([
        Node(
            package='zeuscar_imu',
            executable='imu_test_node',
            name='imu_test_node',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x68,
                'sample_rate_hz': 10.0,
            }],
        ),
    ])
