"""IMUデータパブリッシュノード用launchファイル."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """LaunchDescriptionを生成する."""
    return LaunchDescription([
        Node(
            package='zeuscar_imu',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x68,
                'publish_rate_hz': 50.0,
                'frame_id': 'imu_link',
                'linear_acceleration_stdev': 0.05,
                'angular_velocity_stdev': 0.01,
            }],
        ),
    ])
