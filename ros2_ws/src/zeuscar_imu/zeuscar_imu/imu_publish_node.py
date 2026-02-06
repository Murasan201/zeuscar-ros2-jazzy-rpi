"""IMUデータパブリッシュノード.

ICM-42688 IMUセンサーから取得した加速度・角速度データを
sensor_msgs/msg/Imu メッセージとしてパブリッシュする。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

from zeuscar_imu.icm42688_driver import ICM42688, SMBUS_AVAILABLE

# 定数
GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0


def dps_to_rad_s(dps: float) -> float:
    """角速度をdpsからrad/sに変換する."""
    return dps * DEG_TO_RAD


def g_to_m_s2(g: float) -> float:
    """加速度をgからm/s²に変換する."""
    return g * GRAVITY


def create_imu_msg(
    scaled_data: dict,
    frame_id: str = 'imu_link',
    angular_velocity_stdev: float = 0.01,
    linear_acceleration_stdev: float = 0.05,
    stamp=None,
):
    """IMUデータからImuメッセージ相当の辞書を構築する.

    Args:
        scaled_data: ICM42688.read_scaled_data()の戻り値
        frame_id: IMUフレームID
        angular_velocity_stdev: 角速度標準偏差 (rad/s)
        linear_acceleration_stdev: 加速度標準偏差 (m/s²)
        stamp: タイムスタンプ（Noneの場合は設定しない）
    """
    angular_var = angular_velocity_stdev ** 2
    linear_var = linear_acceleration_stdev ** 2

    return {
        'frame_id': frame_id,
        'orientation': [0.0, 0.0, 0.0, 0.0],
        'orientation_covariance': [-1.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0],
        'angular_velocity': {
            'x': dps_to_rad_s(scaled_data['gyro_x']),
            'y': dps_to_rad_s(scaled_data['gyro_y']),
            'z': dps_to_rad_s(scaled_data['gyro_z']),
        },
        'angular_velocity_covariance': [
            angular_var, 0.0, 0.0,
            0.0, angular_var, 0.0,
            0.0, 0.0, angular_var,
        ],
        'linear_acceleration': {
            'x': g_to_m_s2(scaled_data['accel_x']),
            'y': g_to_m_s2(scaled_data['accel_y']),
            'z': g_to_m_s2(scaled_data['accel_z']),
        },
        'linear_acceleration_covariance': [
            linear_var, 0.0, 0.0,
            0.0, linear_var, 0.0,
            0.0, 0.0, linear_var,
        ],
    }


def _dict_to_imu_msg(msg_dict: dict, stamp=None) -> Imu:
    """辞書形式のメッセージデータをsensor_msgs/msg/Imuに変換する."""
    msg = Imu()

    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = msg_dict['frame_id']

    # orientation（6軸IMUでは利用不可）
    msg.orientation.x = msg_dict['orientation'][0]
    msg.orientation.y = msg_dict['orientation'][1]
    msg.orientation.z = msg_dict['orientation'][2]
    msg.orientation.w = msg_dict['orientation'][3]
    msg.orientation_covariance = msg_dict['orientation_covariance']

    # angular_velocity
    msg.angular_velocity.x = msg_dict['angular_velocity']['x']
    msg.angular_velocity.y = msg_dict['angular_velocity']['y']
    msg.angular_velocity.z = msg_dict['angular_velocity']['z']
    msg.angular_velocity_covariance = msg_dict['angular_velocity_covariance']

    # linear_acceleration
    msg.linear_acceleration.x = msg_dict['linear_acceleration']['x']
    msg.linear_acceleration.y = msg_dict['linear_acceleration']['y']
    msg.linear_acceleration.z = msg_dict['linear_acceleration']['z']
    msg.linear_acceleration_covariance = msg_dict['linear_acceleration_covariance']

    return msg


class ImuPublishNode(Node):
    """IMUデータパブリッシュノード."""

    MAX_CONSECUTIVE_FAILURES = 5

    def __init__(self):
        """ノードの初期化."""
        super().__init__('imu_node')

        # パラメータの宣言
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('linear_acceleration_stdev', 0.05)
        self.declare_parameter('angular_velocity_stdev', 0.01)

        self._i2c_bus = self.get_parameter('i2c_bus').value
        self._i2c_address = self.get_parameter('i2c_address').value
        self._publish_rate = self.get_parameter('publish_rate_hz').value
        self._frame_id = self.get_parameter('frame_id').value
        self._linear_acceleration_stdev = self.get_parameter(
            'linear_acceleration_stdev'
        ).value
        self._angular_velocity_stdev = self.get_parameter(
            'angular_velocity_stdev'
        ).value

        # エラーカウンタ
        self._error_count = 0

        # IMUセンサー初期化
        self._imu = ICM42688(bus=self._i2c_bus, address=self._i2c_address)
        if not self._init_imu():
            self.get_logger().error('IMU初期化に失敗しました。ノードを終了します。')
            raise SystemExit(1)

        # QoS設定（SensorDataQoS相当）
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # パブリッシャー作成
        self._imu_pub = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)

        # タイマー作成
        timer_period = 1.0 / self._publish_rate
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'IMUパブリッシュノード開始: {self._publish_rate} Hz, '
            f'frame_id={self._frame_id}'
        )

    def _init_imu(self) -> bool:
        """IMUセンサーを初期化する."""
        if not SMBUS_AVAILABLE:
            self.get_logger().error('smbus2ライブラリがインストールされていません')
            return False

        if not self._imu.open():
            self.get_logger().error(f'I2Cバス {self._i2c_bus} を開けません')
            return False

        if not self._imu.verify_device():
            self.get_logger().error('ICM-42688が検出されません')
            self._imu.close()
            return False

        if not self._imu.initialize():
            self.get_logger().error('IMUの初期化に失敗しました')
            self._imu.close()
            return False

        self.get_logger().info('ICM-42688を初期化しました')
        return True

    def _timer_callback(self):
        """タイマーコールバック: IMUデータを読み取りパブリッシュする."""
        data = self._imu.read_scaled_data()

        if data is None:
            self._error_count += 1
            self.get_logger().warn(
                f'IMUデータ読み取り失敗 ({self._error_count}回連続)'
            )
            if self._error_count >= self.MAX_CONSECUTIVE_FAILURES:
                self.get_logger().error(
                    f'{self.MAX_CONSECUTIVE_FAILURES}回連続で読み取り失敗。'
                    'IMUを再初期化します。'
                )
                self._imu.initialize()
                self._error_count = 0
            return

        self._error_count = 0

        # メッセージ構築
        stamp = self.get_clock().now().to_msg()
        msg_dict = create_imu_msg(
            scaled_data=data,
            frame_id=self._frame_id,
            angular_velocity_stdev=self._angular_velocity_stdev,
            linear_acceleration_stdev=self._linear_acceleration_stdev,
        )
        imu_msg = _dict_to_imu_msg(msg_dict, stamp=stamp)

        self._imu_pub.publish(imu_msg)

    def destroy_node(self):
        """ノード終了処理."""
        self._imu.close()
        self.get_logger().info('IMU接続をクローズしました')
        super().destroy_node()


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)
    node = ImuPublishNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
