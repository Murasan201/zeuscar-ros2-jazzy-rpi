"""ICM-42688 IMUセンサー動作確認テストノード.

ロボットを短時間動かしながらIMUセンサーからデータを取得し、
加速度・ジャイロスコープの値をログ出力して動作確認を行う。
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from zeuscar_imu.icm42688_driver import ICM42688, SMBUS_AVAILABLE


class ImuTestNode(Node):
    """IMUセンサー動作確認テストノード."""

    # テストシーケンス（狭いスペース対応）
    TEST_SEQUENCE = [
        ('STOP', '静止状態（基準値取得）', 2.0),
        ('FORWARD', '前進テスト', 0.3),
        ('STOP', '停止（前進後）', 1.0),
        ('BACKWARD', '後退テスト', 0.3),
        ('STOP', '停止（後退後）', 1.0),
        ('TURNLEFT', '左旋回テスト', 0.3),
        ('STOP', '停止（左旋回後）', 1.0),
        ('TURNRIGHT', '右旋回テスト', 0.3),
        ('STOP', '停止（右旋回後）', 1.0),
    ]

    def __init__(self):
        """ノードの初期化."""
        super().__init__('imu_test_node')

        # パラメータの宣言
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('sample_rate_hz', 10.0)

        self._i2c_bus = self.get_parameter('i2c_bus').value
        self._i2c_address = self.get_parameter('i2c_address').value
        self._sample_rate = self.get_parameter('sample_rate_hz').value

        # モーターコマンドパブリッシャー
        self._motor_pub = self.create_publisher(String, '/zeuscar/motor_cmd', 10)

        # IMUセンサー初期化
        self._imu = ICM42688(bus=self._i2c_bus, address=self._i2c_address)
        self._imu_available = False

        self.get_logger().info('=== IMUセンサー動作確認テスト ===')
        self.get_logger().info(f'I2Cバス: {self._i2c_bus}, アドレス: 0x{self._i2c_address:02X}')

        # テスト開始用タイマー（1秒後に開始）
        self._test_timer = self.create_timer(1.0, self._run_test)
        self._test_started = False

    def _send_motor_command(self, command: str):
        """モーターコマンドを送信する."""
        msg = String()
        msg.data = command
        self._motor_pub.publish(msg)

    def _init_imu(self) -> bool:
        """IMUセンサーを初期化する."""
        if not SMBUS_AVAILABLE:
            self.get_logger().error('smbus2ライブラリがインストールされていません')
            self.get_logger().error('pip install smbus2 を実行してください')
            return False

        if not self._imu.open():
            self.get_logger().error(f'I2Cバス {self._i2c_bus} を開けません')
            self.get_logger().error('I2Cが有効になっているか確認してください')
            return False

        who_am_i = self._imu.get_who_am_i()
        self.get_logger().info(f'WHO_AM_I: 0x{who_am_i:02X} (期待値: 0x47)')

        if not self._imu.verify_device():
            self.get_logger().error('ICM-42688が検出されません')
            self.get_logger().error('配線を確認してください')
            return False

        self.get_logger().info('ICM-42688を検出しました')

        if not self._imu.initialize():
            self.get_logger().error('IMUの初期化に失敗しました')
            return False

        self.get_logger().info('IMUを初期化しました')
        return True

    def _log_imu_data(self, label: str):
        """IMUデータを取得してログ出力する."""
        data = self._imu.read_scaled_data()
        if data is None:
            self.get_logger().warn(f'{label}: データ取得失敗')
            return

        self.get_logger().info(
            f'{label}:\n'
            f'  加速度 [g]: X={data["accel_x"]:+.3f}, '
            f'Y={data["accel_y"]:+.3f}, Z={data["accel_z"]:+.3f}\n'
            f'  角速度 [dps]: X={data["gyro_x"]:+.2f}, '
            f'Y={data["gyro_y"]:+.2f}, Z={data["gyro_z"]:+.2f}\n'
            f'  温度: {data["temperature"]:.1f}℃'
        )

    def _sample_imu_during_motion(self, duration: float, label: str):
        """動作中にIMUデータをサンプリングする."""
        samples = []
        interval = 1.0 / self._sample_rate
        start_time = time.time()

        while (time.time() - start_time) < duration:
            data = self._imu.read_scaled_data()
            if data is not None:
                samples.append(data)
            time.sleep(interval)

        if samples:
            # 平均値を計算
            avg = {
                'accel_x': sum(s['accel_x'] for s in samples) / len(samples),
                'accel_y': sum(s['accel_y'] for s in samples) / len(samples),
                'accel_z': sum(s['accel_z'] for s in samples) / len(samples),
                'gyro_x': sum(s['gyro_x'] for s in samples) / len(samples),
                'gyro_y': sum(s['gyro_y'] for s in samples) / len(samples),
                'gyro_z': sum(s['gyro_z'] for s in samples) / len(samples),
            }
            self.get_logger().info(
                f'{label} (サンプル数: {len(samples)}):\n'
                f'  加速度平均 [g]: X={avg["accel_x"]:+.3f}, '
                f'Y={avg["accel_y"]:+.3f}, Z={avg["accel_z"]:+.3f}\n'
                f'  角速度平均 [dps]: X={avg["gyro_x"]:+.2f}, '
                f'Y={avg["gyro_y"]:+.2f}, Z={avg["gyro_z"]:+.2f}'
            )

    def _run_test(self):
        """テストを実行する."""
        if self._test_started:
            return
        self._test_started = True
        self._test_timer.cancel()

        # IMU初期化
        self._imu_available = self._init_imu()
        if not self._imu_available:
            self.get_logger().error('IMUが利用できないためテストを中止します')
            self._send_motor_command('STOP')
            raise SystemExit(1)

        self.get_logger().info('')
        self.get_logger().info('=== テスト開始 ===')
        self.get_logger().info('※ロボットが動きます。周囲に注意してください。')
        self.get_logger().info('')

        try:
            for command, description, duration in self.TEST_SEQUENCE:
                self.get_logger().info(f'--- {description} ({command}, {duration}秒) ---')

                # コマンド送信
                self._send_motor_command(command)

                # 動作中のIMUデータサンプリング
                self._sample_imu_during_motion(duration, description)

                self.get_logger().info('')

            self.get_logger().info('=== テスト完了 ===')
            self.get_logger().info('')
            self.get_logger().info('【確認ポイント】')
            self.get_logger().info('1. 静止状態: accel_z ≈ +1.0g（重力）、gyro ≈ 0')
            self.get_logger().info('2. 前進/後退: accel_x に変化が見られるか')
            self.get_logger().info('3. 左旋回: gyro_z > 0、右旋回: gyro_z < 0')

        except KeyboardInterrupt:
            self.get_logger().info('テストが中断されました')
        finally:
            self._send_motor_command('STOP')
            self._imu.close()
            self.get_logger().info('モーター停止、IMU接続クローズ')

        raise SystemExit(0)


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)
    node = ImuTestNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
