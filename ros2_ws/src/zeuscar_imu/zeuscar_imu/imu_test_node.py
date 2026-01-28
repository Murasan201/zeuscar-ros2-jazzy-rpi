"""ICM-42688 IMUセンサー動作確認テストノード.

ロボットを短時間動かしながらIMUセンサーからデータを取得し、
加速度・ジャイロスコープの値をログ出力して動作確認を行う。
"""
import time
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


class ICM42688:
    """ICM-42688 IMUセンサードライバ.

    Attributes:
        i2c_bus: I2Cバス番号
        i2c_address: I2Cアドレス
    """

    # レジスタアドレス
    REG_WHO_AM_I = 0x75
    REG_PWR_MGMT0 = 0x4E
    REG_GYRO_CONFIG0 = 0x4F
    REG_ACCEL_CONFIG0 = 0x50
    REG_TEMP_DATA1 = 0x1D
    REG_TEMP_DATA0 = 0x1E
    REG_ACCEL_DATA_X1 = 0x1F
    REG_GYRO_DATA_X1 = 0x25

    # 期待されるWHO_AM_I値
    WHO_AM_I_VALUE = 0x47

    # スケールファクター（デフォルト設定）
    ACCEL_SCALE = 16384.0  # ±2g設定時: 16384 LSB/g
    GYRO_SCALE = 131.0     # ±250dps設定時: 131 LSB/dps

    def __init__(self, bus: int = 1, address: int = 0x68):
        """初期化.

        Args:
            bus: I2Cバス番号（Raspberry Pi 4では通常1）
            address: I2Cアドレス（SAO=GND: 0x68, SAO=VCC: 0x69）
        """
        self.i2c_bus = bus
        self.i2c_address = address
        self._bus = None

    def open(self) -> bool:
        """I2C接続を開く.

        Returns:
            接続成功の場合True
        """
        if not SMBUS_AVAILABLE:
            return False

        try:
            self._bus = smbus2.SMBus(self.i2c_bus)
            return True
        except OSError:
            return False

    def close(self):
        """I2C接続を閉じる."""
        if self._bus is not None:
            self._bus.close()
            self._bus = None

    def verify_device(self) -> bool:
        """デバイスIDを確認する.

        Returns:
            ICM-42688が検出された場合True
        """
        if self._bus is None:
            return False

        try:
            who_am_i = self._bus.read_byte_data(self.i2c_address, self.REG_WHO_AM_I)
            return who_am_i == self.WHO_AM_I_VALUE
        except OSError:
            return False

    def get_who_am_i(self) -> int:
        """WHO_AM_Iレジスタの値を取得する.

        Returns:
            WHO_AM_Iの値（エラー時は-1）
        """
        if self._bus is None:
            return -1

        try:
            return self._bus.read_byte_data(self.i2c_address, self.REG_WHO_AM_I)
        except OSError:
            return -1

    def initialize(self) -> bool:
        """センサーを初期化する.

        Returns:
            初期化成功の場合True
        """
        if self._bus is None:
            return False

        try:
            # 電源管理: ジャイロとアクセルを低ノイズモードで有効化
            # GYRO_MODE = 11 (Low Noise), ACCEL_MODE = 11 (Low Noise)
            self._bus.write_byte_data(self.i2c_address, self.REG_PWR_MGMT0, 0x0F)
            time.sleep(0.05)  # 起動待ち

            # ジャイロ設定: ±250dps, ODR=1kHz
            self._bus.write_byte_data(self.i2c_address, self.REG_GYRO_CONFIG0, 0x06)

            # 加速度設定: ±2g, ODR=1kHz
            self._bus.write_byte_data(self.i2c_address, self.REG_ACCEL_CONFIG0, 0x06)

            time.sleep(0.05)
            return True
        except OSError:
            return False

    def read_raw_data(self) -> dict | None:
        """生データを読み取る.

        Returns:
            センサーデータの辞書、エラー時はNone
        """
        if self._bus is None:
            return None

        try:
            # 加速度データ読み取り（6バイト: X, Y, Z 各2バイト）
            accel_data = self._bus.read_i2c_block_data(
                self.i2c_address, self.REG_ACCEL_DATA_X1, 6
            )

            # ジャイロデータ読み取り（6バイト: X, Y, Z 各2バイト）
            gyro_data = self._bus.read_i2c_block_data(
                self.i2c_address, self.REG_GYRO_DATA_X1, 6
            )

            # 温度データ読み取り（2バイト）
            temp_data = self._bus.read_i2c_block_data(
                self.i2c_address, self.REG_TEMP_DATA1, 2
            )

            # 16bit符号付き整数に変換（ビッグエンディアン）
            accel_x = struct.unpack('>h', bytes(accel_data[0:2]))[0]
            accel_y = struct.unpack('>h', bytes(accel_data[2:4]))[0]
            accel_z = struct.unpack('>h', bytes(accel_data[4:6]))[0]

            gyro_x = struct.unpack('>h', bytes(gyro_data[0:2]))[0]
            gyro_y = struct.unpack('>h', bytes(gyro_data[2:4]))[0]
            gyro_z = struct.unpack('>h', bytes(gyro_data[4:6]))[0]

            temp_raw = struct.unpack('>h', bytes(temp_data[0:2]))[0]

            return {
                'accel_x_raw': accel_x,
                'accel_y_raw': accel_y,
                'accel_z_raw': accel_z,
                'gyro_x_raw': gyro_x,
                'gyro_y_raw': gyro_y,
                'gyro_z_raw': gyro_z,
                'temp_raw': temp_raw,
            }
        except OSError:
            return None

    def read_scaled_data(self) -> dict | None:
        """スケール変換済みデータを読み取る.

        Returns:
            変換済みセンサーデータの辞書、エラー時はNone
            - accel_x/y/z: 加速度 (g)
            - gyro_x/y/z: 角速度 (dps)
            - temperature: 温度 (℃)
        """
        raw = self.read_raw_data()
        if raw is None:
            return None

        # 温度計算: Temperature (°C) = (TEMP_DATA / 132.48) + 25
        temperature = (raw['temp_raw'] / 132.48) + 25.0

        return {
            'accel_x': raw['accel_x_raw'] / self.ACCEL_SCALE,
            'accel_y': raw['accel_y_raw'] / self.ACCEL_SCALE,
            'accel_z': raw['accel_z_raw'] / self.ACCEL_SCALE,
            'gyro_x': raw['gyro_x_raw'] / self.GYRO_SCALE,
            'gyro_y': raw['gyro_y_raw'] / self.GYRO_SCALE,
            'gyro_z': raw['gyro_z_raw'] / self.GYRO_SCALE,
            'temperature': temperature,
        }


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
