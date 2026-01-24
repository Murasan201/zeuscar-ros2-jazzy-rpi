"""
ZeusCar モーターコントローラノード

Raspberry PiからArduinoへシリアル通信でモーターコマンドを送信する。
/cmd_vel (geometry_msgs/Twist) および /zeuscar/motor_cmd (std_msgs/String) を購読。
"""
import serial
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MotorControllerNode(Node):
    """Arduinoモーターコントローラとの通信を管理するノード."""

    # 有効なコマンド一覧
    VALID_COMMANDS = [
        'FORWARD', 'BACKWARD', 'LEFT', 'RIGHT',
        'LEFTFORWARD', 'RIGHTFORWARD', 'LEFTBACKWARD', 'RIGHTBACKWARD',
        'TURNLEFT', 'TURNRIGHT', 'STOP'
    ]

    def __init__(self):
        """ノードの初期化."""
        super().__init__('motor_controller_node')

        # パラメータの宣言
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('linear_threshold', 0.1)
        self.declare_parameter('angular_threshold', 0.1)

        # パラメータの取得
        self._serial_port = self.get_parameter('serial_port').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self._linear_threshold = self.get_parameter('linear_threshold').value
        self._angular_threshold = self.get_parameter('angular_threshold').value

        # シリアルポート初期化
        self._serial = None
        self._init_serial()

        # QoS設定
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        # /zeuscar/motor_cmd サブスクライバ（直接コマンド）
        self._motor_cmd_sub = self.create_subscription(
            String,
            'zeuscar/motor_cmd',
            self._motor_cmd_callback,
            qos
        )

        # /cmd_vel サブスクライバ（Twist形式）
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            qos
        )

        # タイムアウト用タイマー
        self._last_cmd_time = self.get_clock().now()
        self._timer = self.create_timer(0.1, self._timeout_callback)

        # 最後に送信したコマンド
        self._last_command = 'STOP'

        self.get_logger().info(
            f'MotorControllerNode started. Serial: {self._serial_port} @ {self._baud_rate}bps'
        )

    def _init_serial(self):
        """シリアルポートを初期化する."""
        try:
            self._serial = serial.Serial(
                self._serial_port,
                self._baud_rate,
                timeout=1.0
            )
            # Arduino起動待ち
            time.sleep(2.0)
            self.get_logger().info(
                f'Serial connection established: {self._serial_port}'
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self._serial = None

    def _send_command(self, command: str) -> bool:
        """Arduinoにコマンドを送信する.

        Args:
            command: 送信するコマンド文字列

        Returns:
            送信成功の場合True
        """
        if command not in self.VALID_COMMANDS:
            self.get_logger().warning(f'Invalid command: {command}')
            return False

        if self._serial is None or not self._serial.is_open:
            self.get_logger().warning('Serial port is not open')
            return False

        # 同じコマンドの連続送信を抑制
        if command == self._last_command and command != 'STOP':
            return True

        try:
            # コマンド送信（改行付き）
            self._serial.write(f'{command}\n'.encode('ascii'))
            self._last_command = command
            self.get_logger().debug(f'Sent command: {command}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            return False

    def _motor_cmd_callback(self, msg: String):
        """直接コマンドトピックのコールバック.

        Args:
            msg: コマンド文字列メッセージ
        """
        command = msg.data.strip().upper()
        self._last_cmd_time = self.get_clock().now()

        if self._send_command(command):
            self.get_logger().info(f'Motor command: {command}')

    def _cmd_vel_callback(self, msg: Twist):
        """Twistメッセージからコマンドを生成して送信する.

        Args:
            msg: 速度指令メッセージ
        """
        self._last_cmd_time = self.get_clock().now()
        command = self._twist_to_command(msg)

        if self._send_command(command):
            self.get_logger().debug(
                f'cmd_vel -> {command} '
                f'(linear: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, '
                f'angular: z={msg.angular.z:.2f})'
            )

    def _twist_to_command(self, msg: Twist) -> str:
        """Twistメッセージをコマンド文字列に変換する.

        Args:
            msg: Twistメッセージ

        Returns:
            対応するコマンド文字列
        """
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # 閾値以下は0として扱う
        if abs(linear_x) < self._linear_threshold:
            linear_x = 0.0
        if abs(linear_y) < self._linear_threshold:
            linear_y = 0.0
        if abs(angular_z) < self._angular_threshold:
            angular_z = 0.0

        # 旋回が優先
        if angular_z > 0:
            return 'TURNLEFT'
        elif angular_z < 0:
            return 'TURNRIGHT'

        # 直進・後退
        if linear_x > 0 and linear_y == 0:
            return 'FORWARD'
        elif linear_x < 0 and linear_y == 0:
            return 'BACKWARD'

        # 横移動
        if linear_x == 0 and linear_y > 0:
            return 'LEFT'
        elif linear_x == 0 and linear_y < 0:
            return 'RIGHT'

        # 斜め移動
        if linear_x > 0 and linear_y > 0:
            return 'LEFTFORWARD'
        elif linear_x > 0 and linear_y < 0:
            return 'RIGHTFORWARD'
        elif linear_x < 0 and linear_y > 0:
            return 'LEFTBACKWARD'
        elif linear_x < 0 and linear_y < 0:
            return 'RIGHTBACKWARD'

        # すべて0の場合
        return 'STOP'

    def _timeout_callback(self):
        """タイムアウト時にSTOPコマンドを送信する."""
        now = self.get_clock().now()
        elapsed = (now - self._last_cmd_time).nanoseconds / 1e9

        if elapsed > self._cmd_vel_timeout and self._last_command != 'STOP':
            self._send_command('STOP')
            self.get_logger().debug('Timeout: sent STOP command')

    def destroy_node(self):
        """ノード破棄時にシリアルポートをクローズする."""
        if self._serial is not None and self._serial.is_open:
            self._send_command('STOP')
            self._serial.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    """エントリポイント."""
    rclpy.init(args=args)
    node = MotorControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
