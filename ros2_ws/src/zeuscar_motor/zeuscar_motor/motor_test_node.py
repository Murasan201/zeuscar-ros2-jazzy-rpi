"""全方向モーターテストノード.

すべての移動コマンドを順番に短時間実行してモーター動作を確認する。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class MotorTestNode(Node):
    """全方向モーターテストノード."""

    # テストするコマンドと表示名
    TEST_COMMANDS = [
        ('FORWARD', '前進'),
        ('BACKWARD', '後退'),
        ('LEFT', '左移動'),
        ('RIGHT', '右移動'),
        ('LEFTFORWARD', '左斜め前'),
        ('RIGHTFORWARD', '右斜め前'),
        ('LEFTBACKWARD', '左斜め後'),
        ('RIGHTBACKWARD', '右斜め後'),
        ('TURNLEFT', '左旋回'),
        ('TURNRIGHT', '右旋回'),
    ]

    def __init__(self):
        """ノードの初期化."""
        super().__init__('motor_test_node')

        # パラメータ
        self.declare_parameter('duration', 0.5)  # 各動作の継続時間（秒）
        self.declare_parameter('pause', 0.5)     # 動作間の停止時間（秒）

        self._duration = self.get_parameter('duration').value
        self._pause = self.get_parameter('pause').value

        # パブリッシャー
        self._pub = self.create_publisher(String, '/zeuscar/motor_cmd', 10)

        self.get_logger().info('モーターテストノードを開始します')
        self.get_logger().info(f'各動作: {self._duration}秒, 停止間隔: {self._pause}秒')

        # テスト開始用タイマー（少し遅延させて開始）
        self._test_timer = self.create_timer(1.0, self._run_test)
        self._test_started = False

    def _send_command(self, command: str):
        """コマンドを送信する."""
        msg = String()
        msg.data = command
        self._pub.publish(msg)

    def _run_test(self):
        """全方向テストを実行する."""
        if self._test_started:
            return
        self._test_started = True
        self._test_timer.cancel()

        self.get_logger().info('=== 全方向モーターテスト開始 ===')

        try:
            for command, name in self.TEST_COMMANDS:
                self.get_logger().info(f'テスト中: {name} ({command})')
                self._send_command(command)
                time.sleep(self._duration)

                # 停止
                self._send_command('STOP')
                time.sleep(self._pause)

            self.get_logger().info('=== 全方向モーターテスト完了 ===')

        except KeyboardInterrupt:
            self.get_logger().info('テストが中断されました')
        finally:
            # 安全のため停止コマンドを送信
            self._send_command('STOP')
            self.get_logger().info('モーターを停止しました')

        # ノードを終了
        raise SystemExit(0)


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)
    node = MotorTestNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
