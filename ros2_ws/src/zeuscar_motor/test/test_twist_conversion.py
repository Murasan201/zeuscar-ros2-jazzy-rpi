"""Twist→コマンド変換ロジックのテスト."""
import pytest
from geometry_msgs.msg import Twist


class MockMotorControllerNode:
    """テスト用のモックノード."""

    def __init__(self):
        self._linear_threshold = 0.1
        self._angular_threshold = 0.1

    def _twist_to_command(self, msg: Twist) -> str:
        """Twistメッセージをコマンド文字列に変換する."""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        if abs(linear_x) < self._linear_threshold:
            linear_x = 0.0
        if abs(linear_y) < self._linear_threshold:
            linear_y = 0.0
        if abs(angular_z) < self._angular_threshold:
            angular_z = 0.0

        if angular_z > 0:
            return 'TURNLEFT'
        elif angular_z < 0:
            return 'TURNRIGHT'

        if linear_x > 0 and linear_y == 0:
            return 'FORWARD'
        elif linear_x < 0 and linear_y == 0:
            return 'BACKWARD'

        if linear_x == 0 and linear_y > 0:
            return 'LEFT'
        elif linear_x == 0 and linear_y < 0:
            return 'RIGHT'

        if linear_x > 0 and linear_y > 0:
            return 'LEFTFORWARD'
        elif linear_x > 0 and linear_y < 0:
            return 'RIGHTFORWARD'
        elif linear_x < 0 and linear_y > 0:
            return 'LEFTBACKWARD'
        elif linear_x < 0 and linear_y < 0:
            return 'RIGHTBACKWARD'

        return 'STOP'


def create_twist(linear_x=0.0, linear_y=0.0, angular_z=0.0) -> Twist:
    """Twistメッセージを作成するヘルパー."""
    msg = Twist()
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = angular_z
    return msg


class TestTwistConversion:
    """Twist変換のテストクラス."""

    def setup_method(self):
        """テストごとの初期化."""
        self.node = MockMotorControllerNode()

    def test_forward(self):
        """前進コマンドのテスト."""
        twist = create_twist(linear_x=1.0)
        assert self.node._twist_to_command(twist) == 'FORWARD'

    def test_backward(self):
        """後退コマンドのテスト."""
        twist = create_twist(linear_x=-1.0)
        assert self.node._twist_to_command(twist) == 'BACKWARD'

    def test_left(self):
        """左移動コマンドのテスト."""
        twist = create_twist(linear_y=1.0)
        assert self.node._twist_to_command(twist) == 'LEFT'

    def test_right(self):
        """右移動コマンドのテスト."""
        twist = create_twist(linear_y=-1.0)
        assert self.node._twist_to_command(twist) == 'RIGHT'

    def test_leftforward(self):
        """左斜め前進コマンドのテスト."""
        twist = create_twist(linear_x=1.0, linear_y=1.0)
        assert self.node._twist_to_command(twist) == 'LEFTFORWARD'

    def test_rightforward(self):
        """右斜め前進コマンドのテスト."""
        twist = create_twist(linear_x=1.0, linear_y=-1.0)
        assert self.node._twist_to_command(twist) == 'RIGHTFORWARD'

    def test_leftbackward(self):
        """左斜め後退コマンドのテスト."""
        twist = create_twist(linear_x=-1.0, linear_y=1.0)
        assert self.node._twist_to_command(twist) == 'LEFTBACKWARD'

    def test_rightbackward(self):
        """右斜め後退コマンドのテスト."""
        twist = create_twist(linear_x=-1.0, linear_y=-1.0)
        assert self.node._twist_to_command(twist) == 'RIGHTBACKWARD'

    def test_turnleft(self):
        """左旋回コマンドのテスト."""
        twist = create_twist(angular_z=1.0)
        assert self.node._twist_to_command(twist) == 'TURNLEFT'

    def test_turnright(self):
        """右旋回コマンドのテスト."""
        twist = create_twist(angular_z=-1.0)
        assert self.node._twist_to_command(twist) == 'TURNRIGHT'

    def test_stop(self):
        """停止コマンドのテスト."""
        twist = create_twist()
        assert self.node._twist_to_command(twist) == 'STOP'

    def test_threshold_forward(self):
        """閾値以下の値は無視されることを確認."""
        twist = create_twist(linear_x=0.05)
        assert self.node._twist_to_command(twist) == 'STOP'

    def test_angular_priority(self):
        """旋回が直進より優先されることを確認."""
        twist = create_twist(linear_x=1.0, angular_z=1.0)
        assert self.node._twist_to_command(twist) == 'TURNLEFT'
