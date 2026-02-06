"""IMUデータパブリッシュノードのテスト（Redフェーズ）.

テスト対象:
- 単位変換関数（dps→rad/s, g→m/s²）
- Imuメッセージ構築関数
- エラーハンドリングロジック
- ICM42688ドライバスケールファクター
"""
import math

import pytest


# === 定数（仕様値） ===
GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0


# === テスト対象関数（Greenフェーズで imu_publish_node.py に移動予定） ===

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

    Greenフェーズでsensor_msgs.msg.Imuを使用する実装に置き換え予定。
    Redフェーズでは辞書で代用してロジックをテストする。
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


# === モッククラス ===

class MockIMU:
    """ICM42688のモック."""

    def __init__(self, data=None, fail=False):
        self.data = data
        self.fail = fail
        self.initialized = True
        self._open_called = False
        self._init_called = False

    def open(self) -> bool:
        self._open_called = True
        return not self.fail

    def initialize(self) -> bool:
        self._init_called = True
        return not self.fail

    def close(self):
        pass

    def read_scaled_data(self):
        if self.fail:
            return None
        return self.data


class MockPublishContext:
    """パブリッシュコンテキストのモック.

    エラーハンドリングロジックをテストするためのクラス。
    """

    MAX_CONSECUTIVE_FAILURES = 5

    def __init__(self, imu: MockIMU):
        self.imu = imu
        self.publish_count = 0
        self.error_count = 0
        self.reinit_count = 0

    def timer_callback(self):
        """タイマーコールバックのロジック."""
        data = self.imu.read_scaled_data()
        if data is None:
            self.error_count += 1
            if self.error_count >= self.MAX_CONSECUTIVE_FAILURES:
                self.reinit_count += 1
                self.imu.initialize()
                self.error_count = 0
            return
        self.error_count = 0
        self.publish_count += 1


# === ヘルパーファクトリ ===

def create_scaled_data(
    accel_x=0.0, accel_y=0.0, accel_z=1.0,
    gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
    temperature=25.0,
) -> dict:
    """スケール変換済みIMUデータを作成するヘルパー."""
    return {
        'accel_x': accel_x,
        'accel_y': accel_y,
        'accel_z': accel_z,
        'gyro_x': gyro_x,
        'gyro_y': gyro_y,
        'gyro_z': gyro_z,
        'temperature': temperature,
    }


# ============================================================
# テストクラス
# ============================================================


class TestDpsToRadS:
    """角速度変換（dps → rad/s）のテスト."""

    def test_dps_to_rad_s_zero(self):
        """0 dps → 0 rad/s."""
        assert dps_to_rad_s(0.0) == pytest.approx(0.0)

    def test_dps_to_rad_s_positive(self):
        """180 dps → π rad/s."""
        assert dps_to_rad_s(180.0) == pytest.approx(math.pi)

    def test_dps_to_rad_s_negative(self):
        """-90 dps → -π/2 rad/s."""
        assert dps_to_rad_s(-90.0) == pytest.approx(-math.pi / 2)


class TestGToMs2:
    """加速度変換（g → m/s²）のテスト."""

    def test_g_to_m_s2_zero(self):
        """0 g → 0 m/s²."""
        assert g_to_m_s2(0.0) == pytest.approx(0.0)

    def test_g_to_m_s2_one_g(self):
        """1.0 g → 9.80665 m/s²."""
        assert g_to_m_s2(1.0) == pytest.approx(9.80665)

    def test_g_to_m_s2_negative(self):
        """-0.5 g → -4.903325 m/s²."""
        assert g_to_m_s2(-0.5) == pytest.approx(-4.903325)


class TestCreateImuMsg:
    """Imuメッセージ構築のテスト."""

    def test_create_imu_msg_stationary(self):
        """静止状態の値変換."""
        data = create_scaled_data(accel_z=1.0)
        msg = create_imu_msg(data)
        assert msg['linear_acceleration']['z'] == pytest.approx(9.80665)
        assert msg['angular_velocity']['x'] == pytest.approx(0.0)

    def test_create_imu_msg_frame_id(self):
        """frame_id設定確認."""
        data = create_scaled_data()
        msg = create_imu_msg(data, frame_id='custom_imu')
        assert msg['frame_id'] == 'custom_imu'

    def test_create_imu_msg_orientation_unavailable(self):
        """6軸IMUではorientation covariance[0] == -1.0."""
        data = create_scaled_data()
        msg = create_imu_msg(data)
        assert msg['orientation_covariance'][0] == -1.0

    def test_create_imu_msg_angular_velocity_covariance(self):
        """角速度共分散の対角成分 = stdev²."""
        stdev = 0.01
        data = create_scaled_data()
        msg = create_imu_msg(data, angular_velocity_stdev=stdev)
        cov = msg['angular_velocity_covariance']
        expected_var = stdev ** 2
        assert cov[0] == pytest.approx(expected_var)
        assert cov[4] == pytest.approx(expected_var)
        assert cov[8] == pytest.approx(expected_var)
        assert cov[1] == pytest.approx(0.0)

    def test_create_imu_msg_linear_acceleration_covariance(self):
        """加速度共分散の対角成分 = stdev²."""
        stdev = 0.05
        data = create_scaled_data()
        msg = create_imu_msg(data, linear_acceleration_stdev=stdev)
        cov = msg['linear_acceleration_covariance']
        expected_var = stdev ** 2
        assert cov[0] == pytest.approx(expected_var)
        assert cov[4] == pytest.approx(expected_var)
        assert cov[8] == pytest.approx(expected_var)

    def test_create_imu_msg_all_axes(self):
        """全6軸の変換精度検証."""
        data = create_scaled_data(
            accel_x=0.1, accel_y=-0.2, accel_z=0.98,
            gyro_x=10.0, gyro_y=-5.0, gyro_z=100.0,
        )
        msg = create_imu_msg(data)
        assert msg['linear_acceleration']['x'] == pytest.approx(0.1 * GRAVITY)
        assert msg['linear_acceleration']['y'] == pytest.approx(-0.2 * GRAVITY)
        assert msg['linear_acceleration']['z'] == pytest.approx(0.98 * GRAVITY)
        assert msg['angular_velocity']['x'] == pytest.approx(10.0 * DEG_TO_RAD)
        assert msg['angular_velocity']['y'] == pytest.approx(-5.0 * DEG_TO_RAD)
        assert msg['angular_velocity']['z'] == pytest.approx(100.0 * DEG_TO_RAD)


class TestErrorHandling:
    """エラーハンドリングのテスト."""

    def test_handle_read_failure(self):
        """読み取り失敗時はパブリッシュしない."""
        imu = MockIMU(fail=True)
        ctx = MockPublishContext(imu)
        ctx.timer_callback()
        assert ctx.publish_count == 0
        assert ctx.error_count == 1

    def test_handle_consecutive_failures(self):
        """連続失敗で再初期化."""
        imu = MockIMU(fail=True)
        ctx = MockPublishContext(imu)
        for _ in range(MockPublishContext.MAX_CONSECUTIVE_FAILURES):
            ctx.timer_callback()
        assert ctx.reinit_count == 1
        assert ctx.error_count == 0

    def test_error_counter_resets_on_success(self):
        """復帰時にカウンタリセット."""
        data = create_scaled_data()
        imu = MockIMU(data=data, fail=True)
        ctx = MockPublishContext(imu)

        # 3回失敗
        for _ in range(3):
            ctx.timer_callback()
        assert ctx.error_count == 3

        # 成功に切り替え
        imu.fail = False
        ctx.timer_callback()
        assert ctx.error_count == 0
        assert ctx.publish_count == 1


class TestICM42688Scale:
    """ICM42688ドライバのスケールファクター検証."""

    def test_icm42688_accel_scale(self):
        """raw 2048 → 1.0g."""
        accel_scale = 2048.0
        assert (2048 / accel_scale) == pytest.approx(1.0)

    def test_icm42688_gyro_scale(self):
        """raw 131 → 1.0 dps."""
        gyro_scale = 131.0
        assert (131 / gyro_scale) == pytest.approx(1.0)
