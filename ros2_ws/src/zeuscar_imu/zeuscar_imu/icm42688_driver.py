"""ICM-42688 IMUセンサードライバモジュール."""
import time
import struct

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
    # 注意: ACCEL_CONFIG0=0x06 はFS_SEL=000(±16g)を設定するため、
    #       スケールファクターは2048 LSB/gを使用する
    ACCEL_SCALE = 2048.0   # ±16g設定時: 2048 LSB/g
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
