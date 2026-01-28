# ICM-42688 IMUセンサー仕様書

## 概要

本ドキュメントは、ZeusCar ロボットに搭載する6軸IMU（慣性計測ユニット）センサーモジュールの仕様をまとめたものである。

## 製品情報

- **購入元**: Amazon.co.jp
- **商品名**: 多機能 ICM42688 6軸ジャイロ加速度計モジュール
- **ASIN**: B0GF4CVPDW
- **センサーチップ**: ICM-42688（TDK/InvenSense製）

## センサー仕様

### ICM-42688 チップ仕様

| 項目 | 仕様 |
|------|------|
| センサータイプ | 6軸IMU（3軸ジャイロスコープ + 3軸加速度計） |
| インターフェース | I2C / SPI 両対応 |
| 動作電圧 | 1.71V〜3.6V（3.3V推奨） |
| ジャイロスコープ範囲 | ±15.625〜±2000 dps（プログラマブル） |
| 加速度計範囲 | ±2〜±16 g（プログラマブル） |

## モジュール基板仕様

### ピン配置

基板裏面の印刷より確認（左から右）:

| ピン番号 | ピン名 | 機能 | 説明 |
|----------|--------|------|------|
| 1 | VCC | 電源 | 3.3V 電源入力 |
| 2 | GND | グラウンド | 共通グラウンド |
| 3 | SCL/SCK | クロック | I2C: SCL / SPI: SCLK |
| 4 | SDA/SDI | データ | I2C: SDA / SPI: MOSI |
| 5 | SAO/SDO | アドレス/データ出力 | I2C: アドレス選択 / SPI: MISO |
| 6 | CS | チップセレクト | SPI: チップセレクト（LOW アクティブ） |
| 7 | INT1 | 割り込み1 | データレディ等の割り込み出力 |
| 8 | INT2 | 割り込み2 | 追加の割り込み出力 |

### 座標系

基板表面に X, Y, Z 軸の方向が印刷されている。ロボット搭載時は軸方向に注意すること。

### 対応センサーチップ

基板裏面の印刷によると、この基板は以下のセンサーに対応している：
- ICM-42688（●マークで選択）
- ICM-42670

本モジュールは **ICM-42688** が搭載されている。

## Raspberry Pi 4 との接続

### 接続方式: I2C

I2C接続を使用する。配線本数が少なく、実装が容易である。

### 配線一覧表

| No. | IMUモジュール | 機能 | Raspberry Pi 4 | 備考 |
|-----|---------------|------|----------------|------|
| 1 | VCC | 電源 | 3.3V (Pin 1) | 3.3V電源 |
| 2 | GND | グラウンド | GND (Pin 9) | 共通グラウンド |
| 3 | SCL | I2Cクロック | GPIO3/SCL1 (Pin 5) | I2C Bus 1 |
| 4 | SDA | I2Cデータ | GPIO2/SDA1 (Pin 3) | I2C Bus 1 |
| 5 | SAO | I2Cアドレス選択 | GND (Pin 9) | I2Cアドレス = 0x68 |
| 6 | CS | チップセレクト | 3.3V (Pin 1) | I2C使用時はHIGH固定 |
| 7 | INT1 | 割り込み1 | 未接続 | 初期段階では不要 |
| 8 | INT2 | 割り込み2 | 未接続 | 初期段階では不要 |

### 電源供給について

- ICM-42688 の消費電流は通常動作時 **約 0.5〜1mA** と非常に低い
- Raspberry Pi 4 の 3.3V ピンの供給能力は **最大約 50mA**（GPIO全体で共有）
- IMU センサー単体であれば 3.3V ピンからの直接給電で問題なし

### 配線図

```
IMU接続ピン
    ↓
3.3V ← VCC, CS  (1) (2)  5V
SDA ←────────  GPIO2 (3) (4)  5V
SCL ←────────  GPIO3 (5) (6)  GND
                GPIO4 (7) (8)  GPIO14
GND ← GND, SAO    GND (9) (10) GPIO15
```

### Raspberry Pi 4 GPIO ピン配置参考

```
                    3.3V (1) (2) 5V
          I2C SDA - GPIO2 (3) (4) 5V
          I2C SCL - GPIO3 (5) (6) GND
                    GPIO4 (7) (8) GPIO14
                      GND (9) (10) GPIO15
                   GPIO17 (11) (12) GPIO18
                   GPIO27 (13) (14) GND
                   GPIO22 (15) (16) GPIO23
                     3.3V (17) (18) GPIO24
                   GPIO10 (19) (20) GND
                    GPIO9 (21) (22) GPIO25
                   GPIO11 (23) (24) GPIO8
                      GND (25) (26) GPIO7
                    GPIO0 (27) (28) GPIO1
                    GPIO5 (29) (30) GND
                    GPIO6 (31) (32) GPIO12
                   GPIO13 (33) (34) GND
                   GPIO19 (35) (36) GPIO16
                   GPIO26 (37) (38) GPIO20
                      GND (39) (40) GPIO21
```

### I2C アドレス

SAO ピンを GND に接続するため、I2C アドレスは **0x68** となる。

| SAO接続先 | I2Cアドレス | 本構成 |
|-----------|-------------|--------|
| GND | 0x68 | ✓ 採用 |
| VCC (3.3V) | 0x69 | - |

### 割り込みピン（オプション）

INT1/INT2 はデータレディ割り込みやFIFOウォーターマーク通知などに使用可能。
本構成では ROS 2 によるポーリング方式でデータ取得を行うため、初期段階では未接続とする。

将来的に割り込み駆動が必要になった場合は、以下の GPIO を使用可能：

| ピン | 用途例 | 接続先候補 |
|------|--------|------------|
| INT1 | データレディ通知 | GPIO17 (Pin 11) または GPIO27 (Pin 13) |
| INT2 | 追加割り込み | GPIO22 (Pin 15) または GPIO23 (Pin 16) |

## 動作確認テスト

### テストノード概要

IMUセンサーの動作確認用に `zeuscar_imu` パッケージの `imu_test_node` を使用する。
このノードはロボットを短時間動かしながらIMUデータを取得し、センサーが正常に動作しているか確認する。

### ファイル構成

```
ros2_ws/src/zeuscar_imu/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/zeuscar_imu
├── zeuscar_imu/
│   ├── __init__.py
│   └── imu_test_node.py      # テストノード本体
├── launch/
│   └── imu_test.launch.py
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

### テストシーケンス

狭いスペースでも実行可能なよう、各動作は短時間で完了する：

| 順序 | 動作 | コマンド | 時間 | 確認項目 |
|------|------|----------|------|----------|
| 1 | 静止状態 | STOP | 2.0秒 | 基準値取得（accel_z ≈ +1.0g） |
| 2 | 前進 | FORWARD | 0.3秒 | accel_x の変化 |
| 3 | 停止 | STOP | 1.0秒 | 静止状態への復帰 |
| 4 | 後退 | BACKWARD | 0.3秒 | accel_x の変化（逆方向） |
| 5 | 停止 | STOP | 1.0秒 | 静止状態への復帰 |
| 6 | 左旋回 | TURNLEFT | 0.3秒 | gyro_z > 0 |
| 7 | 停止 | STOP | 1.0秒 | 静止状態への復帰 |
| 8 | 右旋回 | TURNRIGHT | 0.3秒 | gyro_z < 0 |
| 9 | 停止 | STOP | 1.0秒 | 静止状態への復帰 |

### パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|--------------|-----|-------------|------|
| `i2c_bus` | int | 1 | I2Cバス番号 |
| `i2c_address` | int | 0x68 | I2Cアドレス |
| `sample_rate_hz` | float | 10.0 | サンプリングレート (Hz) |

### 事前準備

#### 1. 依存ライブラリのインストール

```bash
pip install smbus2
```

#### 2. I2C の有効化確認

```bash
# I2Cデバイスの検出
sudo i2cdetect -y 1
```

期待される出力（0x68 にデバイスが表示される）:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

#### 3. パッケージのビルド

```bash
cd ~/ros2_ws
colcon build --packages-select zeuscar_imu
source install/setup.bash
```

### テスト実行手順

#### ターミナル1: モーターコントローラを起動

```bash
source ~/ros2_ws/install/setup.bash
ros2 run zeuscar_motor motor_controller_node
```

#### ターミナル2: IMUテストを実行

```bash
source ~/ros2_ws/install/setup.bash
ros2 run zeuscar_imu imu_test_node
```

または launch ファイルを使用:

```bash
ros2 launch zeuscar_imu imu_test.launch.py
```

### 出力例

```
[INFO] [imu_test_node]: === IMUセンサー動作確認テスト ===
[INFO] [imu_test_node]: I2Cバス: 1, アドレス: 0x68
[INFO] [imu_test_node]: WHO_AM_I: 0x47 (期待値: 0x47)
[INFO] [imu_test_node]: ICM-42688を検出しました
[INFO] [imu_test_node]: IMUを初期化しました

[INFO] [imu_test_node]: === テスト開始 ===
[INFO] [imu_test_node]: ※ロボットが動きます。周囲に注意してください。

[INFO] [imu_test_node]: --- 静止状態（基準値取得） (STOP, 2.0秒) ---
[INFO] [imu_test_node]: 静止状態（基準値取得） (サンプル数: 20):
  加速度平均 [g]: X=+0.012, Y=-0.008, Z=+0.998
  角速度平均 [dps]: X=+0.15, Y=-0.22, Z=+0.08

[INFO] [imu_test_node]: --- 前進テスト (FORWARD, 0.3秒) ---
[INFO] [imu_test_node]: 前進テスト (サンプル数: 3):
  加速度平均 [g]: X=+0.156, Y=-0.010, Z=+0.985
  角速度平均 [dps]: X=+0.20, Y=-0.18, Z=+0.12
...
[INFO] [imu_test_node]: === テスト完了 ===

[INFO] [imu_test_node]: 【確認ポイント】
[INFO] [imu_test_node]: 1. 静止状態: accel_z ≈ +1.0g（重力）、gyro ≈ 0
[INFO] [imu_test_node]: 2. 前進/後退: accel_x に変化が見られるか
[INFO] [imu_test_node]: 3. 左旋回: gyro_z > 0、右旋回: gyro_z < 0
```

### 判定基準

| 項目 | 正常値 | 異常時の確認事項 |
|------|--------|------------------|
| WHO_AM_I | 0x47 | 配線確認、I2Cアドレス確認 |
| 静止時 accel_z | +0.95〜+1.05 g | センサー取り付け向き確認 |
| 静止時 gyro | ±5 dps 以内 | センサー初期化確認 |
| 前進時 accel_x | 正の変化 | モーター動作確認 |
| 左旋回時 gyro_z | 正の値 | センサー軸方向確認 |
| 右旋回時 gyro_z | 負の値 | センサー軸方向確認 |

### トラブルシューティング

| 症状 | 原因 | 対処法 |
|------|------|--------|
| `smbus2ライブラリがインストールされていません` | ライブラリ未インストール | `pip install smbus2` |
| `I2Cバスを開けません` | I2C無効 | `sudo raspi-config` で I2C 有効化 |
| `ICM-42688が検出されません` | 配線不良またはアドレス違い | 配線確認、`i2cdetect` でアドレス確認 |
| `WHO_AM_I: 0xFF` | 通信エラー | 配線の接触不良確認 |
| センサー値が常に0 | 電源未供給 | VCC/GND 配線確認 |

## 参考資料

- [TDK ICM-42688-P データシート](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- Amazon商品ページ: https://www.amazon.co.jp/dp/B0GF4CVPDW

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-01-27 | 初版作成。基板写真から仕様を読み取り |
| 2026-01-27 | Raspberry Pi 4 との配線一覧表を追加。GPIO配置図を追加 |
| 2026-01-27 | 配線構成を確定。I2Cアドレス0x68、電源供給説明を追加 |
| 2026-01-28 | 動作確認テストノードの仕様と実行手順を追加 |
