# ICM-42688 6軸IMU セットアップガイド

## はじめに

このガイドでは、ICM-42688 6軸IMU（慣性計測ユニット）センサーを ZeusCar ROS 2 Jazzy 環境で使用するための手順を、ハードウェアの接続から ROS 2 トピック配信まで一貫して解説します。

### 対象者

- Raspberry Pi を使ったロボット開発に興味がある方
- IMU センサーを初めて扱う方
- ROS 2 で IMU データを活用したい方

### 完成イメージ

このガイドを最後まで進めると、以下のことができるようになります：

- ICM-42688 IMU センサーが Raspberry Pi 4 に正しく接続される
- I2C 経由で IMU のデータを読み取れる
- ROS 2 テストノードで IMU の動作確認ができる
- ROS 2 トピック `/imu/data_raw` から `sensor_msgs/msg/Imu` メッセージを受信できる（実装完了後）

---

## 1. ハードウェア準備

### 1.1 必要なもの

| 項目 | 説明 |
|------|------|
| ICM-42688 IMU モジュール | 6軸ジャイロ加速度計モジュール（TDK/InvenSense製） |
| ジャンパワイヤー（メス-メス） | 6本（VCC, GND, SDA, SCL, SAO, CS） |
| Raspberry Pi 4 | ZeusCar に搭載済みのもの |
| ZeusCar 本体 | メカナムホイールロボット（動作確認テスト時に使用） |

### 1.2 IMU モジュールの仕様

| 項目 | 仕様 |
|------|------|
| センサーチップ | ICM-42688（TDK/InvenSense製） |
| センサータイプ | 6軸IMU（3軸ジャイロスコープ + 3軸加速度計） |
| インターフェース | I2C / SPI 両対応（本構成では I2C を使用） |
| 動作電圧 | 1.71V〜3.6V（3.3V推奨） |
| ジャイロスコープ範囲 | ±15.625〜±2000 dps（プログラマブル） |
| 加速度計範囲 | ±2〜±16 g（プログラマブル） |
| I2C アドレス | 0x68（SAO=GND時） |

### 1.3 配線

IMU モジュールと Raspberry Pi 4 を以下のように接続します。

#### 配線一覧表

| IMU ピン | Raspberry Pi | 備考 |
|---------|-------------|------|
| VCC | 3.3V (Pin 1) | 電源 |
| GND | GND (Pin 9) | グラウンド |
| SDA | GPIO2/SDA1 (Pin 3) | I2C データ |
| SCL | GPIO3/SCL1 (Pin 5) | I2C クロック |
| SAO | GND (Pin 9) | アドレス 0x68 設定 |
| CS | 3.3V (Pin 1) | I2C 使用時は HIGH 固定 |
| INT1 | 未接続 | 初期段階では不要 |
| INT2 | 未接続 | 初期段階では不要 |

#### 配線図

```
IMU接続ピン
    ↓
3.3V ← VCC, CS  (1) (2)  5V
SDA ←────────  GPIO2 (3) (4)  5V
SCL ←────────  GPIO3 (5) (6)  GND
                GPIO4 (7) (8)  GPIO14
GND ← GND, SAO    GND (9) (10) GPIO15
```

> **ポイント**: VCC と CS は同じ 3.3V ピン（Pin 1）に、GND と SAO は同じ GND ピン（Pin 9）に接続します。I2C のデータ線は Pin 3（SDA）と Pin 5（SCL）の 2本のみです。

### 1.4 注意事項

- **電源電圧**: ICM-42688 は 3.3V 動作です。**絶対に 5V に接続しないでください**。センサーが破損します。
- **消費電流**: 通常動作時 約 0.5〜1mA と非常に低いため、Raspberry Pi の 3.3V ピンからの直接給電で問題ありません。
- **CS ピン**: I2C モードで使用するため、CS ピンは必ず 3.3V（HIGH）に接続してください。LOW にすると SPI モードになります。
- **SAO ピン**: GND に接続すると I2C アドレスが 0x68 になります。VCC に接続すると 0x69 になります。本構成では GND に接続します。
- **I2C プルアップ抵抗**: Raspberry Pi の GPIO2/GPIO3 には内蔵プルアップ抵抗があるため、外付けプルアップは不要です。

---

## 2. ソフトウェアセットアップ

### 2.1 I2C の有効化

Raspberry Pi で I2C を使用するには、まず I2C インターフェースを有効にする必要があります。

```bash
sudo raspi-config
```

メニューから以下を選択します：

1. `Interface Options` を選択
2. `I2C` を選択
3. `Yes`（有効化）を選択
4. `Finish` を選択

変更を反映するため、再起動が必要な場合があります：

```bash
sudo reboot
```

### 2.2 I2C ツールのインストール

I2C デバイスの検出・デバッグに使用するツールをインストールします。

```bash
sudo apt install -y i2c-tools
```

### 2.3 IMU の接続確認

IMU が I2C バスに正しく接続されているか確認します。

```bash
sudo i2cdetect -y 1
```

期待される出力（0x68 にデバイスが表示される）：

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

`68` が表示されていれば IMU が正しく認識されています。表示されない場合は「6. トラブルシューティング」の「6.1 IMU が検出されない」を参照してください。

### 2.4 smbus2 ライブラリのインストール

Python から I2C デバイスにアクセスするためのライブラリをインストールします。

#### pip3 のインストール

Ubuntu 24.04 では pip がデフォルトでインストールされていない場合があります。まず pip3 をインストールします：

```bash
sudo apt install -y python3-pip
```

#### smbus2 のインストール

Ubuntu 24.04 では PEP 668 により、システムワイドの pip インストールが制限されています。`--break-system-packages` オプションを使用します：

```bash
pip3 install --break-system-packages smbus2
```

> **注意**: `--break-system-packages` オプションを使用すると、システムの Python 環境に直接インストールされます。ROS 2 環境ではこの方法が一般的に使用されます。

#### インストール確認

```bash
python3 -c "import smbus2; print('smbus2 OK')"
```

以下のように表示されれば成功です：

```
smbus2 OK
```

---

## 3. ROS 2 パッケージのビルド

### 3.1 ビルド手順

`zeuscar_imu` パッケージをビルドします。

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zeuscar_imu --symlink-install
source install/setup.bash
```

成功すると以下のように表示されます：

```
Starting >>> zeuscar_imu
Finished <<< zeuscar_imu [x.xs]

Summary: 1 package finished [x.xs]
```

> **補足**: `--symlink-install` オプションを使用すると、Python ファイルの変更時に毎回ビルドし直す必要がなくなります（シンボリックリンクで参照されるため）。

### 3.2 ビルド確認

パッケージが正しくビルドされたか確認します。

```bash
ros2 pkg list | grep zeuscar_imu
```

以下のように表示されれば成功です：

```
zeuscar_imu
```

パッケージに含まれる実行可能ノードを確認します：

```bash
ros2 pkg executables zeuscar_imu
```

以下のように表示されます（実装状況によって内容が異なります）：

```
zeuscar_imu imu_test_node
```

---

## 4. 動作確認テスト

### 4.1 テストの概要

`imu_test_node` は、IMU センサーの動作確認用テストノードです。ロボットを短時間動かしながら IMU のデータを取得し、加速度・ジャイロスコープの値が正常かどうかを確認します。

テストノードは以下の流れで動作します：

1. IMU センサーの初期化（WHO_AM_I 確認、センサー設定）
2. 静止状態で基準値を取得
3. 前進・後退・左旋回・右旋回の各動作を短時間実行
4. 各動作中の IMU データをサンプリングしてログ出力
5. テスト完了後、モーターを停止して終了

### 4.2 事前条件

テストを実行する前に、以下の条件を満たしていることを確認してください。

- **モーターコントローラノード（`motor_controller_node`）が起動可能であること**
  - `zeuscar_motor` パッケージがビルド済みであること
- **Arduino が USB で Raspberry Pi に接続されていること**
  - モーター制御はArduino経由で行います
- **バッテリーが充電済みであること**
  - ロボットのモーターが動作するために十分な電力が必要です
- **ロボットの周囲に十分なスペースがあること**
  - テスト中にロボットが短時間動きます

### 4.3 テスト実行手順

2つのターミナルを使用します。

#### ターミナル 1: モーターコントローラを起動

```bash
source ~/ros2_ws/install/setup.bash
ros2 run zeuscar_motor motor_controller_node
```

#### ターミナル 2: IMU テストを実行

```bash
source ~/ros2_ws/install/setup.bash
ros2 run zeuscar_imu imu_test_node
```

または、launch ファイルを使用して実行することもできます：

```bash
ros2 launch zeuscar_imu imu_test.launch.py
```

> **警告**: テスト開始後、ロボットが自動的に動き出します。周囲の安全を確認してから実行してください。

### 4.4 テストシーケンス

テストノードは以下のシーケンスを自動的に実行します。各動作は狭いスペースでも安全に実行できるよう、短時間に設計されています。

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

### 4.5 テスト出力例

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

### 4.6 判定基準

| 項目 | 正常値 | 異常時の確認事項 |
|------|--------|------------------|
| WHO_AM_I | 0x47 | 配線確認、I2C アドレス確認 |
| 静止時 accel_z | +0.95〜+1.05 g | センサー取り付け向き確認 |
| 静止時 gyro | ±5 dps 以内 | センサー初期化確認 |
| 前進時 accel_x | 正の変化 | モーター動作確認 |
| 左旋回時 gyro_z | 正の値 | センサー軸方向確認 |
| 右旋回時 gyro_z | 負の値 | センサー軸方向確認 |

---

## 5. IMU データパブリッシュノード（実装予定）

> **注意**: このセクションは STORY-025 Green フェーズ完了後に使用可能になります。現時点では実装予定の仕様を記載しています。

### 5.1 概要

`imu_node`（IMU データパブリッシュノード）は、ICM-42688 から取得した加速度・角速度データを ROS 2 標準メッセージ `sensor_msgs/msg/Imu` として `/imu/data_raw` トピックにパブリッシュします。

このノードにより、他の ROS 2 ノード（SLAMやナビゲーション等）が IMU データを利用できるようになります。

| 項目 | 値 |
|------|-----|
| ノード名 | `imu_node` |
| 実装ファイル | `zeuscar_imu/imu_publish_node.py` |
| トピック | `/imu/data_raw` |
| メッセージ型 | `sensor_msgs/msg/Imu` |
| QoS | SensorDataQoS（Best Effort, depth=5） |
| デフォルト周波数 | 50 Hz |
| フレームID | `imu_link` |

### 5.2 起動方法

```bash
source ~/ros2_ws/install/setup.bash
ros2 run zeuscar_imu imu_node
```

### 5.3 パラメータ

ノードの動作はパラメータで制御できます。デフォルト値のまま使用しても問題ありません。

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `i2c_bus` | 1 | I2C バス番号 |
| `i2c_address` | 0x68 | I2C アドレス |
| `publish_rate_hz` | 50.0 | パブリッシュ周波数（Hz） |
| `frame_id` | `imu_link` | フレーム ID |
| `linear_acceleration_stdev` | 0.05 | 加速度標準偏差（m/s²） |
| `angular_velocity_stdev` | 0.01 | 角速度標準偏差（rad/s） |

パラメータを変更して起動する例：

```bash
ros2 run zeuscar_imu imu_node --ros-args \
  -p publish_rate_hz:=100.0 \
  -p frame_id:=imu_link
```

### 5.4 トピック確認

ノード起動後、以下のコマンドで IMU データの配信を確認できます。

```bash
# トピック一覧で確認
ros2 topic list | grep imu
```

期待される出力：

```
/imu/data_raw
```

```bash
# データの確認
ros2 topic echo /imu/data_raw
```

期待される出力例：

```yaml
header:
  stamp:
    sec: 1706000000
    nanosec: 123456789
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.002617
  y: -0.003840
  z: 0.001396
angular_velocity_covariance: [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
linear_acceleration:
  x: 0.11768
  y: -0.07845
  z: 9.79682
linear_acceleration_covariance: [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
```

```bash
# 配信周波数の確認
ros2 topic hz /imu/data_raw
```

期待される出力例：

```
average rate: 50.012
        min: 0.019s max: 0.021s std dev: 0.00050s window: 50
```

### 5.5 Imu メッセージの読み方

`sensor_msgs/msg/Imu` メッセージの各フィールドの意味を説明します。

| フィールド | 説明 |
|-----------|------|
| `header.stamp` | データ取得時のタイムスタンプ |
| `header.frame_id` | IMU センサーの座標フレーム名（`imu_link`） |
| `orientation` | 姿勢（クォータニオン）。6軸 IMU では計算不可のため、すべて 0.0 |
| `orientation_covariance` | 姿勢の共分散行列。`[0]` が -1.0 のとき「データ無効」を意味する |
| `angular_velocity` | 角速度（x, y, z）。単位は rad/s |
| `angular_velocity_covariance` | 角速度の共分散行列。対角成分に標準偏差の二乗を設定 |
| `linear_acceleration` | 加速度（x, y, z）。単位は m/s²。静止時は重力加速度が含まれる |
| `linear_acceleration_covariance` | 加速度の共分散行列。対角成分に標準偏差の二乗を設定 |

> **ポイント**: `orientation_covariance[0]` が -1.0 であることは、「このセンサーは姿勢データを提供しない」ことを他のノードに伝えています。6軸 IMU（加速度 + ジャイロ）では絶対姿勢を計算できないため、この設定が正しいです。9軸 IMU（磁力計付き）や姿勢推定フィルタを使用する場合に姿勢が利用可能になります。

### 5.6 単位系

ROS 2 では SI 単位系が標準です。ICM-42688 から取得した生データは以下のように変換されます。

| 物理量 | ICM-42688 の出力単位 | ROS 2 標準単位 | 変換式 |
|--------|---------------------|---------------|--------|
| 角速度 | dps（degrees per second） | rad/s | value_dps x (pi / 180) |
| 加速度 | g（重力加速度） | m/s² | value_g x 9.80665 |
| 姿勢 | -（6軸では利用不可） | クォータニオン | すべて 0.0（covariance[0] = -1.0） |

> **具体例**: 静止状態で Z 軸の加速度が約 1.0 g の場合、ROS 2 メッセージでは `linear_acceleration.z` が約 9.807 m/s² になります。

---

## 6. トラブルシューティング

### 6.1 IMU が検出されない

**症状**: `sudo i2cdetect -y 1` で 0x68 が表示されない。

**確認手順**:

1. **配線を確認する**
   - VCC が 3.3V（Pin 1）に接続されているか
   - GND が GND（Pin 9）に接続されているか
   - SDA が GPIO2（Pin 3）に接続されているか
   - SCL が GPIO3（Pin 5）に接続されているか

2. **I2C が有効になっているか確認する**
   ```bash
   ls /dev/i2c*
   ```
   `/dev/i2c-1` が表示されなければ I2C が無効です。「2.1 I2C の有効化」の手順を実施してください。

3. **SAO ピンの接続を確認する**
   - SAO が GND に接続されていれば 0x68
   - SAO が VCC に接続されていれば 0x69
   - SAO が未接続だとアドレスが不安定になることがあります

4. **CS ピンの接続を確認する**
   - CS が 3.3V（HIGH）に接続されていることを確認
   - LOW になっていると SPI モードになり、I2C では認識されません

### 6.2 WHO_AM_I 値が不正

**症状**: `WHO_AM_I: 0xFF (期待値: 0x47)` と表示される。

**原因と対処**:

| WHO_AM_I 値 | 原因 | 対処法 |
|-------------|------|--------|
| 0xFF | 通信エラー（配線の接触不良） | ジャンパワイヤーの接続を確認し、しっかり差し込む |
| 0x00 | 電源未供給 | VCC と GND の配線を確認 |
| 0x47 以外の有効値 | 別のセンサーが接続されている | ICM-42688 モジュールであることを確認 |

### 6.3 加速度値が異常に小さい（スケールファクター問題）

**症状**: 静止状態で accel_z が 1.0 g ではなく、非常に小さい値（例: 0.06 g）になる。

**原因**: ドライバのスケールファクターが実際のセンサー設定と一致していない可能性があります。

**確認ポイント**:

- ICM-42688 のレジスタ ACCEL_CONFIG0（0x50）に書き込まれた値を確認
- 設定値 `0x06` の場合、FS_SEL=000（±16g）のため、スケールファクターは 2048 LSB/g
- ドライバの `ACCEL_SCALE` 定数が正しい値（2048.0）になっているか確認

```python
# icm42688_driver.py 内のスケールファクター
ACCEL_SCALE = 2048.0   # ±16g設定時: 2048 LSB/g
GYRO_SCALE = 131.0     # ±250dps設定時: 131 LSB/dps
```

### 6.4 smbus2 のインストールエラー

**症状**: `pip3 install smbus2` で `externally-managed-environment` エラーが発生する。

```
error: externally-managed-environment
× This environment is externally managed
```

**対処法**: Ubuntu 24.04 の PEP 668 制限により、`--break-system-packages` オプションが必要です。

```bash
pip3 install --break-system-packages smbus2
```

### 6.5 python コマンドが見つからない

**症状**: `python` コマンドを実行すると `command not found` エラーになる。

**原因**: Ubuntu 24.04 では `python` コマンドはデフォルトで存在しません。`python3` を使用してください。

```bash
# NG
python -c "import smbus2"

# OK
python3 -c "import smbus2; print('smbus2 OK')"
```

> **補足**: ROS 2 Jazzy 環境では、ノードの実行は `ros2 run` コマンドを使用するため、`python` / `python3` コマンドを直接使う場面は限られます。

---

## 7. 関連ドキュメント

- [ICM-42688 IMUセンサー仕様書](hardware/icm42688_imu_sensor.md) - ハードウェアの詳細仕様、ピン配置、座標系の情報
- [STORY-025 仕様書](operations/specs/STORY-025_imu_publish_node.md) - IMUデータパブリッシュノードの設計仕様
- [セットアップガイド](setup_guide.md) - Section 9: IMU セットアップの概要手順

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-02-06 | 初版作成 |
