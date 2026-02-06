# STORY-025: IMUデータパブリッシュノード仕様書

## 1. 概要

### 1.1 目的

ICM-42688 IMUセンサーから取得した加速度・角速度データをROS 2標準メッセージ（sensor_msgs/msg/Imu）としてパブリッシュするノードの設計仕様を定義する。

### 1.2 スコープ

- IMUデータパブリッシュノード（imu_node）の設計
- 単位変換ロジック（dps→rad/s, g→m/s²）
- エラーハンドリング戦略

### 1.3 スコープ外

- IMUデータのフィルタリング（カルマンフィルタ等）
- オドメトリ統合
- TFブロードキャスト

### 1.4 前提条件

- STORY-023/024（IMUテスト）完了済み
- ICM-42688 IMUセンサーの全軸正常動作確認済み
- ICM42688ドライバクラスが `icm42688_driver.py` に分離済み

---

## 2. ノード設計

### 2.1 基本情報

| 項目 | 値 |
|------|-----|
| パッケージ | `zeuscar_imu` |
| ノード名 | `imu_node` |
| 実装ファイル | `zeuscar_imu/imu_publish_node.py` |
| エントリポイント | `imu_node = zeuscar_imu.imu_publish_node:main` |

### 2.2 パブリッシュトピック

| 項目 | 値 |
|------|-----|
| トピック名 | `/imu/data_raw` |
| メッセージ型 | `sensor_msgs/msg/Imu` |
| QoS | `SensorDataQoS`（Best Effort, depth=5） |
| デフォルト周波数 | 50 Hz |
| フレームID | `imu_link` |

### 2.3 パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|-------------|-----|-------------|------|
| `i2c_bus` | int | 1 | I2Cバス番号 |
| `i2c_address` | int | 0x68 | I2Cデバイスアドレス |
| `publish_rate_hz` | double | 50.0 | パブリッシュ周波数 (Hz) |
| `frame_id` | string | `imu_link` | IMUフレームID |
| `linear_acceleration_stdev` | double | 0.05 | 加速度標準偏差 (m/s²) |
| `angular_velocity_stdev` | double | 0.01 | 角速度標準偏差 (rad/s) |

---

## 3. 単位変換

### 3.1 角速度変換

ICM-42688の出力（dps: degrees per second）からROS 2標準（rad/s）に変換する。

| 変換 | 式 |
|------|-----|
| dps → rad/s | `value_rad_s = value_dps × (π / 180)` |

### 3.2 加速度変換

ICM-42688の出力（g）からROS 2標準（m/s²）に変換する。

| 変換 | 式 |
|------|-----|
| g → m/s² | `value_m_s2 = value_g × 9.80665` |

### 3.3 orientation（姿勢）

ICM-42688は6軸IMU（加速度 + ジャイロ）のため、絶対姿勢は計算不可。

| 項目 | 値 |
|------|-----|
| orientation | すべて0.0（未使用） |
| orientation_covariance[0] | -1.0（データ無効を示す） |

---

## 4. Imuメッセージ構築

### 4.1 メッセージフィールド

| フィールド | 値の設定方法 |
|-----------|-------------|
| `header.stamp` | `self.get_clock().now().to_msg()` |
| `header.frame_id` | パラメータ `frame_id` |
| `orientation` | 全て0.0 |
| `orientation_covariance[0]` | -1.0 |
| `angular_velocity.x` | gyro_x (dps) × π/180 |
| `angular_velocity.y` | gyro_y (dps) × π/180 |
| `angular_velocity.z` | gyro_z (dps) × π/180 |
| `angular_velocity_covariance` | 対角成分 = stdev² |
| `linear_acceleration.x` | accel_x (g) × 9.80665 |
| `linear_acceleration.y` | accel_y (g) × 9.80665 |
| `linear_acceleration.z` | accel_z (g) × 9.80665 |
| `linear_acceleration_covariance` | 対角成分 = stdev² |

### 4.2 共分散行列

3×3行列（9要素のフラット配列）で、対角成分のみ設定:

```
[stdev², 0, 0, 0, stdev², 0, 0, 0, stdev²]
```

---

## 5. エラーハンドリング

### 5.1 I2C読み取り失敗時

| 項目 | 動作 |
|------|------|
| 単発の読み取り失敗 | warnログ出力、パブリッシュをスキップ |
| 連続失敗カウンタ | 失敗ごとにインクリメント |
| 連続失敗閾値 | 5回（MAX_CONSECUTIVE_FAILURES） |
| 閾値超過時 | errorログ出力、IMU再初期化を試行 |
| 成功時 | 連続失敗カウンタをリセット |

### 5.2 初期化失敗時

| 項目 | 動作 |
|------|------|
| I2C接続失敗 | errorログ出力、ノード終了 |
| デバイスID不一致 | errorログ出力、ノード終了 |
| センサー初期化失敗 | errorログ出力、ノード終了 |

---

## 6. ノードライフサイクル

### 6.1 起動シーケンス

1. パラメータ宣言・取得
2. ICM42688ドライバインスタンス生成
3. I2C接続オープン
4. デバイスID確認（WHO_AM_I）
5. センサー初期化
6. パブリッシャー作成
7. タイマー作成（publish_rate_hzに基づく周期）

### 6.2 タイマーコールバック

1. `read_scaled_data()` でデータ取得
2. 取得失敗時: 失敗カウンタ処理
3. 取得成功時: Imuメッセージ構築・パブリッシュ

### 6.3 終了処理

1. タイマー停止
2. IMU I2C接続クローズ

---

## 7. ファイル構成

```
ros2_ws/src/zeuscar_imu/
├── zeuscar_imu/
│   ├── __init__.py
│   ├── icm42688_driver.py      ← ドライバ（STORY-025で抽出）
│   ├── imu_test_node.py        ← 既存テストノード
│   └── imu_publish_node.py     ← 新規作成（Green フェーズ）
├── test/
│   ├── test_imu_publish.py     ← 新規作成（Red フェーズ）
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── launch/
│   └── imu_test.launch.py
├── setup.py
└── package.xml
```

---

## 8. テスト計画

### 8.1 単体テスト（test_imu_publish.py）

| カテゴリ | テスト数 | 内容 |
|---------|---------|------|
| 単位変換 | 6 | dps→rad/s, g→m/s² の境界値テスト |
| メッセージ構築 | 6 | Imuメッセージの各フィールド検証 |
| エラー処理 | 3 | 読み取り失敗、連続失敗、カウンタリセット |
| ドライバ | 2 | スケールファクター検証 |

### 8.2 テスト方針

- ROS 2ノード起動不要（純粋関数テスト）
- smbus2不要（モックでI2C通信を模擬）
- `pytest.approx()` で浮動小数点比較

---

## 改訂履歴

| 日付 | 担当者 | 内容 |
|------|--------|------|
| 2026-02-06 | - | 初版作成 |
