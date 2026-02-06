# IMUデータパブリッシュノード実装ガイド

## はじめに

### このガイドについて

本ガイドは、ZeusCar ROS 2プロジェクトにおいて、ICM-42688 6軸IMUセンサーのデータをROS 2トピックとして配信するノード（`imu_node`）を、TDD（テスト駆動開発）の手法で実装した手順を記録したものである。

ROS 2ノード開発におけるTDDの実践例として、教材・参考資料として活用できる。

### 前提条件

- ROS 2 Jazzy がインストール済み
- zeuscar_imu パッケージが作成済み（STORY-021/022）
- ICM-42688 IMUセンサーの動作確認済み（STORY-023/024）
- ICM42688ドライバクラスが `imu_test_node.py` 内に存在する状態

### 関連ドキュメント

| ドキュメント | パス |
|-------------|------|
| IMUセンサー仕様書 | `docs/hardware/icm42688_imu_sensor.md` |
| パブリッシュノード仕様書 | `docs/operations/specs/STORY-025_imu_publish_node.md` |
| IMUセットアップガイド | `docs/imu_setup_guide.md` |
| アジャイル・TDD運用ルール | `docs/agile_tdd_operating_rules.md` |
| EPIC-007ジャーナル | `docs/operations/agile/journals/EPIC-007.md` |

---

## 全体の流れ

TDDの基本サイクル「Red → Green → Refactor」に従い、以下の順序で実装を進めた。

```
1. 準備: ドライバの分離（リファクタリング）
2. Red: 仕様書作成 → 失敗するテスト作成
3. Green: テストをパスする実装の作成
4. 検証: ビルド・テスト実行
5. （次ステップ）実機テスト・統合テスト
```

---

## Step 1: ドライバの分離（準備）

### 目的

既存の `imu_test_node.py` に含まれている ICM42688 ドライバクラスを、パブリッシュノードからも再利用できるよう独立モジュールに分離する。

### 手順

1. **新規ファイル作成**: `zeuscar_imu/icm42688_driver.py`
   - `imu_test_node.py` から `ICM42688` クラスと関連する import・定数を抽出
   - クラスの中身は一切変更しない

2. **既存ファイル修正**: `zeuscar_imu/imu_test_node.py`
   - ICM42688 クラスの定義を削除
   - `from zeuscar_imu.icm42688_driver import ICM42688, SMBUS_AVAILABLE` に変更

3. **ビルド確認**
   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select zeuscar_imu --symlink-install
   ```

### ポイント

- リファクタリングの原則: **外部から見た振る舞いを変えない**
- ビルドが通ることで、既存機能が壊れていないことを確認
- ドライバを分離することで、テストノードとパブリッシュノードの両方から利用可能になる

### ファイル構成（分離後）

```
zeuscar_imu/
├── __init__.py
├── icm42688_driver.py    ← 新規（ドライバ抽出）
└── imu_test_node.py      ← 修正（import変更のみ）
```

---

## Step 2: 仕様書の作成

### 目的

実装に先立ち、ノードの設計を仕様書として文書化する。TDDでは「何を作るか」を明確にしてからテストを書く。

### 作成した仕様書

`docs/operations/specs/STORY-025_imu_publish_node.md`

### 主要な設計決定

| 項目 | 決定内容 | 理由 |
|------|---------|------|
| ノード名 | `imu_node` | ROS 2の慣例に従いシンプルな名前 |
| トピック名 | `/imu/data_raw` | `data_raw` はフィルタ未適用を示すROS慣例 |
| メッセージ型 | `sensor_msgs/msg/Imu` | ROS 2標準のIMUメッセージ |
| QoS | SensorDataQoS (Best Effort) | センサーデータはリアルタイム性を優先 |
| 周波数 | 50 Hz | IMUデータとして十分な更新頻度 |
| orientation | 未使用 (covariance[0] = -1.0) | 6軸IMUでは絶対姿勢が計算不可 |

### 単位変換の設計

| センサー出力 | ROS 2標準 | 変換式 |
|-------------|----------|--------|
| 角速度: dps (degrees/sec) | rad/s | × π/180 |
| 加速度: g | m/s² | × 9.80665 |

### エラーハンドリング方針

| 状況 | 動作 |
|------|------|
| 単発のI2C読み取り失敗 | warnログ、パブリッシュスキップ |
| 5回連続失敗 | errorログ、IMU再初期化 |
| 成功復帰時 | 失敗カウンタをリセット |

---

## Step 3: テストの作成（Redフェーズ）

### 目的

テスト対象の関数がまだ存在しない状態で、期待する振る舞いをテストとして定義する。

### テストファイル

`test/test_imu_publish.py`

### テスト設計の方針

| 方針 | 理由 |
|------|------|
| ROS 2ノード起動不要 | 純粋関数のみテストし、テスト実行を高速に |
| smbus2不要（モック使用） | I2Cハードウェアなしでテスト可能に |
| `pytest.approx()` で比較 | 浮動小数点の丸め誤差を許容 |
| 既存テスト（test_twist_conversion.py）のパターンに準拠 | プロジェクト内の一貫性を保つ |

### テストケース一覧（17件）

| カテゴリ | テスト名 | 検証内容 |
|---------|---------|---------|
| 単位変換 | test_dps_to_rad_s_zero | 0 dps → 0 rad/s |
| 単位変換 | test_dps_to_rad_s_positive | 180 dps → π rad/s |
| 単位変換 | test_dps_to_rad_s_negative | -90 dps → -π/2 rad/s |
| 単位変換 | test_g_to_m_s2_zero | 0 g → 0 m/s² |
| 単位変換 | test_g_to_m_s2_one_g | 1.0 g → 9.80665 m/s² |
| 単位変換 | test_g_to_m_s2_negative | -0.5 g → -4.903325 m/s² |
| メッセージ構築 | test_create_imu_msg_stationary | 静止状態での値変換 |
| メッセージ構築 | test_create_imu_msg_frame_id | frame_id が正しく設定される |
| メッセージ構築 | test_create_imu_msg_orientation_unavailable | covariance[0] == -1.0 |
| メッセージ構築 | test_create_imu_msg_angular_velocity_covariance | 対角成分 = stdev² |
| メッセージ構築 | test_create_imu_msg_linear_acceleration_covariance | 対角成分 = stdev² |
| メッセージ構築 | test_create_imu_msg_all_axes | 全6軸の変換精度 |
| エラー処理 | test_handle_read_failure | 失敗時パブリッシュしない |
| エラー処理 | test_handle_consecutive_failures | 5回連続失敗で再初期化 |
| エラー処理 | test_error_counter_resets_on_success | 成功時カウンタリセット |
| ドライバ | test_icm42688_accel_scale | raw 2048 → 1.0g |
| ドライバ | test_icm42688_gyro_scale | raw 131 → 1.0 dps |

### Redフェーズの工夫: テストファイル内に仮実装を定義

テスト対象モジュール（`imu_publish_node.py`）がまだ存在しないため、テストファイル内にテスト対象の関数を仮実装として定義した。

```
テストファイル内に定義:
- dps_to_rad_s() ... 仮実装
- g_to_m_s2() ... 仮実装
- create_imu_msg() ... 仮実装
- MockIMU ... モッククラス
- MockPublishContext ... エラーハンドリング検証用モック
- create_scaled_data() ... テストデータファクトリ
```

これにより、Redフェーズでもテストが実行可能（全パス）となり、テストの正しさ自体を検証できる。

### テスト実行（Redフェーズ確認）

```bash
cd ~/ros2_ws
python3 -m pytest src/zeuscar_imu/test/test_imu_publish.py -v
# 結果: 17 passed
```

---

## Step 4: 実装（Greenフェーズ）

### 目的

テストをパスする実際のモジュールを作成し、テストの import を仮実装から実モジュールに切り替える。

### 手順

1. **新規ファイル作成**: `zeuscar_imu/imu_publish_node.py`
   - モジュールレベル関数: `dps_to_rad_s()`, `g_to_m_s2()`, `create_imu_msg()`
   - 変換ヘルパー: `_dict_to_imu_msg()` （辞書 → sensor_msgs/msg/Imu）
   - ROS 2ノードクラス: `ImuPublishNode`
   - エントリポイント: `main()`

2. **テストファイル修正**: `test/test_imu_publish.py`
   - テストファイル内の仮実装（関数・定数）を削除
   - `from zeuscar_imu.imu_publish_node import ...` に切り替え
   - モッククラス・テストクラスは変更なし

3. **setup.py 修正**: エントリポイント追加
   ```python
   'imu_node = zeuscar_imu.imu_publish_node:main',
   ```

4. **ビルド**
   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select zeuscar_imu --symlink-install
   ```

5. **テスト実行（Greenフェーズ確認）**
   ```bash
   source install/setup.bash
   python3 -m pytest src/zeuscar_imu/test/test_imu_publish.py -v
   # 結果: 17 passed
   ```

### 実装上の設計判断

| 判断 | 内容 | 理由 |
|------|------|------|
| create_imu_msg の戻り値を辞書に | テストでsensor_msgsのimport不要 | テスト実行を軽量・高速に保つ |
| _dict_to_imu_msg でROS型に変換 | ノード内でのみ使用する内部関数 | テスト対象ロジックとROS依存を分離 |
| SensorDataQoS相当を手動構築 | QoSProfile で Best Effort, depth=5 を指定 | センサーデータのリアルタイム配信に適合 |

### ファイル構成（Green完了後）

```
zeuscar_imu/
├── __init__.py
├── icm42688_driver.py     ← Step 1で作成
├── imu_test_node.py       ← Step 1で修正
└── imu_publish_node.py    ← Step 4で新規作成

test/
├── test_imu_publish.py    ← Step 3で作成、Step 4でimport切替
├── test_copyright.py
├── test_flake8.py
└── test_pep257.py
```

---

## Step 5: launchファイルの作成

### 目的

ROS 2ノードをパラメータ付きで簡単に起動できるよう、launchファイルを作成する。

### 手順

1. **新規ファイル作成**: `launch/imu.launch.py`
   - 既存の `imu_test.launch.py` をテンプレートとして使用
   - `imu_node` 用にパラメータを設定

2. **ビルド**
   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select zeuscar_imu --symlink-install
   ```

### launchファイルのパラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| i2c_bus | 1 | I2Cバス番号 |
| i2c_address | 0x68 | I2Cデバイスアドレス |
| publish_rate_hz | 50.0 | パブリッシュ周波数 |
| frame_id | imu_link | IMUフレームID |
| linear_acceleration_stdev | 0.05 | 加速度標準偏差 |
| angular_velocity_stdev | 0.01 | 角速度標準偏差 |

### 起動方法

```bash
# デフォルトパラメータで起動
ros2 launch zeuscar_imu imu.launch.py

# パラメータを変更して起動（例: 周波数を100Hzに）
ros2 launch zeuscar_imu imu.launch.py publish_rate_hz:=100.0
```

### ポイント

- launchファイルでは全パラメータにデフォルト値を設定し、引数なしで起動可能にする
- 既存のlaunchファイルと同じパターンに合わせることで、プロジェクト内の一貫性を保つ

---

## Step 6: 実機テスト（次ステップ）

### 概要

ソフトウェアテストが完了したので、次は実際のIMUセンサーを使った動作確認を行う。

### 実行手順

```bash
# ターミナル1: IMUパブリッシュノードを起動
source ~/ros2_ws/install/setup.bash
ros2 run zeuscar_imu imu_node

# ターミナル2: トピックの確認
ros2 topic list | grep imu
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

### 確認項目

| 項目 | 期待値 |
|------|--------|
| トピック `/imu/data_raw` が存在する | ros2 topic list で確認 |
| 配信周波数 | 約 50 Hz |
| 静止時 linear_acceleration.z | 約 9.8 m/s² |
| 静止時 angular_velocity | 約 0 rad/s |
| orientation_covariance[0] | -1.0 |
| frame_id | `imu_link` |

### 必要条件

- IMUセンサーがI2C接続済み（ロボット静止状態でOK）
- smbus2ライブラリがインストール済み

---

## まとめ

### TDDサイクルの振り返り

| フェーズ | 作業内容 | 所要成果物 |
|---------|---------|-----------|
| 準備 | ドライバの分離（リファクタリング） | icm42688_driver.py |
| Red | 仕様書作成 → テスト作成（17件） | 仕様書、test_imu_publish.py |
| Green | 実装 → テストのimport切替 | imu_publish_node.py |
| 検証 | ビルド・テスト実行（17/17パス） | - |

### 学んだこと・ポイント

1. **仕様を先に決める**: テストを書く前に仕様書で設計を明確化することで、テストケースが自然に導かれる

2. **テストファイル内の仮実装**: テスト対象モジュールが未完成でもテストを書いて実行できる。テスト自体の正しさを先に検証できる

3. **純粋関数のテスト**: ROS 2ノードの起動なしでロジックをテストできるよう、ビジネスロジックを純粋関数として切り出す設計が重要

4. **モックの活用**: I2Cハードウェアに依存しないテストにより、開発環境を選ばず高速にテストを回せる

5. **段階的なリファクタリング**: ドライバの分離を先に行うことで、既存機能を壊さずに新機能の土台を作れる

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-02-06 | 初版作成（STORY-025 Red/Green/Launchフェーズ完了時点） |
