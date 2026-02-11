# STORY-011 オドメトリ統合・SLAM動作確認 実装計画書

## 改訂履歴

| 版数 | 日付 | 作成者 | 変更内容 |
|---|---|---|---|
| 1.0 | 2026-02-09 | PM | 初版作成 |

---

## 1. 目的・背景

### 1.1 目的

IMUデータ（`/imu/data_raw`）からEKF（拡張カルマンフィルタ）を用いてオドメトリを生成し、
`odom → base_footprint` TF変換を確立する。これにより `slam_toolbox` が動作可能となり、
LiDARスキャンからマップを生成・保存できることを確認する。

### 1.2 背景

- `slam_toolbox` は `odom → base_footprint` TF変換が必須
- 現在のTFツリー: `base_footprint → base_link → laser_frame, imu_link`
- `odom` フレームが存在しないためSLAMが起動できない
- IMUは `/imu/data_raw` を50Hzで配信済み（EPIC-007完了）
- ホイールエンコーダは未搭載のため、IMUのみでオドメトリを生成する

### 1.3 技術的前提

IMUのみのオドメトリは位置推定精度に限界がある（加速度二重積分のドリフト）。
ただし `slam_toolbox` のスキャンマッチングがこの誤差を補正するため、
マッピング用途では実用上問題ない。

---

## 2. スコープと非対象

### 2.1 スコープ

- `robot_localization` パッケージのインストールと設定
- EKF設定ファイル（`ekf_params.yaml`）の作成
- EKFノード用launchファイルの作成・統合
- `odom → base_footprint` TF変換の動作確認
- `slam_toolbox` によるマップ生成・保存確認

### 2.2 非対象

- ホイールオドメトリの実装（将来対応）
- IMU + ホイールオドメトリのセンサーフュージョン（将来対応）
- Nav2（ナビゲーション）の統合
- RViz表示確認（STORY-013で実施）

---

## 3. 現在のシステム構成

### 3.1 TFツリー（現状）

```
[存在しない]
  odom
    └─ base_footprint        ← この接続がない
        └─ base_link
            ├─ laser_frame
            └─ imu_link
```

### 3.2 TFツリー（目標）

```
map (slam_toolbox が配信)
 └─ odom (EKFが配信)
     └─ base_footprint
         └─ base_link
             ├─ laser_frame
             └─ imu_link
```

### 3.3 トピック接続図（目標）

```
[imu_node]
    └→ /imu/data_raw (sensor_msgs/Imu, 50Hz)
        └→ [ekf_filter_node]  ← 新規追加
              ├→ /odometry/filtered (nav_msgs/Odometry)
              └→ /tf (odom → base_footprint)

[rplidar_node]
    └→ /scan (sensor_msgs/LaserScan)
        └→ [slam_toolbox]
              ├→ /map (nav_msgs/OccupancyGrid)
              └→ /tf (map → odom)
```

### 3.4 IMU出力フォーマット（現在）

| フィールド | 値 | 備考 |
|---|---|---|
| `frame_id` | `imu_link` | |
| `orientation` | `[0, 0, 0, 0]` | 未対応 |
| `orientation_covariance[0]` | `-1.0` | orientation無効を示す |
| `angular_velocity` | x, y, z (rad/s) | ICM-42688ジャイロ |
| `linear_acceleration` | x, y, z (m/s²) | ICM-42688加速度 |
| QoS | `BEST_EFFORT`, depth=5 | SensorDataQoS相当 |

---

## 4. 入出力インタフェース・パラメータ

### 4.1 EKFノード（robot_localization）

#### 入力

| トピック/TF | 型 | ソース |
|---|---|---|
| `/imu/data_raw` | `sensor_msgs/Imu` | `imu_node` |

#### 出力

| トピック/TF | 型 | 説明 |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | フィルタリング済みオドメトリ |
| `/tf` (`odom → base_footprint`) | `tf2_msgs/TFMessage` | オドメトリTF変換 |

#### パラメータ（ekf_params.yaml）

```yaml
ekf_filter_node:
  ros__parameters:
    # フレーム設定
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom
    frequency: 50.0               # IMUと同じ周波数

    # IMUセンサー設定
    imu0: /imu/data_raw
    imu0_config: [false, false, false,   # x, y, z 位置: 使用しない
                  false, false, false,   # roll, pitch, yaw 姿勢: 使用しない（IMUにorientation無し）
                  false, false, false,   # vx, vy, vz 速度: 使用しない
                  true,  true,  true,    # ax_dot, ay_dot, az_dot 角速度: 使用する
                  false, false, false]   # ax, ay, az 加速度: 使用しない（ドリフト防止）
    imu0_remove_gravitational_acceleration: true

    # 初期共分散（初期不確実性）
    initial_estimate_covariance: [
      1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9]

    # プロセスノイズ共分散
    process_noise_covariance: [
      0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015]
```

**imu0_config の設計判断:**

| インデックス | 状態変数 | 使用 | 理由 |
|---|---|---|---|
| 0-2 | x, y, z (位置) | false | IMUから位置は取得不可 |
| 3-5 | roll, pitch, yaw (姿勢) | false | `orientation_covariance[0]=-1` のため無効 |
| 6-8 | vx, vy, vz (速度) | false | IMUから速度は直接取得不可 |
| 9-11 | wx, wy, wz (角速度) | **true** | ジャイロデータ（ヨー角推定に使用） |
| 12-14 | ax, ay, az (加速度) | false | 二重積分ドリフトが大きいため使用しない |

> **注記**: 加速度を使用しないため、EKFは角速度のみからヨー角を推定する。
> 位置・速度は常にゼロ（静止状態）と推定される。
> `slam_toolbox` のスキャンマッチングが実際の位置推定を担うため、
> オドメトリには「向き（heading）の変化」を提供できれば十分である。

---

## 5. ファイル構成（変更計画）

### 5.1 新規作成ファイル

```
ros2_ws/src/zeuscar_bringup/
├── config/
│   └── ekf_params.yaml              # [新規] EKF設定ファイル
└── launch/
    └── odometry.launch.py            # [新規] EKFノード起動用launch
```

### 5.2 変更ファイル

```
ros2_ws/src/zeuscar_bringup/
├── launch/
│   └── zeuscar.launch.py             # [変更] use_ekf引数追加、EKFノード統合
├── package.xml                       # [変更] robot_localization依存追加
└── setup.py                          # [変更] config/ekf_params.yaml追加（必要に応じて）
```

### 5.3 変更しないファイル

- `slam_params.yaml` — `odom_frame: odom`, `base_frame: base_footprint` は既に正しく設定済み
- `zeuscar.urdf.xacro` — `imu_link` は既に定義済み
- `slam.launch.py` — 変更不要
- `sensors.launch.py` — 変更不要

---

## 6. 実装フェーズ

### Phase 1: 環境準備

#### 1-1. robot_localization インストール

```bash
sudo apt install ros-jazzy-robot-localization
```

#### 1-2. package.xml 更新

`zeuscar_bringup/package.xml` に依存を追加:

```xml
<exec_depend>robot_localization</exec_depend>
```

### Phase 2: EKF設定・launchファイル作成（TDD）

#### 2-1. テスト作成（Red フェーズ）

テスト担当エージェントが以下のテストを作成:

- `test_ekf_launch.py` — EKF launchファイルの構造テスト
  - `ekf_params.yaml` の存在確認
  - `odometry.launch.py` の構造検証
  - `zeuscar.launch.py` の `use_ekf` 引数テスト
  - EKFノードの設定値検証

#### 2-2. 実装（Green フェーズ）

実装担当エージェントが以下を実装:

1. **`config/ekf_params.yaml`** — セクション4.1のパラメータ定義に基づく
2. **`launch/odometry.launch.py`** — `ekf_filter_node` を起動するlaunchファイル
3. **`launch/zeuscar.launch.py` 更新** — `use_ekf` Launch Argument追加

#### 2-3. ビルド確認

```bash
cd ~/ros2_ws
colcon build --packages-select zeuscar_bringup
source install/setup.bash
```

### Phase 3: 単体動作確認（EKF）

#### 3-1. EKF + IMU のみでの動作確認

```bash
# ターミナル1: TF基盤 + IMU + EKF
ros2 launch zeuscar_bringup zeuscar.launch.py use_lidar:=false use_motor:=false use_ekf:=true

# ターミナル2: TF確認
ros2 run tf2_ros tf2_echo odom base_footprint

# ターミナル3: オドメトリトピック確認
ros2 topic echo /odometry/filtered
```

**確認項目:**

| 確認事項 | 期待値 |
|---|---|
| `odom → base_footprint` TF存在 | TF変換が出力される |
| `/odometry/filtered` トピック | メッセージが配信される |
| ロボット静止時のヨー角 | ほぼ 0.0 で安定 |
| ロボット回転時のヨー角 | 回転に追従して変化 |

### Phase 4: SLAM動作確認

#### 4-1. 全ノード起動

```bash
ros2 launch zeuscar_bringup zeuscar.launch.py use_ekf:=true use_slam:=true
```

**起動されるノード:**

| ノード | パッケージ | 役割 |
|---|---|---|
| `robot_state_publisher` | `zeuscar_description` | TF (static) |
| `motor_controller_node` | `zeuscar_motor` | モーター制御 |
| `rplidar_node` | `rplidar_ros` | LiDARデータ取得 |
| `imu_node` | `zeuscar_imu` | IMUデータ取得 |
| `ekf_filter_node` | `robot_localization` | オドメトリ推定 |
| `slam_toolbox` | `slam_toolbox` | マッピング |

#### 4-2. TFツリー全体確認

```bash
ros2 run tf2_tools view_frames
```

期待されるTFツリー:
```
map → odom → base_footprint → base_link → laser_frame
                                         → imu_link
```

#### 4-3. マッピング実行

1. ロボットを手動操作（`/cmd_vel` でTwistメッセージ送信）
2. `slam_toolbox` がスキャンマッチングでマップを構築
3. `/map` トピックでマップが配信されることを確認

```bash
# マップ確認
ros2 topic echo /map --once
```

#### 4-4. マップ保存

```bash
# マップ保存（YAML + PGM形式）
ros2 run nav2_map_server map_saver_cli -f ~/zeuscar_map
```

> **注記**: `nav2_map_server` が未インストールの場合は別途インストールする:
> ```bash
> sudo apt install ros-jazzy-nav2-map-server
> ```

---

## 7. Launch Arguments 設計

### 7.1 zeuscar.launch.py への追加

| 引数名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `use_ekf` | bool | `true` | EKFオドメトリ起動フラグ |

**デフォルト `true` の理由:**
- EKFは軽量でリソース消費が少ない
- `odom` フレームは多くのROS 2ツール（rviz2, nav2等）が前提とする
- SLAM使用時は必須、非使用時も害がない

### 7.2 odometry.launch.py

| 引数名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `use_sim_time` | bool | `false` | シミュレーション時刻 |
| `ekf_params_file` | string | `(パッケージ内デフォルト)` | EKF設定ファイルパス |

---

## 8. 受け入れ基準

### AC-1: EKF動作

- [ ] `ekf_filter_node` が正常起動する
- [ ] `/odometry/filtered` トピックが配信される
- [ ] `odom → base_footprint` TF変換が存在する
- [ ] ロボット静止時にヨー角がドリフトしない（±0.1 rad/min 以内）

### AC-2: SLAM動作

- [ ] `slam_toolbox` が正常起動する（TFエラーなし）
- [ ] `/map` トピックが配信される
- [ ] ロボット走行中にマップが更新される
- [ ] `map → odom → base_footprint` のTFチェーンが完成する

### AC-3: マップ保存

- [ ] `map_saver_cli` でマップファイル（.yaml + .pgm）が保存できる
- [ ] 保存されたマップが部屋の形状を概ね反映している

### AC-4: 統合起動

- [ ] `zeuscar.launch.py use_ekf:=true use_slam:=true` で全ノードが一発起動する
- [ ] `use_ekf:=false` でEKFなしでの起動が可能（後方互換性）
- [ ] 既存テストが全てパスする

---

## 9. テスト・検証計画

### 9.1 ユニットテスト（自動）

| テストID | テスト内容 | ファイル |
|---|---|---|
| T-EKF-01 | `ekf_params.yaml` の存在確認 | `test_ekf_launch.py` |
| T-EKF-02 | `odometry.launch.py` の構造検証 | `test_ekf_launch.py` |
| T-EKF-03 | `zeuscar.launch.py` の `use_ekf` 引数存在確認 | `test_launch_files.py`（既存拡張） |
| T-EKF-04 | EKFパラメータの値検証（フレーム名等） | `test_ekf_launch.py` |
| T-EKF-05 | `package.xml` に `robot_localization` 依存の存在 | `test_ekf_launch.py` |

### 9.2 実機テスト（手動）

| テストID | テスト内容 | 期待結果 |
|---|---|---|
| T-INT-01 | EKF単体起動 | `odom → base_footprint` TFが配信される |
| T-INT-02 | ロボット静止状態でヨー角確認 | ドリフトが小さい（±0.1 rad/min以内） |
| T-INT-03 | ロボット回転時のヨー角追従 | 回転方向に応じたヨー角変化 |
| T-INT-04 | SLAM + EKF統合起動 | TFチェーン完成、`/map` 配信開始 |
| T-INT-05 | マッピング走行 | マップが壁形状を反映して成長 |
| T-INT-06 | マップ保存 | `.yaml` + `.pgm` ファイルが生成される |
| T-INT-07 | `use_ekf:=false` での起動 | EKFなしで他ノードは正常起動 |

---

## 10. 依存関係

### 10.1 パッケージ依存

| パッケージ | 用途 | 状態 |
|---|---|---|
| `ros-jazzy-robot-localization` | EKF（オドメトリ生成） | **未インストール** |
| `ros-jazzy-slam-toolbox` | SLAM（マッピング） | インストール済み |
| `ros-jazzy-nav2-map-server` | マップ保存 | **要確認** |

### 10.2 ストーリー依存

| 依存先 | 状態 | 必要なもの |
|---|---|---|
| STORY-014/015（bringup統合） | Done | `zeuscar.launch.py` |
| STORY-025（IMUパブリッシュ） | Done | `/imu/data_raw` トピック |
| STORY-005（LiDAR動作確認） | Done | `/scan` トピック |
| STORY-008（TFツリー） | Done | `base_footprint → base_link → laser_frame` |

---

## 11. リスクと対策

| リスク | 影響度 | 対策 |
|---|---|---|
| IMUのみではヨー角ドリフトが大きい | 中 | `slam_toolbox`のスキャンマッチングで補正。`minimum_travel_distance/heading`を調整 |
| EKFとIMUのQoS不整合 | 高 | `robot_localization`のデフォルトはRELIABLE。IMUはBEST_EFFORT。QoS設定の確認が必要 |
| 全ノード同時起動時のリソース不足（RPi4） | 中 | 既存のTimerAction遅延起動を活用。EKFはIMUと同時起動で問題ない |
| `slam_toolbox` がTF待ちでタイムアウト | 中 | `transform_timeout: 0.2` は既設定。EKF起動順序で対応 |

### 11.1 QoS整合性の確認事項

`robot_localization` のデフォルトQoSと `imu_node` のQoSが一致するか要確認。
不一致の場合、EKF設定で `imu0_queue_size` やQoSプロファイルの調整が必要。

---

## 12. 設計判断記録

### Q1: 加速度データをEKFに入力するか？

**判断:** 使用しない（`imu0_config` の加速度フラグを `false`）

**理由:**
- 加速度の二重積分は累積誤差が非常に大きい
- ホイールオドメトリがないため、加速度バイアスの推定が困難
- `slam_toolbox` のスキャンマッチングが位置推定を担うため、不要

### Q2: EKFのデフォルトをONにするか？

**判断:** デフォルト ON（`use_ekf:=true`）

**理由:**
- 軽量でリソース消費が少ない
- `odom` フレームはROS 2エコシステムの標準
- SLAM以外のツール（rviz2のFixed Frameなど）でも有用

### Q3: launchファイルの配置場所は？

**判断:** `zeuscar_bringup/launch/odometry.launch.py` として独立ファイル作成

**理由:**
- `sensors.launch.py`（センサーデータ取得）とは役割が異なる
- オドメトリは「センサーデータの処理」であり、グループ分けとして独立が適切
- `zeuscar.launch.py` から `IncludeLaunchDescription` で統合

### Q4: 新規パッケージ（zeuscar_odometry等）を作成するか？

**判断:** 作成しない。`zeuscar_bringup` に統合する

**理由:**
- EKFは `robot_localization` の既存ノードを使うだけで、カスタムノードは不要
- 設定ファイルとlaunchファイルのみなので、`zeuscar_bringup` の役割に合致
- パッケージ数を不必要に増やさない

---

## 13. 作業見積もり

| フェーズ | 作業内容 |
|---|---|
| Phase 1 | `robot_localization` インストール、`package.xml` 更新 |
| Phase 2 | テスト作成（Red）→ 実装（Green）→ ビルド確認 |
| Phase 3 | EKF単体の実機動作確認 |
| Phase 4 | SLAM統合テスト、マッピング走行、マップ保存 |
| Phase 5 | ドキュメント整備（ジャーナル、ガイド、セットアップガイド更新） |

---

## 14. 参考資料

- [robot_localization wiki](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [robot_localization ROS 2](https://github.com/cra-ros-pkg/robot_localization)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- 本プロジェクト設計仕様書: `docs/operations/specs/STORY-014-015_bringup_design.md`

---

**以上**
