# オドメトリ（EKF）実装ガイド

STORY-011: IMUからのオドメトリ生成とSLAM動作確認

## 概要

このガイドでは、IMU（ICM-42688）のデータから `robot_localization` パッケージのEKF（拡張カルマンフィルタ）を使って
オドメトリを生成し、`slam_toolbox` でマッピングを行う手順を説明する。

### 完成後のシステム構成

```
[imu_node] → /imu/data_raw → [imu_filter_madgwick] → /imu/data → [ekf_filter_node] → /tf (odom→base_footprint)
                                                                                     → /odometry/filtered
[rplidar_node] → /scan → [slam_toolbox] → /map
                                         → /tf (map→odom)
```

### TFツリー

```
map (slam_toolbox)
 └─ odom (ekf_filter_node)
     └─ base_footprint (robot_state_publisher)
         └─ base_link
             ├─ laser_frame
             └─ imu_link
```

---

## 1. robot_localization のインストール

```bash
sudo apt update
sudo apt install ros-jazzy-robot-localization
```

`robot_localization` はEKF（拡張カルマンフィルタ）やUKF（無香料カルマンフィルタ）を提供するROS 2パッケージ。
複数のセンサーデータを融合してロボットの位置・姿勢を推定する。

---

## 2. EKF設定ファイルの設計

### 2.1 ファイル配置

```
zeuscar_bringup/config/ekf_params.yaml
```

`zeuscar_bringup` に配置する理由:
- EKFは既存ノード（`robot_localization`）を使うだけで、カスタムノード不要
- 設定ファイルのみなので、統合パッケージの役割に合致

### 2.2 フレーム設定

```yaml
odom_frame: odom              # オドメトリフレーム名
base_link_frame: base_footprint  # ベースフレーム名
world_frame: odom              # ワールドフレーム（odom = EKFが推定する座標系）
```

- `world_frame: odom` は「EKFが `odom` フレーム内で状態推定する」という意味
- `world_frame: map` にすると、EKFがマップ座標系で推定することになるが、通常はSLAM側に任せる

### 2.3 imu0_config の設計（最重要）

`imu0_config` は15個のbooleanで、IMUデータのどの成分をEKFに入力するかを制御する。

```yaml
imu0_config: [false, false, false,   # x, y, z (位置)
              false, false, false,   # roll, pitch, yaw (姿勢)
              false, false, false,   # vx, vy, vz (速度)
              true,  true,  true,    # wx, wy, wz (角速度) ← これだけ使用
              false, false, false]   # ax, ay, az (加速度)
```

**角速度のみ使用する理由:**

| 成分 | 使用 | 理由 |
|---|---|---|
| 位置 (x, y, z) | 不使用 | IMUから位置は取得できない |
| 姿勢 (roll, pitch, yaw) | 不使用 | `orientation_covariance[0] = -1`（未対応） |
| 速度 (vx, vy, vz) | 不使用 | IMUから速度は直接取得できない |
| 角速度 (wx, wy, wz) | **使用** | ジャイロデータ → ヨー角の変化を推定 |
| 加速度 (ax, ay, az) | 不使用 | 二重積分のドリフトが非常に大きい |

**ポイント**: 角速度をEKFに入力すると、EKFが時間積分してヨー角（向き）を推定する。
位置情報はゼロのまま（動かない）だが、`slam_toolbox` のスキャンマッチングが
実際の位置推定を担当するので問題ない。

### 2.4 プロセスノイズ

```yaml
process_noise_covariance: [15x15対角行列]
```

- 対角成分が大きいほど「状態が変化しやすい」とEKFが考える
- 角速度関連（wx, wy, wz）のノイズは 0.01〜0.02 に設定
- 将来ホイールオドメトリを追加する際は、位置・速度のノイズを調整する

---

## 3. launchファイルの設計

### 3.1 odometry.launch.py

```python
# EKFノードを起動するlaunchファイル
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
        ekf_params_file,
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ],
)
```

- `executable='ekf_node'` は `robot_localization` パッケージが提供するノード
- パラメータファイルと `use_sim_time` を渡す

### 3.2 zeuscar.launch.py への統合

`use_ekf` Launch Argument を追加:

```python
declare_use_ekf = DeclareLaunchArgument(
    'use_ekf',
    default_value='true',
    description='EKFオドメトリ起動フラグ'
)

odometry_launch = IncludeLaunchDescription(
    ...,
    condition=IfCondition(LaunchConfiguration('use_ekf')),
)
```

**デフォルト `true` の理由:**
- 軽量（CPU負荷ほぼゼロ）
- `odom` フレームはROS 2の多くのツールが前提とする
- SLAM使用時は必須

---

## 4. package.xml の更新

```xml
<exec_depend>robot_localization</exec_depend>
```

`exec_depend` に追加する理由:
- ビルド時には不要（Pythonパッケージのため）
- 実行時に `ekf_node` バイナリが必要

---

## 5. ビルドと単体テスト

```bash
cd ~/ros2_ws
colcon build --packages-select zeuscar_bringup
source install/setup.bash

# テスト実行
python3 -m pytest src/zeuscar_bringup/test/test_ekf_launch.py -v
```

### テスト項目（21件）

- `ekf_params.yaml` の存在と内容検証（13件）
  - フレーム名、IMUトピック、imu0_config の各フラグ
- `odometry.launch.py` の構造検証（5件）
  - ファイル存在、import可能、Launch Arguments、EKFノード含有
- `zeuscar.launch.py` の `use_ekf` 引数検証（2件）
- `package.xml` の依存確認（1件）

---

## 6. 実機テスト手順

### 6.1 EKF単体確認

```bash
# ターミナル1: TF + IMU + EKF起動（LiDAR・モーターなし）
ros2 launch zeuscar_bringup zeuscar.launch.py use_lidar:=false use_motor:=false use_ekf:=true

# ターミナル2: TF確認
ros2 run tf2_ros tf2_echo odom base_footprint

# ターミナル3: オドメトリトピック確認
ros2 topic echo /odometry/filtered --once
```

**確認ポイント:**
- `odom → base_footprint` TFが配信されている
- `/odometry/filtered` にメッセージが来ている
- ロボット静止時、ヨー角がドリフトしていない

### 6.2 SLAM統合確認

```bash
# 全ノード起動
ros2 launch zeuscar_bringup zeuscar.launch.py use_ekf:=true use_slam:=true

# TFツリー確認
ros2 run tf2_tools view_frames

# マップ確認
ros2 topic echo /map --once
```

### 6.3 マッピング走行

1. 全ノード起動後、ロボットを手動操作（`teleop_twist_keyboard` 等で `/cmd_vel` 送信）
2. 部屋を一周させてマップを構築
3. マップ保存:

```bash
# nav2_map_serverが必要（未インストールの場合）
sudo apt install ros-jazzy-nav2-map-server

# マップ保存
ros2 run nav2_map_server map_saver_cli -f ~/zeuscar_map
```

---

## 7. 設計判断まとめ

| 判断事項 | 決定 | 理由 |
|---|---|---|
| 加速度をEKFに入力するか | 不使用 | ドリフトが大きく、SLAMのスキャンマッチングで十分 |
| EKFのデフォルト起動 | ON | 軽量でodomフレームはROS 2標準 |
| 新規パッケージ作成 | なし | 設定+launchのみなのでbringupに統合 |
| launchファイル配置 | 独立ファイル | センサー取得とデータ処理は役割が異なる |

---

## 8. SLAM統合テスト実施結果（2026-02-10）

### 8.1 テスト環境

- FASTRTPS_DEFAULT_PROFILES_FILE による UDP専用通信（TSB-EKF-013対策）
- 起動コマンド:
  ```bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastrtps_udp_only.xml
  ros2 launch zeuscar_bringup zeuscar.launch.py use_ekf:=true use_slam:=true use_motor:=false use_rviz:=false
  ```

### 8.2 確認結果

| 確認項目 | 結果 | 詳細 |
|---------|------|------|
| 全ノード起動 | 合格 | robot_state_publisher, ekf, slam_toolbox, rplidar, imu の5ノード正常起動 |
| slam_toolbox ライフサイクル | 合格 | `active [3]` に自動遷移 |
| `/scan` データフロー | 合格 | ~6.8Hz安定 |
| `/imu/data_raw` データフロー | 合格 | ~50Hz安定 |
| `/odometry/filtered` データフロー | 合格 | ~30Hz安定（設定値通り） |
| `/map` データフロー | 合格 | 低頻度配信（静止時は更新間隔が長い）、68x80px / 0.05m解像度 |
| TF `map→odom` | 合格 | ~6.9Hz（slam_toolbox配信） |
| TF `odom→base_footprint` | 合格 | ~30Hz（EKF配信） |
| TFツリー完全性 | 合格 | `map→odom→base_footprint→base_link→{laser_frame, imu_link}` |
| マップ保存 | 合格 | `save_map`サービスで`.pgm`+`.yaml`出力（`nav2-map-server`必須、TSB-EKF-015参照） |

### 8.3 マップ保存手順

`nav2-map-server` のインストールが前提:
```bash
sudo apt-get install -y ros-jazzy-nav2-map-server
```

slam_toolbox のサービスでマップを保存:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/pi/zeuscar_map'}}"
# result=0 で成功。zeuscar_map.pgm と zeuscar_map.yaml が生成される
```

### 8.4 検出した問題

- **TSB-EKF-015**: `nav2-map-server` 未インストール時に `save_map` が `result=255` で失敗
  → `ros-jazzy-nav2-map-server` インストールで解決

---

## 9. RViz2可視化確認（2026-02-11）

### 9.1 テスト環境

- GUI環境: RealVNC Server経由（DISPLAY=:0）
- 起動コマンド:
  ```bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastrtps_udp_only.xml
  ros2 launch zeuscar_bringup zeuscar.launch.py use_ekf:=true use_slam:=true use_motor:=false use_rviz:=true
  ```

### 9.2 確認結果

| 確認項目 | 結果 | 詳細 |
|---------|------|------|
| LaserScan（/scan） | 合格 | 赤色ポイントクラウド、部屋形状・方向が正確、手かざしで即座に反応 |
| TFツリー | 合格 | 全5フレーム表示、フレーム名・座標軸が正常 |
| RobotModel | 合格 | /robot_descriptionからモデル表示 |
| Map（/map） | 合格 | SLAM占有格子マップ（68x80px）表示 |
| RViz2ノード起動 | 合格 | 6ノード同時稼働（robot_state_publisher, ekf, slam_toolbox, rplidar, imu, rviz2） |

### 9.3 既知の問題: IMU角速度のみのEKFによる点群ドリフト

**症状:** 数分後に前方の点群位置がずれる。後方の壁とMAPは正常。

**原因:**
1. LiDARモーター振動 → 不安定な設置台で機体が微振動
2. IMUが振動を角速度として検出 → EKFが積分してodomフレームがドリフト
3. RPi4のCPU高負荷（RViz2同時稼働）→ EKFレート超過、slam_toolboxメッセージドロップ
4. ホイールオドメトリ未統合 → 「静止している」制約がない

**対策方針:** `imu_filter_madgwick` 導入でIMU姿勢推定を改善（Section 10参照）

詳細: `docs/operations/troubleshooting/STORY-013_vnc_visualization.md` TSB-VIS-005

---

## 10. IMUフィルタ導入 — imu_filter_madgwick（2026-02-11 実装済み）

### 10.1 背景

TSB-VIS-005で、IMU角速度のみのEKFではLiDAR振動でドリフトする問題が判明。
`imu_filter_madgwick` を導入し、加速度+ジャイロから姿勢を推定してEKFに入力することで対策。

### 10.2 インストール

```bash
sudo apt install ros-jazzy-imu-filter-madgwick
```

### 10.3 データフロー（改善後）

```
改善前: /imu/data_raw（角速度のみ）→ EKF → ドリフト大
改善後: /imu/data_raw → madgwick → /imu/data（姿勢付き）→ EKF → ドリフト抑制
```

### 10.4 設定ファイル

**imu_filter_params.yaml** (`zeuscar_bringup/config/`):

| パラメータ | 値 | 理由 |
|---|---|---|
| `use_mag` | `false` | 6軸IMU（磁力計なし） |
| `publish_tf` | `false` | TFはEKFが担当 |
| `world_frame` | `"enu"` | REP-103準拠 |
| `gain` | `0.1` | 加速度補正の強さ（デフォルト値） |

**ekf_params.yaml** 変更点:

```yaml
# 変更前
imu0: /imu/data_raw
imu0_config: [false, false, false,
              false, false, false,   # 姿勢: 全て不使用
              ...
              true,  true,  true,    # 角速度のみ
              false, false, false]

# 変更後
imu0: /imu/data              # madgwickフィルタ済みトピック
imu0_config: [false, false, false,
              true,  true,  false,   # roll/pitch有効、yaw無効（磁力計なし）
              false, false, false,
              true,  true,  true,    # 角速度も継続使用
              false, false, false]
```

**設計判断: yaw orientationを無効にした理由:**
- 6軸IMU（磁力計なし）のためmadgwickのyawに絶対基準がない
- yawはEKFで角速度を積分して推定し、slam_toolboxがmap→odomで補正
- roll/pitchは重力方向が絶対基準となるため安定

### 10.5 sensors.launch.py の変更

IMU有効時（`use_imu:=true`）にmadgwickノードも自動起動:
```python
imu_filter_node = Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    remappings=[
        ('imu/data_raw', '/imu/data_raw'),
        ('imu/data', '/imu/data'),
    ],
    condition=IfCondition(LaunchConfiguration('use_imu')),
)
```

### 10.6 実機テスト結果

```
[imu_filter_madgwick_node] Starting ImuFilter
[imu_filter_madgwick_node] Using dt computed from message headers
[imu_filter_madgwick_node] Imu filter gain set to 0.100000
[imu_filter_madgwick_node] First IMU message received.
```

- `/imu/data` に orientation 付きメッセージ配信確認
- EKF `/odometry/filtered` に姿勢データ反映確認
- 全30テストパス（9件新規追加）

### 10.7 テスト項目（9件追加、計30件）

| クラス | テスト数 | 内容 |
|---|---|---|
| TestEkfParamsFile | +2 | orientation roll/pitch有効、yaw無効 |
| TestImuFilterParamsFile | +5 | imu_filter_params.yaml検証 |
| TestSensorsLaunchMadgwick | +1 | sensors.launch.pyにmadgwickノード |
| TestPackageXmlDependency | +1 | imu_filter_madgwick依存 |

---

## 11. 将来の拡張

### 11.1 ホイールオドメトリ追加

`ekf_params.yaml` に `odom0` を追加:

```yaml
odom0: /wheel_odom
odom0_config: [false, false, false,
               false, false, false,
               true,  true,  false,  # vx, vy のみ使用
               false, false, false,
               false, false, false]
```

これによりIMU（姿勢+角速度）とホイール（速度）のセンサーフュージョンが実現し、
静止時のドリフトが大幅に抑制される。

---

## 参考資料

- [robot_localization ドキュメント](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [slam_toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [imu_filter_madgwick](https://index.ros.org/p/imu_filter_madgwick/)
