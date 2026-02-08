# STORY-014/015 統合bringup設計仕様書

## 改訂履歴

| 版数 | 日付 | 作成者 | 変更内容 |
|---|---|---|---|
| 1.0 | 2026-02-07 | Design Agent | 初版作成 |

---

## 1. 既存構成サマリー

### 1.1 パッケージ別一覧表

#### zeuscar_description（TF定義）

| 項目 | 内容 |
|---|---|
| **ノード** | robot_state_publisher |
| **実行ファイル** | robot_state_publisher (ROS2標準) |
| **パブリッシュトピック** | /tf_static (tf2_msgs/TFMessage), /robot_description (std_msgs/String) |
| **サブスクライブトピック** | なし |
| **パラメータ** | robot_description (URDF内容), use_sim_time (デフォルト: false) |
| **launchファイル** | description.launch.py |
| **Launch Arguments** | use_sim_time |
| **URDFファイル** | urdf/zeuscar.urdf.xacro |
| **TFツリー** | base_footprint → base_link → laser_frame |
| **備考** | IMUリンク（imu_link）は未定義 |

#### zeuscar_lidar（LiDARセンサー）

| 項目 | 内容 |
|---|---|
| **ノード** | rplidar_node |
| **実行ファイル** | rplidar_composition (rplidar_ros) |
| **パブリッシュトピック** | /scan (sensor_msgs/LaserScan) |
| **サブスクライブトピック** | なし |
| **パラメータ** | serial_port (/dev/rplidar), serial_baudrate (115200), frame_id (laser_frame), inverted (false), angle_compensate (true) |
| **launchファイル** | lidar.launch.py |
| **Launch Arguments** | serial_port, frame_id |
| **configファイル** | config/rplidar_a1.yaml |
| **備考** | RPLIDAR A1M8対応 |

#### zeuscar_motor（モーター制御）

| 項目 | 内容 |
|---|---|
| **ノード** | motor_controller_node |
| **実行ファイル** | motor_controller_node (zeuscar_motor.motor_controller_node:main) |
| **パブリッシュトピック** | なし |
| **サブスクライブトピック** | /cmd_vel (geometry_msgs/Twist), /zeuscar/motor_cmd (std_msgs/String) |
| **パラメータ** | serial_port (/dev/ttyACM0), baud_rate (9600), cmd_vel_timeout (0.5), linear_threshold (0.1), angular_threshold (0.1) |
| **launchファイル** | motor.launch.py |
| **Launch Arguments** | serial_port, baud_rate |
| **configファイル** | config/motor_params.yaml |
| **備考** | Arduinoとシリアル通信でモーターコマンド送信 |

#### zeuscar_imu（IMUセンサー）

| 項目 | 内容 |
|---|---|
| **ノード** | imu_node |
| **実行ファイル** | imu_node (zeuscar_imu.imu_publish_node:main) |
| **パブリッシュトピック** | /imu/data_raw (sensor_msgs/Imu) |
| **サブスクライブトピック** | なし |
| **パラメータ** | i2c_bus (1), i2c_address (0x68), publish_rate_hz (50.0), frame_id (imu_link), linear_acceleration_stdev (0.05), angular_velocity_stdev (0.01) |
| **launchファイル** | imu.launch.py |
| **Launch Arguments** | なし（全てハードコード） |
| **configファイル** | なし |
| **備考** | ICM-42688 6軸IMU、orientation未対応（-1で無効化） |

#### zeuscar_slam（SLAM）

| 項目 | 内容 |
|---|---|
| **ノード** | slam_toolbox |
| **実行ファイル** | async_slam_toolbox_node (slam_toolbox) |
| **パブリッシュトピック** | /map (nav_msgs/OccupancyGrid), /tf (tf2_msgs/TFMessage, map→odom) |
| **サブスクライブトピック** | /scan (sensor_msgs/LaserScan), /tf (odom→base_footprint) |
| **パラメータ** | odom_frame (odom), map_frame (map), base_frame (base_footprint), scan_topic (/scan), mode (mapping), resolution (0.05), max_laser_range (12.0), その他多数 |
| **launchファイル** | slam.launch.py |
| **Launch Arguments** | use_sim_time |
| **configファイル** | config/slam_params.yaml |
| **備考** | オドメトリソース未実装（将来対応予定） |

#### zeuscar_bringup（統合起動）

| 項目 | 内容 |
|---|---|
| **現在の状態** | launchディレクトリが空、rviz設定ファイルのみ存在 |
| **目的** | 全パッケージの統合起動、システム全体のエントリポイント |

---

## 2. TF/トピック接続図

### 2.1 TFツリー（現状）

```
map (SLAMが生成、未実装)
 └─ odom (オドメトリソース未実装)
     └─ base_footprint (robot_state_publisher)
         └─ base_link (robot_state_publisher)
             └─ laser_frame (robot_state_publisher)
```

**不足要素:**
- `imu_link` がURDFに未定義（IMUノードは `imu_link` フレームを使用）
- `odom` フレームのパブリッシャーが未実装

### 2.2 TFツリー（設計目標）

```
map (SLAM使用時のみ)
 └─ odom (将来実装)
     └─ base_footprint
         └─ base_link
             ├─ laser_frame
             └─ imu_link (追加必要)
```

### 2.3 トピック接続図

```
[rplidar_node]
    └→ /scan (sensor_msgs/LaserScan)
        ├→ [slam_toolbox] (SLAM使用時)
        └→ [rviz2] (可視化)

[imu_node]
    └→ /imu/data_raw (sensor_msgs/Imu)
        └→ [rviz2] / [将来のセンサーフュージョン]

[motor_controller_node]
    ←─ /cmd_vel (geometry_msgs/Twist)
    ←─ /zeuscar/motor_cmd (std_msgs/String)

[robot_state_publisher]
    └→ /tf_static (tf2_msgs/TFMessage)
        └→ [全ノード]

[slam_toolbox] (SLAM使用時)
    ├→ /map (nav_msgs/OccupancyGrid)
    └→ /tf (map → odom)
```

---

## 3. 統合launch設計

### 3.1 ファイル構成

zeuscar_bringupパッケージに以下のlaunchファイルを作成する。

```
zeuscar_bringup/
├─ launch/
│  ├─ zeuscar.launch.py           # 統合メインlaunch（全機能起動）
│  ├─ sensors.launch.py           # センサー系グループlaunch
│  ├─ robot_base.launch.py        # ベースシステムlaunch（TF + Motor）
│  └─ slam.launch.py              # SLAM専用launch（オプション）
├─ config/
│  └─ zeuscar_params.yaml         # 統合パラメータ（将来拡張用）
└─ rviz/
   ├─ default.rviz                # 基本可視化設定
   └─ slam.rviz                   # SLAM用可視化設定
```

### 3.2 起動順序・依存関係

#### 起動順序の考慮事項

1. **TF基盤（robot_state_publisher）**: 最優先起動
   - 理由: 他のノードがTFツリーを参照するため

2. **センサーノード（LiDAR, IMU）**: TF基盤確立後
   - 理由: frame_idの参照先が必要

3. **モーターコントローラ**: 並行起動可能
   - 理由: TF依存なし

4. **SLAM**: センサーノードとTF確立後
   - 理由: /scanトピックとTFツリーが必須

#### 依存関係図

```
[robot_state_publisher] ← 依存
    ↓
    ├─ [rplidar_node]
    ├─ [imu_node]
    └─ [slam_toolbox] ← /scanとTF必要

[motor_controller_node] (独立起動可能)
```

### 3.3 グループ化設計

#### グループ1: ロボットベース（robot_base.launch.py）

- robot_state_publisher（TF）
- motor_controller_node（駆動制御）

**用途:** 最小構成での動作確認、モーター単独テスト

#### グループ2: センサー系（sensors.launch.py）

- rplidar_node（LiDAR）
- imu_node（IMU）

**用途:** センサー動作確認、キャリブレーション

#### グループ3: SLAM系（slam.launch.py）

- slam_toolbox（地図生成・自己位置推定）

**用途:** マッピング作業、既存地図での位置推定

#### 統合起動（zeuscar.launch.py）

- 上記全グループをIncludeLaunchDescriptionで統合
- 条件付き起動制御
- sensors_launch は TimerAction により遅延起動（TSB-INT-003対策）。`sensor_startup_delay` 引数（デフォルト3.0秒）で制御し、robot_base（TF + Motor）が先に起動・安定した後にセンサー系を起動する

### 3.4 パラメータ管理

#### 方針: **個別YAML維持 + 統合YAML追加**

**理由:**
1. 各パッケージの独立性維持（単体テスト可能）
2. パッケージ固有設定は個別YAMLで管理
3. システム全体の共通設定のみ統合YAMLで管理

#### パラメータ配置

| パラメータ種類 | 配置場所 | 例 |
|---|---|---|
| パッケージ固有 | 各パッケージのconfig/*.yaml | rplidar_a1.yaml, motor_params.yaml, slam_params.yaml |
| システム共通 | zeuscar_bringup/config/zeuscar_params.yaml | use_sim_time, namespace設定（将来） |
| Launch引数 | launchファイル内でDeclareLaunchArgument | serial_port, use_lidar, use_slam等 |

### 3.5 Launch Arguments

#### zeuscar.launch.py（統合メイン）

| 引数名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| use_sim_time | bool | false | シミュレーション時刻の使用 |
| use_lidar | bool | true | LiDAR起動フラグ |
| use_imu | bool | true | IMU起動フラグ |
| use_motor | bool | true | モーター起動フラグ |
| use_slam | bool | false | SLAM起動フラグ（デフォルトOFF） |
| use_rviz | bool | false | RViz自動起動フラグ |
| serial_port_motor | string | /dev/ttyACM0 | モーターシリアルポート |
| serial_port_lidar | string | /dev/rplidar | LiDARシリアルポート |
| sensor_startup_delay | float | 3.0 | センサー起動遅延（秒）。TSB-INT-003対策 |

#### robot_base.launch.py

| 引数名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| use_sim_time | bool | false | シミュレーション時刻の使用 |
| use_motor | bool | true | モーター起動フラグ |
| serial_port | string | /dev/ttyACM0 | Arduinoシリアルポート |
| baud_rate | int | 9600 | シリアル通信速度 |

#### sensors.launch.py

| 引数名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| use_lidar | bool | true | LiDAR起動フラグ |
| use_imu | bool | true | IMU起動フラグ |
| serial_port_lidar | string | /dev/rplidar | LiDARシリアルポート |

#### slam.launch.py

| 引数名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| use_sim_time | bool | false | シミュレーション時刻の使用 |
| params_file | string | (zeuscar_slam/config/slam_params.yaml) | SLAMパラメータファイル |
| mode | string | mapping | mapping or localization |

### 3.6 エラーハンドリング

#### デバイス接続エラー対応

1. **シリアルポート未接続（LiDAR, Motor）**
   - ノード起動失敗時のログ出力
   - 他ノードへの影響を最小化（起動継続）
   - 対策: udev rules設定ガイドをドキュメント化

2. **I2C通信エラー（IMU）**
   - smbus2ライブラリのエラー検出
   - 権限不足エラーの警告表示
   - 対策: i2c-devドライバ有効化、ユーザー権限設定

3. **TF接続不整合**
   - robot_state_publisher起動失敗時の全体停止
   - 理由: TFはシステム全体の基盤
   - 対策: URDFファイル検証機能の追加

#### ログレベル設定

- デフォルト: `screen` 出力
- デバッグ時: `--ros-args --log-level DEBUG`

---

## 4. ディレクトリ構造（変更後）

### zeuscar_bringup パッケージ

```
zeuscar_bringup/
├─ launch/
│  ├─ zeuscar.launch.py           # [新規] 統合メインlaunch
│  ├─ robot_base.launch.py        # [新規] TF + Motor
│  ├─ sensors.launch.py           # [新規] LiDAR + IMU
│  └─ slam.launch.py              # [新規] SLAM専用（zeuscar_slamのラッパー）
├─ config/
│  └─ zeuscar_params.yaml         # [新規] 統合パラメータ（将来拡張用）
├─ rviz/
│  ├─ default.rviz                # [既存] 基本可視化設定
│  └─ slam.rviz                   # [新規] SLAM用可視化設定（将来）
├─ package.xml
├─ setup.py                       # [変更] configディレクトリ追加
└─ setup.cfg
```

### zeuscar_description パッケージ（URDF変更）

```
zeuscar_description/
├─ urdf/
│  └─ zeuscar.urdf.xacro          # [変更] imu_link追加必要
└─ ...
```

**追加必要事項:**
- `imu_link` の定義（base_link → imu_link）
- IMU取り付け位置（実測値ベース）

---

## 5. テスト計画（テスト担当エージェントへの引き継ぎ用）

### 5.1 テスト項目

#### T1: 個別パッケージ起動テスト

| テストID | テスト内容 | 期待結果 |
|---|---|---|
| T1-1 | zeuscar_description単独起動 | robot_state_publisherが起動、/tf_staticパブリッシュ確認 |
| T1-2 | zeuscar_lidar単独起動 | rplidar_nodeが起動、/scanパブリッシュ確認 |
| T1-3 | zeuscar_imu単独起動 | imu_nodeが起動、/imu/data_rawパブリッシュ確認 |
| T1-4 | zeuscar_motor単独起動 | motor_controller_nodeが起動、/cmd_vel購読確認 |
| T1-5 | zeuscar_slam単独起動 | slam_toolboxが起動、/scanとTF接続確認 |

#### T2: 統合launch起動テスト

| テストID | テスト内容 | 期待結果 |
|---|---|---|
| T2-1 | robot_base.launch.py起動 | TFとMotorノード起動確認 |
| T2-2 | sensors.launch.py起動 | LiDARとIMUノード起動確認 |
| T2-3 | zeuscar.launch.py（全機能OFF） | 最小構成起動確認 |
| T2-4 | zeuscar.launch.py（センサーON） | TF + センサーノード起動確認 |
| T2-5 | zeuscar.launch.py（全機能ON） | 全ノード起動確認、トピック接続確認 |

#### T3: Launch Arguments動作テスト

| テストID | テスト内容 | 期待結果 |
|---|---|---|
| T3-1 | use_lidar:=false | LiDARノード不起動、他ノード正常 |
| T3-2 | use_imu:=false | IMUノード不起動、他ノード正常 |
| T3-3 | use_motor:=false | Motorノード不起動、他ノード正常 |
| T3-4 | use_slam:=true | SLAMノード起動、/map生成確認 |
| T3-5 | serial_port_motor:=/dev/custom | カスタムポート設定反映確認 |

#### T4: TF整合性テスト

| テストID | テスト内容 | 期待結果 |
|---|---|---|
| T4-1 | `ros2 run tf2_tools view_frames` | TFツリー正常、全フレーム接続 |
| T4-2 | `ros2 topic echo /tf_static` | base_footprint→base_link→laser_frame, imu_link確認 |
| T4-3 | LiDARフレーム参照 | /scanのframe_idがlaser_frameと一致 |
| T4-4 | IMUフレーム参照 | /imu/data_rawのframe_idがimu_linkと一致 |

#### T5: エラーハンドリングテスト

| テストID | テスト内容 | 期待結果 |
|---|---|---|
| T5-1 | LiDARデバイス未接続 | rplidar_nodeエラーログ、他ノード継続 |
| T5-2 | Motorデバイス未接続 | motor_controllerエラーログ、他ノード継続 |
| T5-3 | IMU I2C権限不足 | imu_nodeエラーログ、対策メッセージ表示 |
| T5-4 | URDF読み込み失敗 | robot_state_publisher起動失敗、明確なエラー表示 |

#### T6: RViz統合テスト

| テストID | テスト内容 | 期待結果 |
|---|---|---|
| T6-1 | RViz + TF表示 | RobotModel表示、TFツリー可視化 |
| T6-2 | RViz + LiDAR表示 | LaserScan点群表示 |
| T6-3 | RViz + IMU表示 | IMUフレーム表示（可能な場合） |
| T6-4 | RViz + SLAM表示 | マップ表示、ロボット位置表示 |

### 5.2 テスト環境要件

- Raspberry Pi 4（Ubuntu 24.04 + ROS 2 Jazzy）
- ZeusCar実機（Arduino接続済み）
- RPLIDAR A1M8接続済み（/dev/rplidar）
- ICM-42688 IMU接続済み（I2C）
- 開発用PC（RViz表示用、オプション）

### 5.3 テスト前提条件

1. 全パッケージがビルド済み（`colcon build`）
2. udev rulesが設定済み（/dev/rplidar, /dev/arduino等）
3. I2C権限設定済み（`sudo usermod -aG i2c $USER`）
4. ROS 2環境がsource済み（`. install/setup.bash`）

### 5.4 テストデータ収集

各テストで以下を記録する：
- ノード起動ログ（`ros2 launch`の出力）
- トピック一覧（`ros2 topic list`）
- TFツリー図（`ros2 run tf2_tools view_frames`）
- QoS設定確認（`ros2 topic info /scan --verbose`）

---

## 6. 実装上の注意事項

### 6.1 URDF変更（zeuscar_description）

**必須対応:**
- `imu_link` の追加
  - 親リンク: base_link
  - 取り付け位置: 実測値ベースで定義（未計測の場合は仮値 + コメント明記）
  - フレームID: `imu_link`（imu.launch.pyと一致させる）

**推奨対応:**
- IMUの視覚的表現（visual/collision）
- ドキュメントコメント充実

### 6.2 Launch引数の命名規則

- boolean型: `use_*`（例: use_lidar, use_slam）
- デバイスパス: `*_port` or `serial_port_*`（例: serial_port_lidar）
- 数値型: 単位を明確に（例: publish_rate_hz, timeout_sec）

### 6.3 後方互換性

- 既存の個別launchファイル（lidar.launch.py等）は維持
- 統合launchは各パッケージのlaunchをIncludeLaunchDescriptionで呼び出す
- パラメータ上書きは明示的に記述

### 6.4 ドキュメント更新

実装完了後、以下を更新すること：
- `docs/guides/bringup_integration_guide.md`（実装ガイド）
- `README.md`（起動方法の更新）
- `docs/setup_guide.md`（統合起動手順の追加）

---

## 7. 実装の優先順位

### Phase 1: 基盤整備（URDF + robot_base）

1. URDF変更（imu_link追加）
2. robot_base.launch.py作成（TF + Motor）
3. 動作確認テスト（T1-1, T2-1）

### Phase 2: センサー統合

4. sensors.launch.py作成（LiDAR + IMU）
5. 動作確認テスト（T2-2, T4-3, T4-4）

### Phase 3: 統合launch

6. zeuscar.launch.py作成（全機能統合）
7. Launch Arguments実装
8. 動作確認テスト（T2-3～T2-5, T3-1～T3-5）

### Phase 4: SLAM統合（オプション）

9. slam.launch.py作成（ラッパー）
10. use_slamフラグ実装
11. 動作確認テスト（T3-4, T6-4）

### Phase 5: ドキュメント整備

12. 実装ガイド作成
13. README更新
14. トラブルシューティング追記

---

## 8. 将来拡張の考慮事項

### 8.1 Nav2統合

- 統合launchにuse_nav2フラグ追加
- nav2パラメータ管理の追加
- オドメトリソースの統合

### 8.2 マルチロボット対応

- namespace対応（zeuscar_params.yamlで管理）
- TF prefix対応
- トピック名のリマッピング

### 8.3 Docker化

- 統合launchの環境変数対応
- デバイスマウント（/dev/rplidar等）の抽象化

### 8.4 CI/CD

- launchファイルの構文チェック
- TFツリー整合性自動検証
- シミュレーション環境でのlaunchテスト

---

## 9. 参考資料

- ROS 2 Launch Documentation: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- rplidar_ros: https://github.com/Slamtec/rplidar_ros
- 本プロジェクト要件定義書: docs/zeus_car_ros_2_jazzy_rpi_requirements.md
- 旧プロジェクト（参照専用）: reference/zeuscar-project/

---

## 10. 設計判断記録

### Q1: 個別YAMLと統合YAMLどちらを採用するか？

**判断:** 個別YAML維持 + 統合YAML追加（ハイブリッド方式）

**理由:**
- 各パッケージの独立性を保ちたい（単体テスト、他プロジェクトでの再利用）
- システム全体の共通設定は統合YAMLで一元管理したい
- 将来のマルチロボット対応を見据え、namespace等の共通設定を分離

### Q2: SLAMをデフォルトでONにするか？

**判断:** デフォルトOFF（use_slam:=false）

**理由:**
- 現時点でオドメトリソースが未実装のため、SLAMが正常動作しない
- センサー動作確認、モーター制御テスト等でSLAM不要なユースケースが多い
- 明示的に有効化することで、ユーザーが意図的に起動したことが明確

### Q3: Launch Argumentsの粒度はどうするか？

**判断:** 機能単位でのON/OFF制御 + 主要デバイスパスの上書き

**理由:**
- 機能単位のフラグ（use_lidar等）により、柔軟な構成テストが可能
- デバイスパス上書きにより、環境差異に対応
- 過度に細かいパラメータは各パッケージのYAMLで管理

### Q4: エラー時の起動継続 vs 全体停止どちらを採用するか？

**判断:** センサーノードは継続、TFノードは停止

**理由:**
- TF（robot_state_publisher）はシステム全体の基盤のため、失敗時は即座に停止すべき
- センサーノードは一部のみ動作しても価値があるケースが多い（例: LiDARのみでの動作確認）
- デバッグ時にログを確認しやすい

---

**以上**
