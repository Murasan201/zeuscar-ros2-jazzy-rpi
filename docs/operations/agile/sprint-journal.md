# スプリントジャーナル

## 概要

本ドキュメントはスプリント計画・進行ログの要約と各バックログジャーナルへのリンク集を管理する。

---

## プロジェクト初期化フェーズ

### 2026-01-12: プロジェクト基盤整備

#### 実施内容

1. **CLAUDE.mdルールファイル作成**
   - 業務開始時の確認事項（要件定義書参照）
   - ドキュメント管理ルール（docs/集約、README.md索引更新）
   - 開発プロセス（アジャイル・TDD、マルチエージェント体制）
   - 実装ルール（Python、コーディングガイドライン、コメントスタイル）

2. **docs/ディレクトリ整備**
   - 既存ドキュメントをdocs/に集約
   - docs/README.md索引を作成

3. **operations/ディレクトリ構成作成**
   - agile/ - バックログ、スプリントジャーナル、アクションアイテム
   - specs/ - 仕様書
   - pm-briefs/ - PMエスカレーション
   - work-orders/ - 作業依頼書
   - troubleshooting/ - トラブルシューティング
   - templates/ - テンプレート

4. **プロダクトバックログ初版作成**
   - 要件定義書に基づきエピック・ストーリーを定義
   - EPIC-001〜005、STORY-001〜015を登録

#### 成果物

- `CLAUDE.md`
- `docs/README.md`
- `docs/operations/agile/product-backlog.md`
- `docs/operations/agile/sprint-journal.md`
- `docs/operations/agile/action-items.md`
- `docs/operations/information-requests.md`
- 各種READMEとテンプレート

#### 次のアクション

- スプリント1の計画
- 優先バックログの選定と着手

---

## 現在のスプリント

### スプリント 1

- **期間**: 2026-01-12〜
- **ゴール**: ROS 2環境構築完了、LiDAR統合開始
- **ステータス**: 進行中

#### 選定バックログ

| ID | タイトル | 担当 | ステータス | ジャーナル |
|---|---|---|---|---|
| STORY-001 | ROS 2ワークスペース作成 | - | Done | - |
| STORY-002 | 基本パッケージ構成作成 | - | Done | - |
| STORY-003 | zeuscar_lidarパッケージ作成 | - | Done | - |
| STORY-004 | LiDAR launchファイル作成 | - | Done | - |
| STORY-005 | /scanトピック動作確認 | - | Done | EPIC-002 |
| STORY-006 | zeuscar_descriptionパッケージ作成 | - | Done | EPIC-003 |
| STORY-007 | URDF作成 | - | Done | EPIC-003 |
| STORY-008 | TFツリー設計・実装 | - | Done | EPIC-003 |
| STORY-009 | zeuscar_slamパッケージ作成 | - | Done | EPIC-004 |
| STORY-010 | slam_toolbox設定 | - | Done | EPIC-004 |
| STORY-011 | マップ生成動作確認 | - | Done | ※全項目合格（2026-02-10） |
| STORY-012 | RViz設定ファイル作成 | - | Done | EPIC-005 |
| STORY-013 | LaserScan/TF/RobotModel表示確認 | - | Done | EPIC-005 |
| STORY-016 | zeuscar_motorパッケージ作成 | - | Done | EPIC-006 |
| STORY-017 | motor_controller_node実装 | - | Done | EPIC-006 |
| STORY-018 | cmd_vel対応 | - | Done | EPIC-006 |
| STORY-019 | udevルール設定（Arduino） | - | Done | EPIC-006 |
| STORY-020 | Arduino駆動系動作確認 | - | Done | EPIC-006 |
| STORY-021 | zeuscar_imuパッケージ作成 | - | Done | [EPIC-007](journals/EPIC-007.md) |
| STORY-022 | IMUドライバ実装（ICM-42688） | - | Done | [EPIC-007](journals/EPIC-007.md) |
| STORY-023 | IMUテストノード実装 | - | Done | [EPIC-007](journals/EPIC-007.md) |
| STORY-024 | IMU動作確認 | - | Done | [EPIC-007](journals/EPIC-007.md) |
| STORY-025 | IMUデータパブリッシュノード実装 | - | Done | [EPIC-007](journals/EPIC-007.md) |
| STORY-014 | zeuscar_bringupパッケージ作成 | - | Done | [STORY-014-015](journals/STORY-014-015.md) |
| STORY-015 | 統合launchファイル作成 | - | Done | [STORY-014-015](journals/STORY-014-015.md) |

#### 完了した作業

**2026-01-12: EPIC-001完了**

1. **ROS 2 Jazzyインストール**
   - ros-jazzy-desktopをインストール
   - ~/.bashrcにsource設定を追加

2. **colconビルドツールインストール**
   - python3-colcon-common-extensionsをインストール

3. **ワークスペース作成**
   - ~/ros2_ws/src/ ディレクトリ構造を作成
   - .colcon_defaults.yamlでビルド設定を定義

4. **ZeusCar用パッケージ作成**
   - zeuscar_bringup（起動用）
   - zeuscar_description（URDF/TF）
   - zeuscar_lidar（LiDARラッパー）
   - zeuscar_slam（SLAM設定）

5. **ビルド確認**
   - colcon buildで4パッケージのビルド成功

6. **セットアップガイド更新**
   - docs/setup_guide.mdにワークスペース作成手順を追記

**2026-01-12: EPIC-002 LiDAR統合**

1. **rplidar_rosパッケージインストール**
   - ros-jazzy-rplidar-rosをインストール

2. **zeuscar_lidarパッケージ更新**
   - lidar.launch.pyを作成（rplidar_rosをラップ）
   - rplidar_a1.yamlで設定パラメータを定義
   - 99-rplidar.rulesでudevルールを作成
   - setup.pyにlaunch/configのインストール設定を追加
   - package.xmlに依存関係を追加

3. **セットアップガイド更新**
   - Section 4（LiDARのセットアップ）を追記

**2026-01-19: EPIC-002 LiDAR統合完了**

1. **udevルール設定**
   - /etc/udev/rules.d/99-rplidar.rules を作成
   - /dev/rplidar シンボリックリンクを確認

2. **LiDAR動作確認**
   - ros2 launch zeuscar_lidar lidar.launch.py 起動成功
   - RPLIDAR SDK Version: 1.12.0
   - Firmware Ver: 1.29, Hardware Rev: 7
   - Scan mode: Sensitivity, max_distance: 12.0m

3. **/scanトピック確認**
   - Type: sensor_msgs/msg/LaserScan
   - frame_id: laser_frame（正しく設定）
   - Publisher count: 2

**2026-01-19: EPIC-003 TF/URDF設計完了**

1. **ロボット本体寸法の実測**
   - 全長（X軸方向）: 163mm
   - 全幅（Y軸方向）: 177mm

2. **LiDAR取り付け位置の実測**
   - X位置: +3.5mm（ロボット中心から前方）
   - Y位置: -4.5mm（ロボット中心から右方向）
   - Z位置: 235mm（地面からセンサー中央）
   - Yaw角: +90°（設計値、左方向に90度回転）

3. **TF変換パラメータ確定**
   - x: +0.0035m, y: -0.0045m, z: +0.235m
   - roll: 0, pitch: 0, yaw: +1.5708 rad

4. **仕様書更新**
   - IR-001_epic003_robot_dimensions.md にすべての実測値を記録

5. **zeuscar_descriptionパッケージ作成**
   - URDF（zeuscar.urdf.xacro）を作成
   - description.launch.pyを作成
   - xacroパッケージをインストール（ros-jazzy-xacro）
   - ParameterValue問題を修正（ROS 2 Jazzy固有の対応）

6. **ビルドと動作確認**
   - colcon buildでzeuscar_descriptionをビルド
   - robot_state_publisherが正常起動を確認
   - /tf_staticトピックで正しいTF変換を確認
     - base_footprint → base_link: z=0.05m
     - base_link → laser_frame: x=0.0035m, y=-0.0045m, z=0.185m, yaw=90°

7. **トラブルシューティング記録**
   - EPIC-003_tf_urdf.md を作成
   - TSB-003-001: xacroパッケージ未インストール
   - TSB-003-002: robot_descriptionパラメータのYAMLパースエラー

**2026-01-19: EPIC-004/EPIC-005 SLAM・可視化準備**

1. **slam_toolboxインストール**
   - ros-jazzy-slam-toolboxをインストール
   - 依存パッケージ（Ceres Solver等）も自動インストール

2. **zeuscar_slamパッケージ作成**
   - slam_params.yaml設定ファイル作成（ZeusCar用にカスタマイズ）
   - slam.launch.pyを作成
   - setup.pyとpackage.xmlを更新

3. **RViz設定ファイル作成**
   - zeuscar_bringup/rviz/zeuscar.rviz作成
   - Grid、RobotModel、TF、LaserScan、Map表示を設定
   - setup.pyを更新してrvizディレクトリをインストール

4. **ビルド確認**
   - zeuscar_slam、zeuscar_bringupのビルド成功

5. **ドキュメント整備**
   - セットアップガイドにSection 6（SLAM）、Section 7（RViz）を追加
   - トラブルシューティング 8.8-8.10を追加
   - EPIC-004_slam.md、EPIC-005_visualization.mdを作成

6. **残タスク**
   - STORY-011: オドメトリ統合後にマップ生成確認
   - STORY-013: ディスプレイ接続後に表示確認
   - IMU（ICM42688）到着待ち（来週予定）

**2026-01-24: EPIC-006 Arduino駆動系統合開始**

1. **参照実装の調査**
   - reference/zeuscar-project/を分析
   - Arduino側ファームウェア（raspi-ctrl-v_2_00.ino）の仕様確認
   - Raspberry Pi側subscriber.pyの仕様確認
   - PC側publisher.pyの仕様確認

2. **通信プロトコル確認**
   - シリアル通信: 9600bps, ASCII + 改行区切り
   - サポートコマンド: FORWARD, BACKWARD, LEFT, RIGHT等11種類
   - メカナムホイール4輪構成

3. **仕様書作成**
   - docs/operations/specs/EPIC-006_arduino_interface.md 作成
   - ROS 2インタフェース設計（/cmd_vel, /zeuscar/motor_cmd）
   - パラメータ設計（serial_port, baud_rate, timeout等）
   - 参照実装からの改善点を定義

4. **バックログ更新**
   - EPIC-006追加
   - STORY-016〜020を登録

5. **ルールファイル更新**
   - CLAUDE.mdにreference/ディレクトリのルールを追加
   - .gitignoreにreference/を追加

6. **zeuscar_motorパッケージ実装（STORY-016〜019）**
   - zeuscar_motorパッケージ作成
   - motor_controller_node.py実装
     - /cmd_vel (geometry_msgs/Twist) 購読対応
     - /zeuscar/motor_cmd (std_msgs/String) 購読対応
     - Twist→コマンド変換ロジック実装
     - タイムアウト機能（自動停止）
   - 設定ファイル（motor_params.yaml）作成
   - launchファイル（motor.launch.py）作成
   - udevルール（99-arduino.rules）作成
   - 単体テスト13件全てパス

7. **ビルド確認**
   - colcon buildでzeuscar_motorのビルド成功
   - pytest全13テストケースがパス

**2026-01-24: EPIC-006 Arduino駆動系統合完了**

1. **motor_test_node追加**
   - 全方向モーターテストノードを実装
   - 10方向すべてを順番にテスト
   - duration/pauseパラメータで調整可能

2. **udevルールインストール**
   - 99-arduino.rulesを/etc/udev/rules.d/にコピー
   - /dev/arduinoシンボリックリンクを確認

3. **実機テスト完了（STORY-020）**
   - motor_controller_nodeを起動
   - motor_test_nodeで全方向テスト実行
   - 前進、後退、左右移動、斜め移動、旋回すべて動作確認

**2026-02-03: EPIC-007 IMU統合開始**

1. **環境セットアップ**
   - i2c-toolsインストール
   - python3-pipインストール
   - smbus2ライブラリインストール（PEP 668対応: `--break-system-packages`）
   - I2Cデバイス確認（0x68で検出）

2. **zeuscar_imuパッケージビルド**
   - colcon buildでビルド成功

3. **IMUテスト実行（1回目）**
   - 時刻: 21:15:45
   - 結果: 部分的成功
   - WHO_AM_I = 0x47（正常）
   - ジャイロスコープ: 正常動作（左旋回+5.76dps、右旋回-7.33dps）
   - 加速度X軸: 正常動作（前進+0.028g、後退-0.014g）
   - **問題検出**: 加速度Z軸が期待値の約1/8（0.126g vs 1.0g）

4. **問題分析と修正**
   - 原因: ACCEL_SCALEが±2g用（16384）だが、レジスタ設定は±16g
   - 修正: ACCEL_SCALE = 2048.0（±16g用）に変更
   - パッケージ再ビルド完了

5. **IMUテスト実行（2回目）**
   - 状態: バッテリー切れにより中断
   - 次回: バッテリー充電後に再実行予定

6. **ドキュメント整備**
   - docs/setup_guide.md - Section 9（IMUセットアップ）追加
   - docs/setup_guide.md - Section 10.14-10.18（IMUトラブルシューティング）追加
   - docs/operations/troubleshooting/IMU_test_report_20260203_211545.md作成
   - docs/operations/agile/journals/EPIC-007.md作成
   - docs/operations/agile/product-backlog.md - EPIC-007追加

**2026-02-04: EPIC-007 IMUテスト2回目完了（全項目合格）**

1. **IMUテスト実行（2回目）**
   - バッテリー充電後に再実行
   - 結果: **全項目合格**
   - 静止状態: accel_z = +1.009g（期待: +1.0g）→ スケールファクター修正成功
   - 前進: accel_x = +0.094g、後退: accel_x = -0.076g → 加速度検出正常
   - 左旋回: gyro_z = +7.42dps、右旋回: gyro_z = -10.21dps → ジャイロ検出正常

2. **バックログ更新**
   - STORY-023（IMUテストノード実装）→ Done
   - STORY-024（IMU動作確認）→ Done

**2026-02-06〜07: STORY-025 IMUデータパブリッシュノード実装**

1. **TDD Red→Green**
   - テスト17件作成 → 実装 → 全パス
   - imu_publish_node: ICM-42688から50Hzで/imu/data_rawを配信

2. **EPIC-007完了**
   - STORY-021〜025 全Done

**2026-02-07: STORY-014/015 統合bringup設計・実装**

1. **設計仕様書作成**
   - 階層化launch構成（個別→グループ→統合の3層）
   - Launch Arguments設計（use_lidar, use_imu, use_motor等8引数）

2. **TDD Red→Green**
   - URDF imu_link追加テスト12件 + launchファイルテスト39件 = 51件全パス
   - robot_base.launch.py, sensors.launch.py, zeuscar.launch.py作成

**2026-02-08: 実機統合テスト実施・TSB-INT-003対策**

1. **実機統合テスト S0-S5 全6ステージ実施**
   - S0 事前確認: 合格
   - S1 最小構成（TFのみ）: 合格
   - S2 センサー追加（LiDAR + IMU）: 合格
   - S3 モーター追加: 合格
   - S4 全機能統合: 合格（TimerAction導入後）
   - S5 Launch Arguments動作確認: 合格
   - 検出問題3件（TSB-INT-001〜003）全て解決済

2. **TSB-INT-003対策（TDD）**
   - 問題: 全ノード同時起動時にLiDAR初期化失敗
   - 対策: TimerActionによるセンサー3秒遅延起動
   - テスト4件追加 → 55/55全パス
   - 実機確認: zeuscar.launch.py一発起動で全ノード正常動作

3. **ドキュメント整備**
   - テスト計画書、トラブルシューティング、実装ガイド、設計仕様書を更新

**2026-02-09: STORY-011 オドメトリ統合・SLAM動作確認開始**

1. **Phase 1: robot_localization インストール・EKF設定**
   - `ros-jazzy-robot-localization` インストール
   - TDD Red: `test_ekf_launch.py` 21テスト作成
   - TDD Green: `ekf_params.yaml`, `odometry.launch.py` 作成
   - `zeuscar.launch.py` に `use_ekf` 引数追加
   - `package.xml` に依存追加
   - 全76テストパス（21新規 + 55既存）

2. **Phase 2: 実機テスト — EKF単体（T-INT-01）: 合格**
   - `ros2 launch zeuscar_bringup zeuscar.launch.py use_ekf:=true use_slam:=false use_motor:=false`
   - `odom → base_footprint` TF配信確認済み
   - `/odometry/filtered` トピック正常

3. **Phase 3: 実機テスト — SLAM統合（T-INT-04）: 失敗→修正→再テスト**
   - **問題1**: slam_toolboxがunconfigured状態で停止 → **解決**（TSB-EKF-008）
     - Jazzy版はライフサイクルノード、`slam.launch.py` を LifecycleNode に変更
   - **問題2**: LifecycleNode に namespace必須 → **解決**（TSB-EKF-009）
   - **問題3**: RPi4メモリ圧迫で電源断 → **解決**（TSB-EKF-010）
     - stack_size 40MB→10MB、EKF 50Hz→30Hz
   - **問題4**: rplidar_compositionのノード名変更で/scanが消失 → **原因特定**（TSB-EKF-012）
     - `name='rplidar_node'` → `name='rplidar_composition'` に変更が必要

4. **Phase 3.5: DDS通信障害発覚 → 外部有識者により解決**
   - ノード発見は正常だがトピックデータ送受信が一切不可
   - `ros2 topic echo` が全トピックでタイムアウト
   - 基本的な `std_msgs/String` の pub/sub すら失敗
   - **原因**: FastRTPS共有メモリ（SHM）トランスポートの障害（TSB-EKF-013）
   - **対策**: UDP専用XMLプロファイル作成 + `.bashrc` に環境変数設定
   - **解決確認**: `std_msgs/String` のpub/subテストで通信復旧を確認

5. **Phase 4: DDS解決後の再テスト — LiDAR問題で中断**
   - TSB-EKF-012修正適用: `lidar.launch.py` のノード名を `rplidar_composition` に変更、ビルド完了
   - `/scan` パブリッシャー登録成功（Publisher count: 1）を確認
   - しかし `/scan` のデータフロー（hz/echo）は未確認のまま中断
   - `killall -9` による強制終了でLiDARシリアルポートが不安定化（TSB-EKF-014）
   - **次回再開時にUSBケーブル再接続が必要**

6. **ドキュメント整備**
   - 実装計画書: `docs/operations/specs/STORY-011_odometry_slam_verification.md`
   - 実装ガイド: `docs/guides/odometry_ekf_implementation_guide.md`
   - トラブルシューティング: `docs/operations/troubleshooting/STORY-011_odometry_ekf.md`（TSB 14件）
   - セットアップガイド: Section 10（オドメトリ）追加

**2026-02-10: STORY-011 SLAM統合テスト完了 — 全項目合格**

1. **LiDARデバイス確認**
   - TSB-EKF-014（シリアルポート不安定化）は時間経過で自然復旧
   - `/dev/rplidar` → `ttyUSB0` 正常認識、USB再接続不要だった

2. **FASTRTPS環境変数確認**
   - `.bashrc` に設定済み、`/home/pi/fastrtps_udp_only.xml` 正常

3. **LiDAR `/scan` データフロー確認 — 合格**
   - 単体起動で `/scan` ~6.8Hz安定を確認（前回未確認だった項目）

4. **全ノード統合起動 — 合格**
   - `zeuscar.launch.py use_ekf:=true use_slam:=true use_motor:=false`
   - 5ノード全て正常起動（robot_state_publisher, ekf, slam_toolbox, rplidar, imu）
   - slam_toolbox: `active [3]` に自動遷移、LiDARセンサー登録成功
   - TFツリー完全: `map→odom→base_footprint→base_link→{laser_frame, imu_link}`
   - トピック確認: `/scan` ~6.8Hz、`/imu/data_raw` ~50Hz、`/odometry/filtered` ~30Hz、`/map` 配信OK

5. **マップ生成・保存テスト — 合格**
   - 問題: `save_map` サービスが `result=255` で失敗（TSB-EKF-015）
   - 原因: `nav2-map-server` 未インストール
   - 対策: `sudo apt-get install -y ros-jazzy-nav2-map-server`
   - 再試行: `result=0`（成功）、`zeuscar_map.pgm` + `zeuscar_map.yaml` 生成確認
   - マップ仕様: 68x80px、解像度0.05m（5cm/ピクセル）

6. **ドキュメント整備**
   - トラブルシューティング: TSB-EKF-014追記（自然復旧）、TSB-EKF-015追加
   - 実装ガイド: Section 8（SLAM統合テスト結果）追加

**2026-02-10: LiDARデータ検証・VNC調査**

1. **LiDAR実データ検証**
   - USB再接続後にLiDAR単体起動で `/scan` データ中身を確認
   - 720ポイント/スキャン（360°、0.5°刻み）
   - 距離データ: 0.19m〜3.4m の範囲で室内環境を正常にスキャン
   - 近距離・中距離・遠距離の障害物検出OK、`inf`（開放空間）も正常
   - **結論: LiDARは完全に正常動作**

2. **TSB-EKF-014再発と対策**
   - 前回のSIGTERM停止後にシリアルポート不安定化が再発
   - SDK Version表示で停止する典型的な症状
   - USB再接続で即座に復旧
   - **教訓**: LiDARプロセス停止後は毎回デバイス状態を確認すること

3. **STORY-013: RealVNC構築完了**
   - RealVNC Server 7.12.0 インストール（TSB-VIS-001）
   - Wayland無効化 → X11強制（TSB-VIS-002）
   - GDM自動ログイン有効化（TSB-VIS-003）
   - ヘッドレスHDMIダミーディスプレイ設定（TSB-VIS-004、主原因）
     - `vc4-kms-v3d`（Full KMS）が `hdmi_force_hotplug` を無視する問題
     - `video=HDMI-A-1:1920x1080@60D` をcmdline.txtに追加で解決
   - VNC接続（1920x1080）でGNOMEデスクトップ表示成功
   - トラブルシューティング: `docs/operations/troubleshooting/STORY-013_vnc_visualization.md`

**2026-02-11: STORY-013 RViz可視化確認完了 — EPIC-005完了**

1. **全ノード統合起動（EKF + SLAM + LiDAR + IMU + RViz2）**
   - `zeuscar.launch.py use_ekf:=true use_slam:=true use_motor:=false use_rviz:=true`
   - 6ノード全て正常起動（robot_state_publisher, ekf, slam_toolbox, rplidar, imu, rviz2）

2. **RViz2表示確認 — 全4項目合格**
   - LaserScan（/scan）: 赤色ポイントクラウド表示OK、部屋形状・方向が正確、手かざしで即座に反応
   - TFツリー: `map→odom→base_footprint→base_link→{laser_frame, imu_link}` 全フレーム表示OK
   - RobotModel: `/robot_description`からモデル表示OK
   - Map（/map）: SLAM占有格子マップ（68x80px）表示OK

3. **データフロー確認**
   - `/scan`: Publisher 1, Subscriber 2（slam + rviz）
   - `/map`: Publisher 1, Subscriber 2
   - `/robot_description`: Publisher 1, Subscriber 1
   - TF: `odom→base_footprint` ~26.9Hz、`map→odom` ~6.6Hz

4. **バックログ更新**
   - STORY-013 → Done、EPIC-005（可視化）→ Done
   - **スプリント1の全ストーリー完了**

**2026-02-11: imu_filter_madgwick導入（TSB-VIS-005対策）**

1. **パッケージインストール**
   - `ros-jazzy-imu-filter-madgwick` (v2.1.5) インストール

2. **TDD Red→Green**
   - テスト9件追加（計30件） → 実装 → 全30テストパス
   - `imu_filter_params.yaml` 新規作成（madgwick設定）
   - `ekf_params.yaml` 更新（`/imu/data` + orientation roll/pitch有効化）
   - `sensors.launch.py` にmadgwickノード追加
   - `package.xml` に依存追加

3. **実機テスト合格**
   - 5ノード正常起動（robot_state_publisher, ekf, rplidar, imu, imu_filter_madgwick）
   - `/imu/data` にorientation付きデータ配信確認
   - EKF `/odometry/filtered` に姿勢データ反映確認

4. **ドキュメント更新**
   - 実装ガイド: Section 10をmadgwick実装記録に更新
   - セットアップガイド: Section 10.6にmadgwickインストール手順追加

**2026-02-11: ユニットテスト実行結果記録（test_ekf_launch.py）**

1. **テスト実行結果**
   - コマンド: `python3 -m pytest src/zeuscar_bringup/test/test_ekf_launch.py -v`
   - 結果: **30 passed in 1.81s**
   - ハードウェア不要（設定ファイル・launchファイルの値検証のみ）

2. **テストクラス構成（6クラス）**
   - TestEkfParamsFile (15件): ekf_params.yaml の内容検証
   - TestOdometryLaunchFile (5件): odometry.launch.py の構造検証
   - TestZeuscarEkfArgument (2件): zeuscar.launch.py の use_ekf 引数
   - TestImuFilterParamsFile (5件): imu_filter_params.yaml の検証（madgwick関連）
   - TestSensorsLaunchMadgwick (1件): sensors.launch.py の madgwick ノード
   - TestPackageXmlDependency (2件): package.xml の依存確認（robot_localization, imu_filter_madgwick）

3. **ドキュメント更新**
   - 実装ガイド: ユニットテスト実行手順・結果を追記
   - トラブルシューティング: テスト実行方法セクション追記

---

### 次回再開時のアクション

#### 優先度: 低（将来対応）

1. **ホイールオドメトリの統合**
   - モーターエンコーダからオドメトリ計算
   - IMU + ホイールオドメトリのセンサーフュージョン

2. **ナビゲーション（Nav2）の導入**
   - 自律走行のためのNav2スタック構築

---

### 現在の環境状態

```
インストール済みパッケージ:
- ros-jazzy-desktop
- ros-jazzy-rplidar-ros
- ros-jazzy-xacro
- ros-jazzy-slam-toolbox
- ros-jazzy-robot-localization (STORY-011)
- ros-jazzy-nav2-map-server (STORY-011、マップ保存に必要)
- ros-jazzy-imu-filter-madgwick (TSB-VIS-005対策、IMU姿勢推定)
- i2c-tools
- python3-pip
- smbus2（pip）

ビルド済みパッケージ:
- zeuscar_bringup（EKF統合済み）
- zeuscar_description
- zeuscar_lidar
- zeuscar_slam（LifecycleNode対応済み）
- zeuscar_motor（Arduino駆動系）
- zeuscar_imu（IMU統合）

動作確認済み:
- LiDAR（/scanトピック、~6.6Hz安定）
- TF（base_footprint → base_link → laser_frame, imu_link）
- robot_state_publisher
- zeuscar_motor単体テスト（13件パス）
- zeuscar_motor実機テスト（全10方向動作確認済み）
- IMU検出（WHO_AM_I = 0x47）
- IMUジャイロスコープ（旋回検出OK）
- IMU加速度X軸（前進/後退検出OK）
- IMU加速度Z軸（+1.009g、スケールファクター修正後に正常動作確認）
- zeuscar_imu imu_publish_node（/imu/data_raw、~50Hz安定）
- 統合bringup実機テスト S0-S5 全合格（55/55テストパス）
- zeuscar.launch.py一発起動で全4ノード正常動作確認済み
- EKF単体テスト合格（odom→base_footprint TF配信確認）  ← NEW
- slam_toolbox LifecycleNode active [3] 到達確認  ← NEW

動作確認済み（STORY-011統合テスト 2026-02-10）:
- 全ノード統合起動（EKF + SLAM + LiDAR + IMU）正常動作
- /scan データフロー ~6.8Hz安定
- slam_toolbox active [3]、/map配信、map→odom TF ~6.9Hz
- マップ保存（save_mapサービス → .pgm + .yaml）成功

動作確認済み（STORY-013 RViz可視化 2026-02-11）:
- RViz2で LaserScan/TF/RobotModel/Map 全4項目表示確認合格
- 6ノード同時稼働（robot_state_publisher, ekf, slam_toolbox, rplidar, imu, rviz2）

既知の制限事項:
- TSB-VIS-005: EKFドリフトによる点群表示ずれ
  → imu_filter_madgwick導入でroll/pitchドリフト抑制済み（2026-02-11）
  → yawドリフトは磁力計なしのため残存、slam_toolboxが補正
  → ホイールオドメトリ統合で更に改善可能

解決済み:
- DDS通信障害（TSB-EKF-013: FastRTPS UDP専用プロファイルで解決）
- rplidar ノード名問題（TSB-EKF-012: rplidar_compositionに修正済み）
- LiDARシリアルポート不安定化（TSB-EKF-014: 時間経過で自然復旧確認）
- save_map失敗（TSB-EKF-015: nav2-map-server導入で解決）
- VNC環境構築（TSB-VIS-001〜004: 全て解決済み）

新規インストール:
- realvnc-vnc-server 7.12.0（VNCリモートデスクトップ）

VNC接続:
- RealVNC Server稼働中（192.168.11.20:5900、1920x1080）
- ヘッドレス環境対応済み（cmdline.txtダミーディスプレイ設定）

注意事項:
- 非対話シェルではFASTRTPS_DEFAULT_PROFILES_FILEを明示的にexportすること
- LiDARプロセス停止後は毎回デバイス状態を確認すること（TSB-EKF-014教訓）
- RViz2はRPi4で重いため、可能ならリモートPC側で実行すること
```

#### PMブリーフ管理

| ID | 概要 | ステータス |
|---|---|---|
| TSB-EKF-013 | FastRTPS DDS通信障害（RPi4 Jazzy） | 解決済み |
| TSB-EKF-014 | LiDARシリアルポート不安定化 | 解決済み（自然復旧確認） |
| TSB-EKF-015 | save_map失敗（nav2-map-server未インストール） | 解決済み |
| TSB-VIS-001 | RealVNC Server未インストール（Ubuntu 24.04） | 解決済み |
| TSB-VIS-002 | Wayland有効でVNC黒画面 | 解決済み |
| TSB-VIS-003 | GUIセッション未ログインでVNC黒画面 | 解決済み |
| TSB-VIS-004 | ヘッドレスHDMI未検出（KMSドライバ問題） | 解決済み |

---

## 過去のスプリント

（スプリント完了後にここへ移動）

---

## 更新履歴

| 日付 | 更新者 | 内容 |
|---|---|---|
| 2026-01-12 | - | 初版作成、プロジェクト初期化フェーズ記録 |
| 2026-01-12 | - | スプリント1開始、EPIC-001完了記録 |
| 2026-01-12 | - | EPIC-002 LiDAR統合進捗を記録 |
| 2026-01-19 | - | EPIC-003 TF/URDF設計完了（ビルド・検証含む） |
| 2026-01-19 | - | EPIC-002 LiDAR統合完了（/scanトピック動作確認） |
| 2026-01-19 | - | EPIC-004/005 SLAM・可視化準備（設定ファイル作成） |
| 2026-01-24 | - | EPIC-006 Arduino駆動系統合開始（仕様書作成） |
| 2026-01-24 | - | EPIC-006 Arduino駆動系統合完了（実機テスト成功） |
| 2026-02-03 | - | EPIC-007 IMU統合開始（テスト1回目、スケールファクター問題検出・修正） |
| 2026-02-04 | - | EPIC-007 IMUテスト2回目完了（全項目合格、STORY-023/024 Done） |
| 2026-02-07 | - | STORY-025 IMUパブリッシュノード実装完了（17/17テストパス） |
| 2026-02-07 | - | STORY-014/015 統合bringup設計・実装（51/51テストパス） |
| 2026-02-08 | - | 実機統合テスト S0-S5 全合格、TSB-INT-003対策（TimerAction導入、55/55テストパス） |
| 2026-02-09 | - | STORY-011開始: EKF実装（76テストパス）、slam_toolbox LifecycleNode対応、DDS障害→解決、LiDARノード名修正 |
| 2026-02-10 | - | STORY-011完了: SLAM統合テスト全項目合格、マップ保存成功、nav2-map-serverインストール、TSB-EKF-015追加 |
| 2026-02-10 | - | STORY-013準備: RealVNC Server構築（TSB-VIS-001〜004解決）、VNC経由デスクトップ表示成功 |
| 2026-02-11 | - | STORY-013完了: RViz2全4項目合格（LaserScan/TF/RobotModel/Map）、EPIC-005完了、スプリント1全ストーリー完了 |
| 2026-02-11 | - | imu_filter_madgwick導入（TSB-VIS-005対策）: TDD 30/30テストパス、実機テスト合格 |
| 2026-02-11 | - | ユニットテスト実行結果記録: test_ekf_launch.py 30テスト全パス（1.81s）、6クラス構成、ハードウェア不要 |
